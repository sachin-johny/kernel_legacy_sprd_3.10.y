/*
 * Copyright (C) 2013 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <mach/gpio.h>
#include <linux/headset_sprd.h>
#include <mach/board.h>
#include <linux/input.h>

#include <mach/adc.h>
#include <asm/io.h>
#include <mach/hardware.h>
#include <mach/adi.h>
#include <linux/module.h>

#ifdef CONFIG_ARCH_SCX35
#include <linux/regulator/consumer.h>
#include <mach/regulator.h>
#include <mach/sci_glb_regs.h>
#include <mach/arch_misc.h>
#endif

//#define SPRD_HEADSET_DBG
#define SPRD_HEADSET_SYS_SUPPORT
//#define SPRD_HEADSET_HEADMICBIAS_POLLING
//#define SPRD_HEADSET_REG_DUMP

#ifdef SPRD_HEADSET_DBG
#define ENTER printk(KERN_INFO "[SPRD_HEADSET_DBG][%d] func: %s  line: %04d\n", adie_type, __func__, __LINE__);
#define PRINT_DBG(format,x...)  printk(KERN_INFO "[SPRD_HEADSET_DBG][%d] " format, adie_type, ## x)
#define PRINT_INFO(format,x...)  printk(KERN_INFO "[SPRD_HEADSET_INFO][%d] " format, adie_type, ## x)
#define PRINT_WARN(format,x...)  printk(KERN_INFO "[SPRD_HEADSET_WARN][%d] " format, adie_type, ## x)
#define PRINT_ERR(format,x...)  printk(KERN_ERR "[SPRD_HEADSET_ERR][%d] func: %s  line: %04d  info: " format, adie_type, __func__, __LINE__, ## x)
#else
#define ENTER
#define PRINT_DBG(format,x...)
#define PRINT_INFO(format,x...)  printk(KERN_INFO "[SPRD_HEADSET_INFO][%d] " format, adie_type, ## x)
#define PRINT_WARN(format,x...)  printk(KERN_INFO "[SPRD_HEADSET_WARN][%d] " format, adie_type, ## x)
#define PRINT_ERR(format,x...)  printk(KERN_ERR "[SPRD_HEADSET_ERR][%d] func: %s  line: %04d  info: " format, adie_type, __func__, __LINE__, ## x)
#endif

#define DELAY_AFTER_POWER_ON 150

#define ADC_FIFO_CNT (5)
#define ADC_GND (100)
#define DEBANCE_LOOP_COUNT_TYPE_DETECT (5)
#define DEBANCE_LOOP_COUNT_BUTTON_DETECT (1)
#define ADC_CHECK_INTERVAL_TYPE_DETECT (5)
#define ADC_CHECK_INTERVAL_BUTTON_DETECT (0)
#define ADC_DEBANCE_VALUE (100)

#ifdef CONFIG_ARCH_SCX35
#define HEADMIC_DETECT_BASE	(ANA_AUDCFGA_INT_BASE)
#else
#define HEADMIC_DETECT_BASE	(SPRD_MISC_BASE	+ 0x700)
#endif
#define HEADMIC_DETECT_REG(X)   (HEADMIC_DETECT_BASE + (X))

#ifdef CONFIG_ARCH_SCX35
#define HEADMIC_BUTTON_BASE	(ANA_HDT_INT_BASE)
#define HEADMIC_BUTTON_REG(X)   (HEADMIC_BUTTON_BASE + (X))
#define HID_CFG0 (0x0080)
#define HID_CFG2 (0x0088)
#define HID_CFG3 (0x008C)
#define HID_CFG4 (0x0090)
#else
#define HEADMIC_BUTTON_BASE	(SPRD_MISC_BASE	+ 0xe00)
#define HEADMIC_BUTTON_REG(X)   (HEADMIC_BUTTON_BASE + (X))
#define HID_CFG0 (0x0034)
#define HID_CFG2 (0x003C)
#define HID_CFG3 (0x0040)
#define HID_CFG4 (0x0044)
#endif
#define HEADMIC_DETECT_GLB_REG(X)   (ANA_REGS_GLB_BASE + (X))

#define HEADMIC_DETECT_INSRT_VOL_SHIFT  (5)
#define HEADMIC_DETECT_INSRT_VOL_MSK    (0x3 << HEADMIC_DETECT_INSRT_VOL_SHIFT)

#define HEADMIC_DETECT_INSRT_2P1V (3)
#define HEADMIC_DETECT_INSRT_2P3V (2)
#define HEADMIC_DETECT_INSRT_2P5V (1)
#define HEADMIC_DETECT_INSRT_2P7V (0)

#define HEADMIC_ADC_SWITCH_BIT (BIT(13))
#define HEADMIC_DETECT_CIRCUIT_BIT (BIT(11))

#define HEADMIC_DET_ADC_BUF (BIT(15))
#define HEADMIC_DET_ADC_EN (BIT(14))

#define ABS(x) (((x) < (0)) ? (-x) : (x))

#define	headset_reg_read(addr)	\
    do {	\
	sci_adi_read(addr);	\
} while(0)

#define headset_reg_msk_or(val, addr, msk)  \
    do {    \
        uint32_t temp;    \
        temp = sci_adi_read(addr);  \
        temp = (temp & (~msk)) | val;   \
        sci_adi_raw_write(addr, temp);    \
    } while(0)

#define headset_reg_clr_bit(addr, bit)   \
    do {    \
        uint32_t temp;    \
        temp = sci_adi_read(addr);  \
        temp = temp & (~bit);   \
        sci_adi_raw_write(addr, temp);  \
    } while(0)

#define headset_reg_set_bit(addr, bit)   \
    do {    \
        uint32_t temp;    \
        temp = sci_adi_read(addr);  \
        temp = temp | bit;  \
        sci_adi_raw_write(addr, temp);  \
    } while(0)

typedef enum sprd_headset_type {
        HEADSET_NORMAL,
        HEADSET_NO_MIC,
        HEADSET_NORTH_AMERICA,
        HEADSET_APPLE,
        HEADSET_TYPE_MAX,
        HEADSET_TYPE_ERR = -1,
} SPRD_HEADSET_TYPE;

#ifdef SPRD_HEADSET_REG_DUMP
static struct delayed_work reg_dump_work;
static struct workqueue_struct *reg_dump_work_queue;
#endif

/***polling ana_sts0 to avoid the hardware defect***/
static struct delayed_work sts_check_work;
static struct workqueue_struct *sts_check_work_queue;
static int plug_status_when_clg_on = 1;//default is plug in
static int sts_check_work_need_to_cancel = 1;
/***polling ana_sts0 to avoid the hardware defect***/

static DEFINE_SPINLOCK(headmic_bias_lock);
static int adie_type = 0; //1=AC, 2=BA, 3=BB
static int gpio_detect_value = 0;
static int gpio_button_value = 0;
static int active_status = 1;
static int adie_chip_id = 0;
static int headset_button_pressed = 0;
static int plug_status = 0; //if the hardware detected the headset is plug in, set plug_status = 1
static struct semaphore headset_sem;
static struct sprd_headset headset = {
        .sdev = {
                .name = "h2w",
        },
};

extern int sprd_codec_headmic_bias_control(int on);

/*  on = 0: open headmic detect circuit */
static void headset_detect_circuit(unsigned on)
{
        if (on) {
                headset_reg_clr_bit(HEADMIC_DETECT_REG(0xA0), HEADMIC_DETECT_CIRCUIT_BIT);
        } else {
                headset_reg_set_bit(HEADMIC_DETECT_REG(0xA0), HEADMIC_DETECT_CIRCUIT_BIT);
        }
}

static void headset_detect_clk_en(void)
{
#ifdef CONFIG_ARCH_SCX35
        //address:0x4003_8800+0x00 (for shark) AUD_EN (audio module en) & HDT_EN (audio HDT module en)
        headset_reg_set_bit(HEADMIC_DETECT_GLB_REG(0x00), (BIT(4) | BIT(5)));
        //address:0x4003_8800+0x04 (for shark) CLK_AUD_HBD_en & CLK_AUD_HID_en
        headset_reg_set_bit(HEADMIC_DETECT_GLB_REG(0x04), (BIT(4) | BIT(5)));
#else
        //address:0x8200_0800+0x84 (for 7710) CLK_AUD_HID_en & CLK_AUD_HBD_en
        headset_reg_set_bit(HEADMIC_DETECT_GLB_REG(0x84), (BIT(14) | BIT(15)));
#endif
}

static void headset_detect_init(void)
{
        headset_detect_clk_en();
        headset_reg_set_bit(HEADMIC_DETECT_REG(0xA0), (HEADMIC_DET_ADC_BUF | HEADMIC_DET_ADC_EN));
        /* set headset detect voltage */
        headset_reg_msk_or(HEADMIC_DETECT_INSRT_2P1V, HEADMIC_DETECT_REG(0xA0), HEADMIC_DETECT_INSRT_VOL_MSK);
}

/* is_set = 1, headset_mic to AUXADC */
static void set_adc_to_headmic(unsigned is_set)
{
        if (is_set) {
                headset_reg_set_bit(HEADMIC_DETECT_REG(0xA0), HEADMIC_ADC_SWITCH_BIT);
        } else {
                headset_reg_clr_bit(HEADMIC_DETECT_REG(0xA0), HEADMIC_ADC_SWITCH_BIT);
        }
}

static void headset_mic_level(int level)
{
        if (level)
                headset_reg_msk_or(0x02, HEADMIC_DETECT_REG(0xA0), 0x1e);
        else
                headset_reg_msk_or(0x16, HEADMIC_DETECT_REG(0xA0), 0x1e);
}

#ifdef SPRD_HEADSET_HEADMICBIAS_POLLING
static void headset_micbias_polling_en(int en)
{
        if(en) {
                //step 1: enable clk for register accessable
                headset_detect_clk_en();
                //step 2: start polling & set timer
                headset_reg_msk_or(0x0020, HEADMIC_BUTTON_REG(HID_CFG2), 0x03E0);//step for T1 & T2 [9:5]
                headset_reg_msk_or(0x000F, HEADMIC_BUTTON_REG(HID_CFG2), 0x001F);//T0 timer count [4:0]
                headset_reg_msk_or(0x000F, HEADMIC_BUTTON_REG(HID_CFG3), 0xFFFF);//T1 timer count [15:0]
                headset_reg_msk_or(0x000F, HEADMIC_BUTTON_REG(HID_CFG4), 0xFFFF);//T2 timer count [15:0]
                headset_reg_msk_or(0x0001, HEADMIC_BUTTON_REG(HID_CFG0), 0x0001);//polling enable [0]
                //step 3: disable headmicbias
                headset_reg_clr_bit(HEADMIC_DETECT_REG(0x40), BIT(5));
                //headset_reg_set_bit(HEADMIC_DETECT_REG(0x40), BIT(1));
                PRINT_INFO("headmicbias polling enable\n");
                PRINT_DBG("ANA_CFG0(0x%08X)  HID_CFG0(0x%08X)  HID_CFG2(0x%08X)  HID_CFG3(0x%08X)  HID_CFG4(0x%08X)\n",
                          sci_adi_read(HEADMIC_DETECT_REG(0x40)),
                          sci_adi_read(HEADMIC_BUTTON_REG(HID_CFG0)),
                          sci_adi_read(HEADMIC_BUTTON_REG(HID_CFG2)),
                          sci_adi_read(HEADMIC_BUTTON_REG(HID_CFG3)),
                          sci_adi_read(HEADMIC_BUTTON_REG(HID_CFG4)));
        } else {
                //step 1: enable headmicbias
                //headset_reg_clr_bit(HEADMIC_DETECT_REG(0x40), BIT(1));
                headset_reg_set_bit(HEADMIC_DETECT_REG(0x40), BIT(5));
                //step 2: stop polling
                headset_reg_clr_bit(HEADMIC_BUTTON_REG(HID_CFG0), 0x0001);
                PRINT_INFO("headmicbias polling disable\n");
                PRINT_DBG("ANA_CFG0(0x%08X)  HID_CFG0(0x%08X)  HID_CFG2(0x%08X)  HID_CFG3(0x%08X)  HID_CFG4(0x%08X)\n",
                          sci_adi_read(HEADMIC_DETECT_REG(0x40)),
                          sci_adi_read(HEADMIC_BUTTON_REG(HID_CFG0)),
                          sci_adi_read(HEADMIC_BUTTON_REG(HID_CFG2)),
                          sci_adi_read(HEADMIC_BUTTON_REG(HID_CFG3)),
                          sci_adi_read(HEADMIC_BUTTON_REG(HID_CFG4)));
        }
}
#endif

static void headset_irq_button_enable(int enable, unsigned int irq)
{
        static int current_irq_state = 1;//irq is enabled after request_irq()

        if (1 == enable) {
                if (0 == current_irq_state) {
                        enable_irq(irq);
                        current_irq_state = 1;
                        return;
                }
        } else {
                if (1 == current_irq_state) {
                        disable_irq_nosync(irq);
                        current_irq_state = 0;
                        return;
                }
        }
}

static void headset_irq_detect_enable(int enable, unsigned int irq)
{
        static int current_irq_state = 1;//irq is enabled after request_irq()

        if (1 == enable) {
                if (0 == current_irq_state) {
                        enable_irq(irq);
                        current_irq_state = 1;
                        return;
                }
        } else {
                if (1 == current_irq_state) {
                        disable_irq_nosync(irq);
                        current_irq_state = 0;
                        return;
                }
        }
}

static void headmicbias_power_on(int on)
{
        unsigned long spin_lock_flags;
        static int current_power_state = 0;

        spin_lock_irqsave(&headmic_bias_lock, spin_lock_flags);
        if (1 == on) {
                if (0 == current_power_state) {
                        sprd_codec_headmic_bias_control(1);
                        current_power_state = 1;
                }
        } else {
                if (1 == current_power_state) {
                        sprd_codec_headmic_bias_control(0);
                        current_power_state = 0;
                }
        }
        spin_unlock_irqrestore(&headmic_bias_lock, spin_lock_flags);

        return;
}

static int array_get_min(int* array, int size)
{
        int ret = 0;
        int i = 0;

        ret =  array[0];
        for(i=1; i<size; i++) {
                if(ret > array[i])
                        ret = array[i];
        }

        return ret;
}

static int array_get_max(int* array, int size)
{
        int ret = 0;
        int i = 0;

        ret =  array[0];
        for(i=1; i<size; i++) {
                if(ret < array[i])
                        ret = array[i];
        }

        return ret;
}

static SPRD_HEADSET_TYPE headset_type_detect(int last_gpio_detect_value)
{
        struct sprd_headset *ht = &headset;
        struct sprd_headset_platform_data *pdata = ht->platform_data;
        int i = 0;
        int j = 0;
        int adc_value = 0;
        int adc_mic_average = 0;
        int adc_left_average = 0;
        int adc_mic[DEBANCE_LOOP_COUNT_TYPE_DETECT] = {0};
        int adc_left[DEBANCE_LOOP_COUNT_TYPE_DETECT] = {0};
        int adc_max = 0;
        int adc_min = 0;
        int state = 0;

        ENTER
        gpio_direction_output(pdata->gpio_switch, 0);
        headset_mic_level(1);
        headset_detect_init();
        headset_detect_circuit(1);

        //get adc value of mic
        set_adc_to_headmic(1);
        msleep(50);
        for(i=0; i< DEBANCE_LOOP_COUNT_TYPE_DETECT; i++) {
                for (j = 0; j < ADC_FIFO_CNT; j++) {
                        adc_value = sci_adc_get_value(ADC_CHANNEL_HEADMIC, 0);
                        PRINT_DBG("adc_value[%d] = %d\n", j, adc_value);
                        adc_mic[i] += adc_value;
                        msleep(ADC_CHECK_INTERVAL_TYPE_DETECT);
                }
                adc_mic[i] = adc_mic[i] /ADC_FIFO_CNT;
                adc_mic_average += adc_mic[i];
                PRINT_DBG("the average adc_mic[%d] = %d\n", i, adc_mic[i]);

                state = gpio_get_value(pdata->gpio_detect);
                if(state != last_gpio_detect_value) {
                        PRINT_INFO("software debance (step 2: gpio check)!!!(headset_type_detect)(mic)\n");
                        goto out;
                }
        }
        adc_max = array_get_max(adc_mic, DEBANCE_LOOP_COUNT_TYPE_DETECT);
        adc_min = array_get_min(adc_mic, DEBANCE_LOOP_COUNT_TYPE_DETECT);
        if((adc_max - adc_min) > ADC_DEBANCE_VALUE) {
                PRINT_INFO("software debance (step 3: adc check)!!!(headset_type_detect)(mic)\n");
                goto out;
        }
        adc_mic_average = adc_mic_average / DEBANCE_LOOP_COUNT_TYPE_DETECT;

        //get adc value of left
        set_adc_to_headmic(0);
        msleep(50);
        for(i=0; i< DEBANCE_LOOP_COUNT_TYPE_DETECT; i++) {
                for (j = 0; j < ADC_FIFO_CNT; j++) {
                        adc_value = sci_adc_get_value(ADC_CHANNEL_HEADMIC, 0);
                        PRINT_DBG("adc_value[%d] = %d\n", j, adc_value);
                        adc_left[i] += adc_value;
                        msleep(ADC_CHECK_INTERVAL_TYPE_DETECT);
                }
                adc_left[i] = adc_left[i] /ADC_FIFO_CNT;
                adc_left_average += adc_left[i];
                PRINT_DBG("the average adc_left[%d] = %d\n", i, adc_left[i]);

                state = gpio_get_value(pdata->gpio_detect);
                if(state != last_gpio_detect_value) {
                        PRINT_INFO("software debance (step 2: gpio check)!!!(headset_type_detect)(left)\n");
                        goto out;
                }
        }
        adc_max = array_get_max(adc_left, DEBANCE_LOOP_COUNT_TYPE_DETECT);
        adc_min = array_get_min(adc_left, DEBANCE_LOOP_COUNT_TYPE_DETECT);
        if((adc_max - adc_min) > ADC_DEBANCE_VALUE) {
                PRINT_INFO("software debance (step 3: adc check)!!!(headset_type_detect)(left)\n");
                goto out;
        }
        adc_left_average = adc_left_average / DEBANCE_LOOP_COUNT_TYPE_DETECT;

        PRINT_INFO("adc_mic_average = %d\n", adc_mic_average);
        PRINT_INFO("adc_left_average = %d\n", adc_left_average);

        if((adc_left_average < ADC_GND) && (adc_mic_average < ADC_GND))
                return HEADSET_NO_MIC;
        else if((adc_left_average < ADC_GND) && (adc_mic_average > ADC_GND))
                return HEADSET_NORMAL;
        else if((adc_left_average > ADC_GND) && (adc_mic_average > ADC_GND)
                        && (ABS(adc_mic_average - adc_left_average) < ADC_GND))
                return HEADSET_NORTH_AMERICA;
        else
                return HEADSET_TYPE_ERR;
out:
        headset_irq_detect_enable(1, ht->irq_detect);
        return HEADSET_TYPE_ERR;
}

static void headset_button_work_func(struct work_struct *work)
{
        struct sprd_headset *ht = &headset;
        struct sprd_headset_platform_data *pdata = ht->platform_data;
        int state;
        int adc_mic[DEBANCE_LOOP_COUNT_BUTTON_DETECT] = {0};
        int adc_mic_average = 0;
        int adc_value = 0;
        int adc_max = 0;
        int adc_min = 0;
        int i = 0;
        int j = 0;
        static int current_key_code = KEY_RESERVED;

        down(&headset_sem);

        ENTER

        state = gpio_get_value(pdata->gpio_button);
        if(state != gpio_button_value) {
                PRINT_INFO("software debance (step 1: gpio check)!!!(headset_button_work_func)\n");
                goto out;
        }

        if(state) {//pressed!
                for(i=0; i< DEBANCE_LOOP_COUNT_BUTTON_DETECT; i++) {
                        for (j = 0; j < ADC_FIFO_CNT; j++) {
                                adc_value = sci_adc_get_value(ADC_CHANNEL_HEADMIC, 0);
                                PRINT_DBG("adc_value[%d] = %d\n", j, adc_value);
                                adc_mic[i] += adc_value;
                                msleep(ADC_CHECK_INTERVAL_BUTTON_DETECT);
                        }
                        adc_mic[i] = adc_mic[i] /ADC_FIFO_CNT;
                        adc_mic_average += adc_mic[i];
                        PRINT_DBG("the average adc_mic[%d] = %d\n", i, adc_mic[i]);

                        state = gpio_get_value(pdata->gpio_button);
                        if(state != gpio_button_value) {
                                PRINT_INFO("software debance (step 2: gpio check)!!!(headset_button_work_func)(pressed)\n");
                                goto out;
                        }
                }

                adc_max = array_get_max(adc_mic, DEBANCE_LOOP_COUNT_BUTTON_DETECT);
                adc_min = array_get_min(adc_mic, DEBANCE_LOOP_COUNT_BUTTON_DETECT);
                if((adc_max - adc_min) > ADC_DEBANCE_VALUE) {
                        PRINT_INFO("software debance (step 3: adc check)!!!(headset_button_work_func)(pressed)\n");
                        goto out;
                }

                adc_mic_average = adc_mic_average / DEBANCE_LOOP_COUNT_BUTTON_DETECT;
                PRINT_INFO("adc_mic_average = %d\n", adc_mic_average);
                for (i = 0; i < ht->platform_data->nbuttons; i++) {
                        if (adc_mic_average >= ht->platform_data->headset_buttons[i].adc_min &&
                            adc_mic_average < ht->platform_data->headset_buttons[i].adc_max) {
                                current_key_code = ht->platform_data->headset_buttons[i].code;
                                break;
                        }
                        current_key_code = KEY_RESERVED;
                }

                if(0 == headset_button_pressed) {
                        input_event(ht->input_dev, EV_KEY, current_key_code, 1);
                        input_sync(ht->input_dev);
                        headset_button_pressed = 1;
                        PRINT_INFO("headset button pressed! current_key_code = %d(0x%04X)\n", current_key_code, current_key_code);
                } else
                        PRINT_ERR("headset button pressed already! current_key_code = %d(0x%04X)\n", current_key_code, current_key_code);

                if (1 == ht->platform_data->irq_trigger_level_button) {
                        irq_set_irq_type(ht->irq_button, IRQF_TRIGGER_LOW);
                } else {
                        irq_set_irq_type(ht->irq_button, IRQF_TRIGGER_HIGH);
                }
                headset_irq_button_enable(1, ht->irq_button);
        } else { //released!
                for(i=0; i< DEBANCE_LOOP_COUNT_BUTTON_DETECT; i++) {
                        for (j = 0; j < ADC_FIFO_CNT; j++) {
                                adc_value = sci_adc_get_value(ADC_CHANNEL_HEADMIC, 0);
                                PRINT_DBG("adc_value[%d] = %d\n", j, adc_value);
                                adc_mic[i] += adc_value;
                                msleep(ADC_CHECK_INTERVAL_BUTTON_DETECT);
                        }
                        adc_mic[i] = adc_mic[i] /ADC_FIFO_CNT;
                        PRINT_DBG("the average adc_mic[%d] = %d\n", i, adc_mic[i]);

                        state = gpio_get_value(pdata->gpio_button);
                        if(state != gpio_button_value) {
                                PRINT_INFO("software debance (step 2: gpio check)!!!(headset_button_work_func)(released)\n");
                                goto out;
                        }
                }

                adc_max = array_get_max(adc_mic, DEBANCE_LOOP_COUNT_BUTTON_DETECT);
                adc_min = array_get_min(adc_mic, DEBANCE_LOOP_COUNT_BUTTON_DETECT);
                if((adc_max - adc_min) > ADC_DEBANCE_VALUE) {
                        PRINT_INFO("software debance (step 3: adc check)!!!(headset_button_work_func)(released)\n");
                        goto out;
                }

                if(1 == headset_button_pressed) {
                        input_event(ht->input_dev, EV_KEY, current_key_code, 0);
                        input_sync(ht->input_dev);
                        headset_button_pressed = 0;
                        PRINT_INFO("headset button released! current_key_code = %d(0x%04X)\n", current_key_code, current_key_code);
                } else
                        PRINT_ERR("headset button released already! current_key_code = %d(0x%04X)\n", current_key_code, current_key_code);

                if (1 == ht->platform_data->irq_trigger_level_button) {
                        irq_set_irq_type(ht->irq_button, IRQF_TRIGGER_HIGH);
                } else {
                        irq_set_irq_type(ht->irq_button, IRQF_TRIGGER_LOW);
                }
                headset_irq_button_enable(1, ht->irq_button);
        }
out:
        headset_irq_button_enable(1, ht->irq_button);
        up(&headset_sem);
        return;
}

static void headset_detect_work_func(struct work_struct *work)
{
        struct sprd_headset *ht = &headset;
        struct sprd_headset_platform_data *pdata = ht->platform_data;
        SPRD_HEADSET_TYPE headset_type;
        int state = 0;
        int ana_sts0 = 0;

        down(&headset_sem);

        ENTER

        //headmicbias_power_on(1);

        state = gpio_get_value(pdata->gpio_detect);
        PRINT_INFO("state = %d, plug_status = %d, gpio_detect_value = %d\n", state, plug_status, gpio_detect_value);

        if(state != gpio_detect_value) {
                PRINT_INFO("software debance (step 1)!!!(headset_detect_work_func)\n");
                goto out;
        }

        if(state == pdata->irq_trigger_level_detect) {//is plug in! only plug in needs to enter debance step2
                msleep(100);
                ana_sts0 = sci_adi_read(ANA_AUDCFGA_INT_BASE+0xC0);//arm base address:0x40038600
                if(((0x00000060 & ana_sts0) != 0x00000060)) {
                        PRINT_INFO("software debance (step 2: Just For Insert)!!!(headset_detect_work_func)\n");
                        goto out;
                }
        }

        if(1 == state && 0 == plug_status) {
                headset_type = headset_type_detect(gpio_detect_value);
                switch (headset_type) {
                case HEADSET_TYPE_ERR:
                        PRINT_INFO("headset_type = %d (HEADSET_TYPE_ERR)\n", headset_type);
                        goto out;
                case HEADSET_NORTH_AMERICA:
                        PRINT_INFO("headset_type = %d (HEADSET_NORTH_AMERICA)\n", headset_type);
                        gpio_direction_output(pdata->gpio_switch, 1);
                        break;
                case HEADSET_NORMAL:
                        PRINT_INFO("headset_type = %d (HEADSET_NORMAL)\n", headset_type);
                        gpio_direction_output(pdata->gpio_switch, 0);
                        break;
                case HEADSET_NO_MIC:
                        PRINT_INFO("headset_type = %d (HEADSET_NO_MIC)\n", headset_type);
                        gpio_direction_output(pdata->gpio_switch, 0);
                        break;
                case HEADSET_APPLE:
                        PRINT_INFO("headset_type = %d (HEADSET_APPLE)\n", headset_type);
                        PRINT_INFO("we have not yet implemented this in the code\n");
                        break;
                default:
                        PRINT_INFO("headset_type = %d (HEADSET_UNKNOWN)\n", headset_type);
                        break;
                }

                if(headset_type == HEADSET_NO_MIC)
                        ht->headphone = 1;
                else
                        ht->headphone = 0;

                if (ht->headphone) {
                        headset_mic_level(0);
                        headset_irq_button_enable(0, ht->irq_button);

                        ht->type = BIT_HEADSET_NO_MIC;
                        switch_set_state(&ht->sdev, ht->type);
                        PRINT_INFO("headphone plug in (headset_detect_work_func)\n");
                } else {
                        if (1 == ht->platform_data->irq_trigger_level_button) {
                                irq_set_irq_type(ht->irq_button, IRQF_TRIGGER_HIGH);
                        } else {
                                irq_set_irq_type(ht->irq_button, IRQF_TRIGGER_LOW);
                        }
                        headset_mic_level(1);
                        headset_irq_button_enable(1, ht->irq_button);

                        ht->type = BIT_HEADSET_MIC;
                        switch_set_state(&ht->sdev, ht->type);
                        PRINT_INFO("headset plug in (headset_detect_work_func)\n");
                }

                /***polling ana_sts0 to avoid the hardware defect***/
                if (0xA000 != adie_chip_id) {
                        plug_status_when_clg_on = 1;
                        sts_check_work_need_to_cancel = 0;
                        queue_delayed_work(sts_check_work_queue, &sts_check_work, msecs_to_jiffies(1000));
                }
                /***polling ana_sts0 to avoid the hardware defect***/

                plug_status = 1;
                if(1 == pdata->irq_trigger_level_detect)
                        irq_set_irq_type(ht->irq_detect, IRQF_TRIGGER_LOW);
                else
                        irq_set_irq_type(ht->irq_detect, IRQF_TRIGGER_HIGH);

                headset_irq_detect_enable(1, ht->irq_detect);
        } else if(0 == state && 1 == plug_status) {

                headset_irq_button_enable(0, ht->irq_button);

                /***polling ana_sts0 to avoid the hardware defect***/
                if (0xA000 != adie_chip_id) {
                        sts_check_work_need_to_cancel = 1;
                        if(0 == plug_status_when_clg_on) {
                                PRINT_INFO("plug out already!!!\n");
                                goto plug_out_already;
                        }
                        plug_status_when_clg_on = 0;
                }
                /***polling ana_sts0 to avoid the hardware defect***/

                //headmicbias_power_on(0);
                if (ht->headphone) {
                        PRINT_INFO("headphone plug out (headset_detect_work_func)\n");
                } else {
                        PRINT_INFO("headset plug out (headset_detect_work_func)\n");
                }
                ht->type = BIT_HEADSET_OUT;
                switch_set_state(&ht->sdev, ht->type);
plug_out_already:
                plug_status = 0;
                if(1 == pdata->irq_trigger_level_detect)
                        irq_set_irq_type(ht->irq_detect, IRQF_TRIGGER_HIGH);
                else
                        irq_set_irq_type(ht->irq_detect, IRQF_TRIGGER_LOW);

                headset_irq_detect_enable(1, ht->irq_detect);
        } else {
                PRINT_INFO("irq_detect must be enabled anyway!!!\n");
                goto out;
        }
out:
        headset_irq_detect_enable(1, ht->irq_detect);
        up(&headset_sem);
        return;
}

/***polling ana_sts0 to avoid the hardware defect***/
static void headset_sts_check_func(struct work_struct *work)
{
        int ana_sts0 = 0;
        int ana_sts0_debance = 0;
        struct sprd_headset *ht = &headset;
        struct sprd_headset_platform_data *pdata = ht->platform_data;
        SPRD_HEADSET_TYPE headset_type;

        down(&headset_sem);

        ENTER

        if(1 == sts_check_work_need_to_cancel)
                goto out;

        ana_sts0_debance = sci_adi_read(ANA_AUDCFGA_INT_BASE+0xC0);//arm base address:0x40038600
        msleep(100);
        ana_sts0 = sci_adi_read(ANA_AUDCFGA_INT_BASE+0xC0);//arm base address:0x40038600
        if((0x00000060 & ana_sts0) != (0x00000060 & ana_sts0_debance)) {
                PRINT_INFO("software debance!!!(headset_sts_check_func)\n");
                goto out;
        }

        if(((0x00000060 & ana_sts0) != 0x00000060) && (1 == plug_status_when_clg_on)) {

                headset_irq_button_enable(0, ht->irq_button);

                if (ht->headphone) {
                        PRINT_INFO("headphone plug out (headset_sts_check_func)\n");
                } else {
                        PRINT_INFO("headset plug out (headset_sts_check_func)\n");
                }
                ht->type = BIT_HEADSET_OUT;
                switch_set_state(&ht->sdev, ht->type);
                plug_status_when_clg_on = 0;
        }
        if(((0x00000060 & ana_sts0) == 0x00000060) && (0 == plug_status_when_clg_on)) {
                headset_type = headset_type_detect(gpio_get_value(pdata->gpio_detect));
                switch (headset_type) {
                case HEADSET_TYPE_ERR:
                        PRINT_INFO("headset_type = %d (HEADSET_TYPE_ERR)\n", headset_type);
                        goto out;
                case HEADSET_NORTH_AMERICA:
                        PRINT_INFO("headset_type = %d (HEADSET_NORTH_AMERICA)\n", headset_type);
                        gpio_direction_output(pdata->gpio_switch, 1);
                        break;
                case HEADSET_NORMAL:
                        PRINT_INFO("headset_type = %d (HEADSET_NORMAL)\n", headset_type);
                        gpio_direction_output(pdata->gpio_switch, 0);
                        break;
                case HEADSET_NO_MIC:
                        PRINT_INFO("headset_type = %d (HEADSET_NO_MIC)\n", headset_type);
                        gpio_direction_output(pdata->gpio_switch, 0);
                        break;
                case HEADSET_APPLE:
                        PRINT_INFO("headset_type = %d (HEADSET_APPLE)\n", headset_type);
                        PRINT_INFO("we have not yet implemented this in the code\n");
                        break;
                default:
                        PRINT_INFO("headset_type = %d (HEADSET_UNKNOWN)\n", headset_type);
                        break;
                }

                if(headset_type == HEADSET_NO_MIC)
                        ht->headphone = 1;
                else
                        ht->headphone = 0;

                if (ht->headphone) {
                        headset_mic_level(0);
                        headset_irq_button_enable(0, ht->irq_button);

                        ht->type = BIT_HEADSET_NO_MIC;
                        switch_set_state(&ht->sdev, ht->type);
                        PRINT_INFO("headphone plug in (headset_sts_check_func)\n");
                } else {
                        if (1 == ht->platform_data->irq_trigger_level_button) {
                                irq_set_irq_type(ht->irq_button, IRQF_TRIGGER_HIGH);
                        } else {
                                irq_set_irq_type(ht->irq_button, IRQF_TRIGGER_LOW);
                        }
                        headset_mic_level(1);
                        headset_irq_button_enable(1, ht->irq_button);

                        ht->type = BIT_HEADSET_MIC;
                        switch_set_state(&ht->sdev, ht->type);
                        PRINT_INFO("headset plug in (headset_sts_check_func)\n");
                }
                plug_status_when_clg_on = 1;
        }
out:
        if(0 == sts_check_work_need_to_cancel)
                queue_delayed_work(sts_check_work_queue, &sts_check_work, msecs_to_jiffies(1000));
        else
                PRINT_INFO("sts_check_work cancelled\n");
        up(&headset_sem);
        return;
}
/***polling ana_sts0 to avoid the hardware defect***/

#ifdef SPRD_HEADSET_REG_DUMP
static void reg_dump_func(struct work_struct *work)
{
        int gpio_detect = 0;
        int gpio_button = 0;

        int ana_sts0 = 0;
        int ana_cfg0 = 0;
        int ana_cfg1 = 0;
        int ana_cfg20 = 0;

        int hid_cfg0 = 0;
        int hid_cfg2 = 0;
        int hid_cfg3 = 0;
        int hid_cfg4 = 0;

        int arm_module_en = 0;
        int arm_clk_en = 0;

        gpio_detect = gpio_get_value(headset.platform_data->gpio_detect);
        gpio_button = gpio_get_value(headset.platform_data->gpio_button);

        sci_adi_write(ANA_REG_GLB_ARM_MODULE_EN, BIT_ANA_AUD_EN, BIT_ANA_AUD_EN);//arm base address:0x40038800 for register accessable
        ana_cfg0 = sci_adi_read(ANA_AUDCFGA_INT_BASE+0x40);//arm base address:0x40038600
        ana_cfg1 = sci_adi_read(ANA_AUDCFGA_INT_BASE+0x44);//arm base address:0x40038600
        ana_cfg20 = sci_adi_read(ANA_AUDCFGA_INT_BASE+0xA0);//arm base address:0x40038600
        ana_sts0 = sci_adi_read(ANA_AUDCFGA_INT_BASE+0xC0);//arm base address:0x40038600

        sci_adi_write(ANA_REG_GLB_ARM_MODULE_EN, BIT_ANA_HDT_EN, BIT_ANA_HDT_EN);//arm base address:0x40038800 for register accessable
        sci_adi_write(ANA_REG_GLB_ARM_CLK_EN, BIT_CLK_AUD_HID_EN, BIT_CLK_AUD_HID_EN);//arm base address:0x40038800 for register accessable

        hid_cfg2 = sci_adi_read(ANA_HDT_INT_BASE+0x88);//arm base address:0x40038700
        hid_cfg3 = sci_adi_read(ANA_HDT_INT_BASE+0x8C);//arm base address:0x40038700
        hid_cfg4 = sci_adi_read(ANA_HDT_INT_BASE+0x90);//arm base address:0x40038700
        hid_cfg0 = sci_adi_read(ANA_HDT_INT_BASE+0x80);//arm base address:0x40038700

        arm_module_en = sci_adi_read(ANA_REG_GLB_ARM_MODULE_EN);
        arm_clk_en = sci_adi_read(ANA_REG_GLB_ARM_CLK_EN);

        PRINT_INFO("GPIO_%03d(det)=%d    GPIO_%03d(but)=%d\n",
                   headset.platform_data->gpio_detect, gpio_detect,
                   headset.platform_data->gpio_button, gpio_button);
        PRINT_INFO("arm_module_en|arm_clk_en|ana_cfg0  |ana_cfg1  |ana_cfg20 |ana_sts0  |hid_cfg2  |hid_cfg3  |hid_cfg4  |hid_cfg0\n");
        PRINT_INFO("0x%08X   |0x%08X|0x%08X|0x%08X|0x%08X|0x%08X|0x%08X|0x%08X|0x%08X|0x%08X\n",
                   arm_module_en, arm_clk_en, ana_cfg0, ana_cfg1, ana_cfg20, ana_sts0, hid_cfg2, hid_cfg3, hid_cfg4, hid_cfg0);

        queue_delayed_work(reg_dump_work_queue, &reg_dump_work, msecs_to_jiffies(500));
        return;
}
#endif

static irqreturn_t headset_button_irq_handler(int irq, void *dev)
{
        struct sprd_headset *ht = dev;

        headset_irq_button_enable(0, ht->irq_button);
        gpio_button_value = gpio_get_value(ht->platform_data->gpio_button);
        PRINT_DBG("headset_button_irq_handler: IRQ_%d(GPIO_%d) = %d\n",
                  ht->irq_button, ht->platform_data->gpio_button, gpio_button_value);
        queue_work(ht->button_work_queue, &ht->work_button);
        return IRQ_HANDLED;
}

static irqreturn_t headset_detect_irq_handler(int irq, void *dev)
{
        struct sprd_headset *ht = dev;

        headset_irq_detect_enable(0, ht->irq_detect);
        gpio_detect_value = gpio_get_value(ht->platform_data->gpio_detect);
        PRINT_DBG("headset_detect_irq_handler: IRQ_%d(GPIO_%d) = %d\n",
                  ht->irq_detect, ht->platform_data->gpio_detect, gpio_detect_value);
        queue_delayed_work(ht->detect_work_queue, &ht->work_detect, msecs_to_jiffies(100));
        return IRQ_HANDLED;
}

#ifdef SPRD_HEADSET_SYS_SUPPORT
/***create sys fs for debug***/
static int headset_suspend(struct platform_device *dev, pm_message_t state);
static int headset_resume(struct platform_device *dev);

static ssize_t headset_suspend_show(struct kobject *kobj, struct kobj_attribute *attr, char *buff)
{
        PRINT_INFO("headset_suspend_show. current active_status = %d\n", active_status);
        return sprintf(buff, "%d\n", active_status);
}

static ssize_t headset_suspend_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buff, size_t len)
{
        pm_message_t pm_message_t_temp = {
                .event = 0,
        };
        unsigned long active = simple_strtoul(buff, NULL, 10);
        PRINT_INFO("headset_suspend_store. buff = %s\n", buff);
        PRINT_INFO("headset_suspend_store. set_val = %ld\n", active);
        if(0 == active && 1 == active_status) {
                headset_suspend(NULL, pm_message_t_temp);
                return len;
        }
        if(1 == active && 0 == active_status) {
                headset_resume(NULL);
                return len;
        }
        PRINT_ERR("suspend set ERROR!Maybe the current active state is alreay what you want!\n");
        return len;
}

static struct kobject *headset_suspend_kobj = NULL;
static struct kobj_attribute headset_suspend_attr =
        __ATTR(active, 0644, headset_suspend_show, headset_suspend_store);

static int headset_suspend_sysfs_init(void)
{
        int ret = -1;

        headset_suspend_kobj = kobject_create_and_add("headset", kernel_kobj);
        if (headset_suspend_kobj == NULL) {
                ret = -ENOMEM;
                PRINT_ERR("register sysfs failed. ret = %d\n", ret);
                return ret;
        }

        ret = sysfs_create_file(headset_suspend_kobj, &headset_suspend_attr.attr);
        if (ret) {
                PRINT_ERR("create sysfs failed. ret = %d\n", ret);
                return ret;
        }

        PRINT_INFO("headset_suspend_sysfs_init success\n");
        return ret;
}
/***create sys fs for debug***/
#endif

static __devinit int headset_detect_probe(struct platform_device *pdev)
{
        struct sprd_headset_platform_data *pdata = pdev->dev.platform_data;
        struct sprd_headset *ht = &headset;
        unsigned long irqflags = 0;
        struct input_dev *input_dev = NULL;
        int i = 0;
        int ret = -1;
        int ana_sts0 = 0;

        adie_chip_id = sci_adi_read(ANA_CTL_GLB_BASE+0x0108);//A-die chip id LOW

        sci_adi_write(ANA_REG_GLB_ARM_MODULE_EN, BIT_ANA_AUD_EN, BIT_ANA_AUD_EN);//arm base address:0x40038800 for register accessable
        ana_sts0 = sci_adi_read(ANA_AUDCFGA_INT_BASE+0xC0);//arm base address:0x40038600

        if (0xA000 == adie_chip_id)
                adie_type = 1;//AC
        else if (0xA001 == adie_chip_id) {
                if ((0x00000040 & ana_sts0) == 0x00000040)
                        adie_type = 2;//BA
                else if ((0x00000040 & ana_sts0) == 0x00000000)
                        adie_type = 3;//BB
        }
        ENTER

        ht->platform_data = pdata;
        headmicbias_power_on(1);
        msleep(5);//this time delay is necessary here
#ifdef SPRD_HEADSET_HEADMICBIAS_POLLING
        headset_micbias_polling_en(1);
#endif
        msleep(DELAY_AFTER_POWER_ON);

        PRINT_INFO("D-die chip id = 0x%08X\n", __raw_readl(REG_AON_APB_CHIP_ID));
        PRINT_INFO("A-die chip id HIGH = 0x%08X\n", sci_adi_read(ANA_CTL_GLB_BASE+0x010C));
        PRINT_INFO("A-die chip id LOW = 0x%08X\n", (adie_chip_id = sci_adi_read(ANA_CTL_GLB_BASE+0x0108)));

        if (0xA000 == adie_chip_id) {
                pdata->gpio_detect -= 1;
                PRINT_INFO("use EIC_AUD_HEAD_INST (EIC4, GPIO_%d) for insert detecting\n", pdata->gpio_detect);
        } else {
                PRINT_INFO("use EIC_AUD_HEAD_INST2 (EIC5, GPIO_%d) for insert detecting\n", pdata->gpio_detect);
        }

        ret = gpio_request(pdata->gpio_switch, "headset_switch");
        if (ret < 0) {
                PRINT_ERR("failed to request GPIO_%d(headset_switch)\n", pdata->gpio_switch);
                goto failed_to_request_gpio_switch;
        }

        ret = gpio_request(pdata->gpio_detect, "headset_detect");
        if (ret < 0) {
                PRINT_ERR("failed to request GPIO_%d(headset_detect)\n", pdata->gpio_detect);
                goto failed_to_request_gpio_detect;
        }

        ret = gpio_request(pdata->gpio_button, "headset_button");
        if (ret < 0) {
                PRINT_ERR("failed to request GPIO_%d(headset_button)\n", pdata->gpio_button);
                goto failed_to_request_gpio_button;
        }

        gpio_direction_output(pdata->gpio_switch, 0);
        gpio_direction_input(pdata->gpio_detect);
        gpio_direction_input(pdata->gpio_button);
        ht->irq_detect = gpio_to_irq(pdata->gpio_detect);
        ht->irq_button = gpio_to_irq(pdata->gpio_button);

        ret = switch_dev_register(&ht->sdev);
        if (ret < 0) {
                PRINT_ERR("switch_dev_register failed!\n");
                goto failed_to_register_switch_dev;
        }

        input_dev = input_allocate_device();
        if ( !input_dev) {
                PRINT_ERR("input_allocate_device for headset_button failed!\n");
                ret = -ENOMEM;
                goto failed_to_allocate_input_device;
        }

        input_dev->name = "headset-keyboard";
        input_dev->id.bustype = BUS_HOST;
        ht->input_dev = input_dev;

        for (i = 0; i < pdata->nbuttons; i++) {
                struct headset_buttons *buttons = &pdata->headset_buttons[i];
                unsigned int type = buttons->type ?: EV_KEY;
                input_set_capability(input_dev, type, buttons->code);
        }

        ret = input_register_device(input_dev);
        if (ret) {
                PRINT_ERR("input_register_device for headset_button failed!\n");
                goto failed_to_register_input_device;
        }

        sema_init(&headset_sem, 1);

        INIT_WORK(&ht->work_button, headset_button_work_func);
        ht->button_work_queue = create_singlethread_workqueue("headset_button");
        if(ht->button_work_queue == NULL) {
                PRINT_ERR("create_singlethread_workqueue for headset_button failed!\n");
                goto failed_to_create_singlethread_workqueue_for_headset_button;
        }

        INIT_DELAYED_WORK(&ht->work_detect, headset_detect_work_func);
        ht->detect_work_queue = create_singlethread_workqueue("headset_detect");
        if(ht->detect_work_queue == NULL) {
                PRINT_ERR("create_singlethread_workqueue for headset_detect failed!\n");
                goto failed_to_create_singlethread_workqueue_for_headset_detect;
        }

        /***polling ana_sts0 to avoid the hardware defect***/
        INIT_DELAYED_WORK(&sts_check_work, headset_sts_check_func);
        sts_check_work_queue = create_singlethread_workqueue("headset_sts_check");
        if(sts_check_work_queue == NULL) {
                PRINT_ERR("create_singlethread_workqueue for headset_sts_check failed!\n");
                goto failed_to_create_singlethread_workqueue_for_headset_sts_check;
        }
        /***polling ana_sts0 to avoid the hardware defect***/

#ifdef SPRD_HEADSET_REG_DUMP
        INIT_DELAYED_WORK(&reg_dump_work, reg_dump_func);
        reg_dump_work_queue = create_singlethread_workqueue("headset_reg_dump");
        if(reg_dump_work_queue == NULL) {
                PRINT_ERR("create_singlethread_workqueue for headset_reg_dump failed!\n");
                goto failed_to_create_singlethread_workqueue_for_headset_reg_dump;
        }
        queue_delayed_work(reg_dump_work_queue, &reg_dump_work, msecs_to_jiffies(500));
#endif

        irqflags = pdata->irq_trigger_level_button ? IRQF_TRIGGER_HIGH : IRQF_TRIGGER_LOW;
        ret = request_irq(ht->irq_button, headset_button_irq_handler, irqflags, "headset_button", ht);
        if (ret) {
                PRINT_ERR("failed to request IRQ_%d(GPIO_%d)\n", ht->irq_button, pdata->gpio_button);
                goto failed_to_request_irq_headset_button;
        }
        headset_irq_button_enable(0, ht->irq_button);//disable button irq before headset detected

        irqflags = pdata->irq_trigger_level_detect ? IRQF_TRIGGER_HIGH : IRQF_TRIGGER_LOW;
        ret = request_irq(ht->irq_detect, headset_detect_irq_handler, irqflags, "headset_detect", ht);
        if (ret < 0) {
                PRINT_ERR("failed to request IRQ_%d(GPIO_%d)\n", ht->irq_detect, pdata->gpio_detect);
                goto failed_to_request_irq_headset_detect;
        }

#ifdef SPRD_HEADSET_SYS_SUPPORT
        ret = headset_suspend_sysfs_init();
#endif

        PRINT_INFO("headset_detect_probe success\n");
        return ret;

failed_to_request_irq_headset_detect:
        free_irq(ht->irq_button, ht);
failed_to_request_irq_headset_button:

#ifdef SPRD_HEADSET_REG_DUMP
        cancel_delayed_work_sync(&reg_dump_work);
        destroy_workqueue(reg_dump_work_queue);
failed_to_create_singlethread_workqueue_for_headset_reg_dump:
#endif

        destroy_workqueue(sts_check_work_queue);
failed_to_create_singlethread_workqueue_for_headset_sts_check:
        destroy_workqueue(ht->detect_work_queue);
failed_to_create_singlethread_workqueue_for_headset_detect:
        destroy_workqueue(ht->button_work_queue);
failed_to_create_singlethread_workqueue_for_headset_button:
        input_unregister_device(input_dev);
failed_to_register_input_device:
        input_free_device(input_dev);
failed_to_allocate_input_device:
        switch_dev_unregister(&ht->sdev);
failed_to_register_switch_dev:
        gpio_free(pdata->gpio_button);
failed_to_request_gpio_button:
        gpio_free(pdata->gpio_detect);
failed_to_request_gpio_detect:
        gpio_free(pdata->gpio_switch);
failed_to_request_gpio_switch:

#ifdef SPRD_HEADSET_HEADMICBIAS_POLLING
        headset_micbias_polling_en(0);
#endif

        headmicbias_power_on(0);
        PRINT_ERR("headset_detect_probe failed\n");
        return ret;
}

#ifdef CONFIG_PM
static int headset_suspend(struct platform_device *dev, pm_message_t state)
{
        headset_irq_button_enable(0, headset.irq_button);
        headset_irq_detect_enable(0, headset.irq_detect);

        /***polling ana_sts0 to avoid the hardware defect***/
        if (0xA000 != adie_chip_id) {
                cancel_delayed_work_sync(&sts_check_work);
                sts_check_work_need_to_cancel =1;
                plug_status_when_clg_on = 0;
        }
        /***polling ana_sts0 to avoid the hardware defect***/
        plug_status = 0;

        if(BIT_HEADSET_OUT != headset.type) {
                headset.type = BIT_HEADSET_OUT;
                switch_set_state(&headset.sdev, headset.type);
                PRINT_INFO("headset plug out (headset_suspend)\n");
        }

#ifdef SPRD_HEADSET_HEADMICBIAS_POLLING
        headset_micbias_polling_en(0);
#endif
        headmicbias_power_on(0);
        active_status = 0;
        PRINT_INFO("suspend (det_irq=%d    but_irq=%d)\n", headset.irq_detect, headset.irq_button);
        return 0;
}

static int headset_resume(struct platform_device *dev)
{
        gpio_direction_output(headset.platform_data->gpio_switch, 0);
        plug_status = 0;
        if (0xA000 != adie_chip_id) {
                sts_check_work_need_to_cancel =1;
                plug_status_when_clg_on = 0;
        }

        headmicbias_power_on(1);
        msleep(5);
#ifdef SPRD_HEADSET_HEADMICBIAS_POLLING
        headset_micbias_polling_en(1);
#endif
        msleep(DELAY_AFTER_POWER_ON);
        PRINT_INFO("resume (det_irq=%d    but_irq=%d)\n", headset.irq_detect, headset.irq_button);
        active_status = 1;

        if(1 == headset.platform_data->irq_trigger_level_detect)
                irq_set_irq_type(headset.irq_detect, IRQF_TRIGGER_HIGH);
        else
                irq_set_irq_type(headset.irq_detect, IRQF_TRIGGER_LOW);
        headset_irq_detect_enable(1, headset.irq_detect);
        return 0;
}
#else
#define headset_suspend NULL
#define headset_resume NULL
#endif

static struct platform_driver headset_detect_driver = {
        .driver = {
                .name = "headset-detect",
                .owner = THIS_MODULE,
        },
        .probe = headset_detect_probe,
        .suspend = headset_suspend,
        .resume = headset_resume,
};

static int __init headset_init(void)
{
        int ret;
        ret = platform_driver_register(&headset_detect_driver);
        return ret;
}

static void __exit headset_exit(void)
{
        platform_driver_unregister(&headset_detect_driver);
}

module_init(headset_init);
module_exit(headset_exit);

MODULE_DESCRIPTION("headset & button detect driver");
MODULE_AUTHOR("Yaochuan Li <yaochuan.li@spreadtrum.com>");
MODULE_LICENSE("GPL");
