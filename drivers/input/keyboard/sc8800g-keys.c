/*
 * File:         drivers/input/keyboard/sc8800g-keys.c
 * Based on:
 * Author:       Richard Feng <Richard Feng@spreadtrum.com>
 *
 * Created:
 * Description:  keypad driver for sc8800s Processors
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see the file COPYING, or write
 * to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/sysrq.h>
#include <mach/mfp.h>
#include <mach/sprd_key.h>
#include "regs_kpd_sc8800g.h"
#include <mach/regs_cpc.h>

#include <mach/regs_ana.h>
#include <mach/regs_gpio.h>
#include <mach/adi_hal_internal.h>

#ifdef CONFIG_ARCH_SC8810
#include <mach/eic.h>
#endif

#define DRV_NAME        	"sprd-keypad"

#define INT_MASK_STS						(SPRD_INTCV_BASE + 0x0000)
#define INT_RAW_STS						(SPRD_INTCV_BASE + 0x0004)
#define INT_EN							(SPRD_INTCV_BASE + 0x0008)
#define INT_DIS							(SPRD_INTCV_BASE + 0x000C)

#define REG_INT_MASK_STS					(*((volatile unsigned int *)INT_MASK_STS))
#define REG_INT_RAW_STS					(*((volatile unsigned int *)INT_RAW_STS))
#define REG_INT_EN						(*((volatile unsigned int *)INT_EN))
#define REG_INT_DIS						(*((volatile unsigned int *)INT_DIS))



#define REG_PIN_CTL_REG					(*((volatile unsigned int *)PIN_CTL_REG))

#define REG_PIN_KEYOUT0_REG				(*((volatile unsigned int *)PIN_KEYOUT0_REG))
#define REG_PIN_KEYOUT1_REG				(*((volatile unsigned int *)PIN_KEYOUT1_REG))
#define REG_PIN_KEYOUT2_REG				(*((volatile unsigned int *)PIN_KEYOUT2_REG))
#define REG_PIN_KEYOUT3_REG				(*((volatile unsigned int *)PIN_KEYOUT3_REG))
#define REG_PIN_KEYOUT4_REG				(*((volatile unsigned int *)PIN_KEYOUT4_REG))
#define REG_PIN_KEYOUT5_REG				(*((volatile unsigned int *)PIN_KEYOUT5_REG))
#define REG_PIN_KEYOUT6_REG				(*((volatile unsigned int *)PIN_KEYOUT6_REG))
#define REG_PIN_KEYOUT7_REG				(*((volatile unsigned int *)PIN_KEYOUT7_REG))

#define REG_PIN_KEYIN0_REG				(*((volatile unsigned int *)PIN_KEYIN0_REG))
#define REG_PIN_KEYIN1_REG				(*((volatile unsigned int *)PIN_KEYIN1_REG))
#define REG_PIN_KEYIN2_REG				(*((volatile unsigned int *)PIN_KEYIN2_REG))
#define REG_PIN_KEYIN3_REG				(*((volatile unsigned int *)PIN_KEYIN3_REG))
#define REG_PIN_KEYIN4_REG				(*((volatile unsigned int *)PIN_KEYIN4_REG))
#define REG_PIN_KEYIN5_REG				(*((volatile unsigned int *)PIN_KEYIN5_REG))
#define REG_PIN_KEYIN6_REG				(*((volatile unsigned int *)PIN_KEYIN6_REG))
#define REG_PIN_KEYIN7_REG				(*((volatile unsigned int *)PIN_KEYIN7_REG))

#define REG_GR_GEN0						(*((volatile unsigned int *)GR_GEN0))

#define KPD_ROW_MIN_NUM				4  /* when config keypad type, the value of */
#define KPD_COL_MIN_NUM				3  /* when config keypad type, the value of */
#define KPD_ROW_MAX_NUM				8  /* when config keypad type, the value of */
#define KPD_COL_MAX_NUM				8  /* when config keypad type, the value of */

#define KPDCTL_ROW						(0xf << 16)  /* enable bit for rows(row4 --- row7) */
#define KPDCTL_COL						(0x1f << 20)  /* enable bit for cols(col3 --- col4) */

#define MAX_MUL_KEY_NUM				3
#define MAX_GPIO_KEY_NUM				3

#define MAX_MUL_MUX_NUM				MAX_MUL_KEY_NUM + MAX_GPIO_KEY_NUM

/* keypad constant */
#define TB_KPD_CONST_BASE				(0x80)
#define TB_KPD_RELEASED					(TB_KPD_CONST_BASE)
#define TB_KPD_PRESSED					(TB_KPD_CONST_BASE + 1)
#define TB_KPD_INVALID_KEY				(0x0FFFF)
#define TB_KPD_GPIO_KEY					(0x0FF00)

#define POWRER_KEY_VAL					ANDROID_KEY_POWER
#define CHECK_TIMER_EXPIRE				(5)//ms
#define CHECK_POWER_TIMER_EXPIRE		(15)//ms
#define AVOID_QUIVER_MIN_COUNT			(1)

/* check if this key is the same as the previous one */
#define IS_KEY_VALID(_key)					((_key == key_ptr->key_code) ? 1 : 0)
//The corresponding bit of KPD_STS register.
#define KPD_INT_ALL						(0xfff)

#define KPD_PRESS_INT0					BIT_0
#define KPD_PRESS_INT1					BIT_1
#define KPD_PRESS_INT2					BIT_2
#define KPD_PRESS_INT3					BIT_3

#define KPD_RELEASE_INT0					BIT_4
#define KPD_RELEASE_INT1					BIT_5
#define KPD_RELEASE_INT2					BIT_6
#define KPD_RELEASE_INT3					BIT_7

#define KPD_LONG_KEY_INT0				BIT_8
#define KPD_LONG_KEY_INT1				BIT_9
#define KPD_LONG_KEY_INT2				BIT_10
#define KPD_LONG_KEY_INT3				BIT_11

#define KPD_COL_CNT						0x7
#define KPD_ROW_CNT						0x70
#define KPD1_COL_CNT					0x7
#define KPD1_ROW_CNT					0x70
#define KPD2_COL_CNT					0x700
#define KPD2_ROW_CNT					0x7000
#define KPD3_COL_CNT					0x70000
#define KPD3_ROW_CNT					0x700000
#define KPD4_COL_CNT					0x7000000
#define KPD4_ROW_CNT					0x70000000

#define KPDPOLARITY_ROW					(0x00FF)
#define KPDPOLARITY_COL					(0xFF00)

#define KPDCLK0_CLK_DIV0					0xFFFF      //Clock dividor [15:0]
#define KPDCLK1_TIME_CNT				0xFFB0      //Time out counter value

#define CFG_ROW_POLARITY				(0x00FF & KPDPOLARITY_ROW)
#define CFG_COL_POLARITY					(0xFF00 & KPDPOLARITY_COL)
#define CFG_CLK_DIV						1


#define ANA_GPIO_IRQ						BIT_1
#define ANA_INT_EN						(SPRD_MISC_BASE + 0x380 + 0x08)

struct sprd_kpad_t {
        struct input_dev *input;
        int irq;
        unsigned short *keycode;
        unsigned int keyup_test_jiffies;
};

typedef struct kpd_key_t
{
    unsigned short key_code;
    unsigned char state;
    unsigned char count;
    unsigned long time_stamp;
    unsigned long timer_id;
} ;

static struct timer_list s_kpd_timer[MAX_MUL_MUX_NUM];
static struct kpd_key_t s_key [MAX_MUL_MUX_NUM];

struct sprd_kpad_t *sprd_kpad;

//yunlong.wang add for key emulator
static unsigned char g_keycode;
static unsigned long pb_keystatus4sleep = 0;	/* 0 : UP;  1 : DOWN */
static ssize_t sprd_kpad_store_emulate(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t sprd_kpad_show_emulate(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t sprd_kpad_store_sysrq(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t sprd_kpad_show_sysrq(struct device* cd,struct device_attribute *attr, char* buf);
static DEVICE_ATTR(emulate, S_IRUGO | S_IWUSR, sprd_kpad_show_emulate, sprd_kpad_store_emulate);
static DEVICE_ATTR(sysrq, S_IRUGO | S_IWUSR, sprd_kpad_show_sysrq, sprd_kpad_store_sysrq);
static sysrq_enabled = 1;
 
static ssize_t sprd_kpad_show_emulate(struct device* cd,struct device_attribute *attr, char* buf)
{
	ssize_t ret = 0;

	sprintf(buf, "%d\n",g_keycode);
	ret = strlen(buf) + 1;
	
	return ret;
}

static ssize_t sprd_kpad_store_emulate(struct device* cd, struct device_attribute *attr,const char* buf, size_t len)
{
	unsigned long keycode = simple_strtoul(buf, NULL, 10);
	g_keycode = keycode;
	
	input_report_key(sprd_kpad->input, g_keycode, 1);
	input_sync(sprd_kpad->input);
	printk("%dD\n", g_keycode);

	mdelay(10);

	input_report_key(sprd_kpad->input, g_keycode, 0);
	input_sync(sprd_kpad->input);	
	printk("%dU\n", g_keycode);
		
	return len;
}

static ssize_t sprd_kpad_show_sysrq(struct device* cd,struct device_attribute *attr, char* buf)
{
	return sprintf(buf, "%d\n", sysrq_enabled);
}

static ssize_t sprd_kpad_store_sysrq(struct device* cd, struct device_attribute *attr,const char* buf, size_t len)
{
	sysrq_enabled = !!simple_strtoul(buf, NULL, NULL);
	return len;
}

static int sprd_kpad_create_sysfs(struct platform_device *pdev)
{
	int err;
	struct device *dev = &(pdev->dev);
	
	err = device_create_file(dev, &dev_attr_emulate);
    if (err == 0)
        err = device_create_file(dev, &dev_attr_sysrq);
    else pr_err("%s: create sysrq\n", __FUNCTION__);

	return err;
}

//yunlong.wang add END


void clear_key(struct kpd_key_t *key_ptr)
{
	key_ptr->key_code   = TB_KPD_INVALID_KEY;
	key_ptr->state      = TB_KPD_RELEASED;
    	key_ptr->count      = 0;
    	key_ptr->time_stamp = 0;
}

static inline int sprd_kpad_find_key(struct sprd_kpad_t *sprd_kpad,
                        struct input_dev *input, u16 keyident)
{
        u16 i;

        for (i = 0; i < input->keycodemax; i++)
                if (sprd_kpad->keycode[i + input->keycodemax] == keyident)
                        return sprd_kpad->keycode[i];

        return -1;
}

static inline void sprd_keycodecpy(unsigned short *keycode,
				const unsigned int *pdata_kc,
				unsigned short keymapsize,
				const struct sprd_gpio_keys *gpio_pdata_kc,
				unsigned short gpio_keymapsize)
{
        unsigned int i;
        unsigned int max = keymapsize + gpio_keymapsize;

        for (i = 0; i < keymapsize; i++) {
                keycode[i] = pdata_kc[i] & 0xffff;
                keycode[i + max] = pdata_kc[i] >> 16;
        }
        for (i = keymapsize; i < max; i++) {
                keycode[i] = gpio_pdata_kc[i-keymapsize].key_value & 0xffff;
                keycode[i + max] = TB_KPD_GPIO_KEY | (i-keymapsize);
        }
}

static void print_kpad(void)
{
	static int count = 0;

	count ++;
	printk("\n\ncount = %d\n\n", count);
	printk("\n\nREG_INT_MASK_STS = 0x%08x\n", REG_INT_MASK_STS);
	printk("REG_GR_SOFT_RST = 0x%08x\n", REG_GR_SOFT_RST);
	printk("REG_INT_RAW_STS = 0x%08x\n", REG_INT_RAW_STS);
	printk("REG_INT_EN = 0x%08x\n", REG_INT_EN);
	printk("REG_INT_DIS = 0x%08x\n", REG_INT_DIS);
	printk("REG_GR_GEN0 = 0x%08x\n", REG_GR_GEN0);
	printk("REG_KPD_CTRL = 0x%08x\n", REG_KPD_CTRL);
	printk("REG_KPD_INT_EN = 0x%08x\n", REG_KPD_INT_EN);
	printk("REG_KPD_INT_RAW_STATUS = 0x%08x\n", REG_KPD_INT_RAW_STATUS);
	printk("REG_KPD_INT_MASK_STATUS = 0x%08x\n", REG_KPD_INT_MASK_STATUS);
	printk("REG_KPD_INT_CLR = 0x%08x\n", REG_KPD_INT_CLR);
	printk("REG_KPD_POLARITY = 0x%08x\n", REG_KPD_POLARITY);
	printk("REG_KPD_DEBOUNCE_CNT = 0x%08x\n", REG_KPD_DEBOUNCE_CNT);
	printk("REG_KPD_LONG_KEY_CNT = 0x%08x\n", REG_KPD_LONG_KEY_CNT);
	printk("REG_KPD_SLEEP_CNT = 0x%08x\n", REG_KPD_SLEEP_CNT);
	printk("REG_KPD_CLK_DIV_CNT = 0x%08x\n", REG_KPD_CLK_DIV_CNT);
	printk("REG_KPD_KEY_STATUS = 0x%08x\n", REG_KPD_KEY_STATUS);
	printk("REG_KPD_SLEEP_STATUS = 0x%08x\n", REG_KPD_SLEEP_STATUS);
	printk("REG_PIN_CTL_REG = 0x%08x\n", REG_PIN_CTL_REG);
#if !defined(CONFIG_ARCH_SC8810) 
	printk("REG_KPD_DEBUG_STATUS1 = 0x%08x\n", REG_KPD_DEBUG_STATUS1);
	printk("REG_KPD_DEBUG_STATUS2 = 0x%08x\n", REG_KPD_DEBUG_STATUS2);

	printk("REG_PIN_KEYOUT0_REG = 0x%08x\n", REG_PIN_KEYOUT0_REG);
	printk("REG_PIN_KEYOUT1_REG = 0x%08x\n", REG_PIN_KEYOUT1_REG);
	printk("REG_PIN_KEYOUT2_REG = 0x%08x\n", REG_PIN_KEYOUT2_REG);
	printk("REG_PIN_KEYOUT3_REG = 0x%08x\n", REG_PIN_KEYOUT3_REG);
	printk("REG_PIN_KEYOUT4_REG = 0x%08x\n", REG_PIN_KEYOUT4_REG);
	printk("REG_PIN_KEYOUT5_REG = 0x%08x\n", REG_PIN_KEYOUT5_REG);
	printk("REG_PIN_KEYOUT6_REG = 0x%08x\n", REG_PIN_KEYOUT6_REG);
	printk("REG_PIN_KEYOUT7_REG = 0x%08x\n", REG_PIN_KEYOUT7_REG);

	printk("REG_PIN_KEYIN0_REG = 0x%08x\n", REG_PIN_KEYIN0_REG);
	printk("REG_PIN_KEYIN1_REG = 0x%08x\n", REG_PIN_KEYIN1_REG);
	printk("REG_PIN_KEYIN2_REG = 0x%08x\n", REG_PIN_KEYIN2_REG);
	printk("REG_PIN_KEYIN3_REG = 0x%08x\n", REG_PIN_KEYIN3_REG);
	printk("REG_PIN_KEYIN4_REG = 0x%08x\n", REG_PIN_KEYIN4_REG);
	printk("REG_PIN_KEYIN5_REG = 0x%08x\n", REG_PIN_KEYIN5_REG);
	printk("REG_PIN_KEYIN6_REG = 0x%08x\n", REG_PIN_KEYIN6_REG);
	printk("REG_PIN_KEYIN7_REG = 0x%08x\n", REG_PIN_KEYIN7_REG);
#endif	
}

void change_state(struct kpd_key_t *key_ptr)
{   
	unsigned short key;

    	/* Change state of the key according to it's current state */
    	if (TB_KPD_RELEASED == key_ptr->state) {
		/* Change state from TB_KPD_RELEASED to TB_KPD_PRESSED */
        	key_ptr->state  = TB_KPD_PRESSED;
        	key_ptr->count  = 1;
		key = sprd_kpad_find_key(sprd_kpad, sprd_kpad->input, key_ptr->key_code);
        if ( key != POWRER_KEY_VAL){
		    mod_timer(&s_kpd_timer[key_ptr->timer_id], jiffies + CHECK_TIMER_EXPIRE);
        }
        else{
		    mod_timer(&s_kpd_timer[key_ptr->timer_id], jiffies + CHECK_POWER_TIMER_EXPIRE);
        }
        	input_report_key(sprd_kpad->input, key, 1);
        	input_sync(sprd_kpad->input);
		printk("%dD\n", key);
	} else {
        	/* Change state from TB_KPD_PRESSED to TB_KPD_RELEASED */
		key = sprd_kpad_find_key(sprd_kpad, sprd_kpad->input, key_ptr->key_code);		
        	input_report_key(sprd_kpad->input, key, 0);
        	input_sync(sprd_kpad->input);	
        	clear_key(key_ptr);
		if (key == POWRER_KEY_VAL)
			pb_keystatus4sleep = 0;	/* UP */
		printk("%dU\n", key);
    	}
}

unsigned long handle_key(unsigned short key_code, struct kpd_key_t *key_ptr)
{
    unsigned long status = 0;
    
    switch(key_ptr->state) {
	case TB_KPD_RELEASED:
        	/* check if it is the first INT of a key */
        	if (key_ptr->count == 0) {
			key_ptr->key_code = key_code;
			key_ptr->time_stamp = jiffies;
            		key_ptr->count ++;
                    	key_ptr->count ++;
                        change_state(key_ptr);
        	} else { //if (key_ptr->count == 0)
            		/* Check if this key is the same as the previous key, if same, handle it, else ignore the previous key */
            		if (IS_KEY_VALID(key_code)) {
                		/* Can't be TB_KPD_INVALID_KEY */
                		if (key_code == TB_KPD_INVALID_KEY) {
					status = TB_KPD_INVALID_KEY;
					return status;
				}          

                    		/* Add count of INT */
                    		key_ptr->count ++;
                    		/* if it is pressed long enough, then changes it's state */
                    		if (key_ptr->count >= AVOID_QUIVER_MIN_COUNT)
                        		change_state(key_ptr);
            		} else if ((key_ptr->key_code == TB_KPD_INVALID_KEY)) { //if (IS_KEY_VALID(key_code))
                		/* Not same, Get key code */
                		key_ptr->key_code = key_code;
                		/* Set time stamp */
                		key_ptr->time_stamp = jiffies;
                		/* Add count of INT */
                		key_ptr->count = 1;             
            		} else {
				status = TB_KPD_INVALID_KEY;
	    		}
        	}// if (key_ptr->count == 0)
        
        break;
        
    	case TB_KPD_PRESSED:
        	/* Check if this key is the same as the old key, if same, handle it, else then only skip this key */
        	if (IS_KEY_VALID(key_code)) {
            		/* Add count of INT */
            		key_ptr->count ++;
        	} else {
           		/* Not same */
            		status = TB_KPD_INVALID_KEY;
        	}
        break;
    
	default: 
        break;

    } //switch(key_ptr->state)

    return status;
}

static void sprd_kpad_timer(unsigned long data)
{
	unsigned short key;
	unsigned long key_sts = 0xffff;
	struct kpd_key_t *key_ptr = (struct kpd_key_t *)data;
	
#if  defined (CONFIG_ARCH_SC8810)
	key_sts = sprd_get_eic_data(EIC_ID_11); /* fix on bug when charging and pb pressed long */
#endif
	/* Check if the key is released, if the state is TB_KPD_PRESSED and count is 0, it means the key is released */
    	if (key_ptr->state == TB_KPD_PRESSED) {
        	if ((key_ptr->count == 0) && (key_sts != 0)) {
            		change_state(key_ptr);
        	} else {
           		key_ptr->count = 0;
                	key = sprd_kpad_find_key(sprd_kpad, sprd_kpad->input, key_ptr->key_code);
	                if ( key != POWRER_KEY_VAL ){
			            mod_timer(&s_kpd_timer[key_ptr->timer_id], jiffies + CHECK_TIMER_EXPIRE);
	                }
	                else{
			            mod_timer(&s_kpd_timer[key_ptr->timer_id], jiffies + CHECK_POWER_TIMER_EXPIRE);
	                }
	    	}
    	}			
}

static irqreturn_t sprd_kpad_isr(int irq, void *dev_id)
{
        int i;
	unsigned short  key_code = 0;
	struct kpd_key_t *key_ptr = NULL;
	unsigned long found = 0, status; 
	unsigned long s_int_status = REG_KPD_INT_RAW_STATUS;
	unsigned long s_key_status = REG_KPD_KEY_STATUS;
	//unsigned long debug1 = REG_KPD_DEBUG_STATUS1;
	//unsigned long debug2 = REG_KPD_DEBUG_STATUS2;
	
	REG_KPD_INT_CLR |= KPD_INT_ALL;
	//printk("int_status = 0x%08x  key_status = 0x%08x  debug1 = 0x%08x  debug2 = 0x%08x\n", s_int_status, s_key_status, debug1, debug2);
	/* check the type of INT */    
    	if ((s_int_status & KPD_PRESS_INT0) || (s_int_status & KPD_LONG_KEY_INT0)) {
        	key_code = (s_key_status  & (KPD1_ROW_CNT | KPD1_COL_CNT));
		/* if key_code is stored, don't seat the code again */
	    	for (i = 0; i< MAX_MUL_KEY_NUM; i++) {
    	        	key_ptr = &s_key[i];
    			if (key_ptr->key_code == key_code) {
	    			handle_key(key_code, key_ptr);	    	
	    			found = 1;
	    			break;
    			}
    		}
	   	if (!found) {			
			/* the key_code is fresh, try to find a seat other than exceed the seat limit */
			for (i = 0; i < MAX_MUL_KEY_NUM; i++) {
				key_ptr = &s_key[i];
				status = handle_key(key_code, key_ptr);
				if (status == 0)
					break;
			}
		}
	} else if ((s_int_status & KPD_PRESS_INT1) || (s_int_status & KPD_LONG_KEY_INT1)) {
        	key_code = (s_key_status  & (KPD2_ROW_CNT | KPD2_COL_CNT)) >> 8;
		/* if key_code is stored, don't seat the code again */
	    	for (i = 0; i< MAX_MUL_KEY_NUM; i++) {
    	        	key_ptr = &s_key[i];
    			if (key_ptr->key_code == key_code) {
	    			handle_key(key_code, key_ptr);	    	
	    			found = 1;
	    			break;
    			}
    		}
	   	if (!found) {
			/* the key_code is fresh, try to find a seat other than exceed the seat limit */	        	   	
			for (i = 0; i < MAX_MUL_KEY_NUM; i++) {
				key_ptr = &s_key[i];
				status = handle_key(key_code, key_ptr);
				if (status == 0)
					break;
			}
		}
	} else if ((s_int_status & KPD_PRESS_INT2) || (s_int_status & KPD_LONG_KEY_INT2)) {
	       	key_code = (s_key_status  & (KPD3_ROW_CNT | KPD3_COL_CNT)) >> 16;
		/* if key_code is stored, don't seat the code again */
	    	for (i = 0; i< MAX_MUL_KEY_NUM; i++) {
    	        	key_ptr = &s_key[i];
    			if (key_ptr->key_code == key_code) {
	    			handle_key(key_code, key_ptr);	    	
	    			found = 1;
	    			break;
    			}
    		}
	   	if (!found) {
			/* the key_code is fresh, try to find a seat other than exceed the seat limit */	        	   	
			for (i = 0; i < MAX_MUL_KEY_NUM; i++) {
				key_ptr = &s_key[i];
				status = handle_key(key_code, key_ptr);
				if (status == 0)
					break;
			}
		}
	} else if (s_int_status & KPD_RELEASE_INT0) {
		if (TB_KPD_RELEASED == s_key[0].state)
			clear_key(&s_key[0]);
	} else if (s_int_status & KPD_RELEASE_INT1) {
		if (TB_KPD_RELEASED == s_key[1].state)
			clear_key(&s_key[1]);
	} else if (s_int_status & KPD_RELEASE_INT2) {
		if (TB_KPD_RELEASED == s_key[2].state)
			clear_key(&s_key[2]);
	}
	
#ifdef CONFIG_MAGIC_SYSRQ	       /* Handle the SysRq Hack */
	/* Vol-Down + Camera */
	if (sysrq_enabled && s_key_status == 0x77778180) {
		printk("########################################################################\n");
		handle_sysrq('m', NULL);
		printk("########################################################################\n");
		handle_sysrq('p', NULL);
		printk("########################################################################\n");
		handle_sysrq('q', NULL);
		printk("########################################################################\n");
		handle_sysrq('w', NULL);
		printk("########################################################################\n");
		handle_sysrq('s', NULL);
		printk("########################################################################\n");
		handle_sysrq('b', NULL);
		printk("########################################################################\n");
	}
#endif
        return IRQ_HANDLED;
}


static int irq_is_detected  = 0;

static irqreturn_t sprd_gpio_isr(int irq, void *dev_id)
{
	int ret, gpio;
	unsigned short  key_code;
	struct kpd_key_t *key_ptr;
	int gpio_id = (int)dev_id;

	//msleep(20);
	gpio = irq_to_gpio(irq);
	ret = gpio_get_value(gpio);
	if (!ret) {
		key_code = gpio_id | TB_KPD_GPIO_KEY;
		/* gpio key never seat the code again */
		key_ptr = &s_key[MAX_MUL_KEY_NUM+gpio_id];
		handle_key(key_code, key_ptr);			
	} else {
		if (TB_KPD_RELEASED == s_key[MAX_MUL_KEY_NUM+gpio_id].state) {
			clear_key(&s_key[MAX_MUL_KEY_NUM+gpio_id]);
		}
	}
        return IRQ_HANDLED;
}

static irqreturn_t sprd_pint_isr(int irq, void *dev_id)
{	
	irqreturn_t ret = 0;
	unsigned short  key_code;
	struct kpd_key_t *key_ptr;	
	int gpio_id = (int)dev_id;
	ret = sprd_get_eic_data(EIC_ID_11);
	
	if ((pb_keystatus4sleep == 0) && (ret != 0)) {
		/* fix on bug : kernel output a lot of log when pb presse shortly */
		pb_keystatus4sleep = 1;	/* DOWN */
		ret = 0;
	}
	if (!ret) {
		key_code = gpio_id | TB_KPD_GPIO_KEY;
		/* gpio key never seat the code again */
		key_ptr = &s_key[MAX_MUL_KEY_NUM+gpio_id];
		handle_key(key_code, key_ptr);			
	} else {
		if (TB_KPD_RELEASED == s_key[MAX_MUL_KEY_NUM+gpio_id].state) {
			clear_key(&s_key[MAX_MUL_KEY_NUM+gpio_id]);
		}
	}
	return ret;
}
static void gpio_key_init(unsigned long gpio, const int gpio_id)
{
	unsigned long err, irq, ret;

	err = gpio_request(gpio, "keypad");
	if (err)
		printk("can not alloc gpio for %d Key\n", gpio);
	else
		printk("alloc gpio for %d Key\n", gpio);

	gpio_direction_input(gpio);
	irq = sprd_alloc_gpio_irq(gpio);
	ret = request_threaded_irq(irq, NULL, sprd_gpio_isr, IRQF_TRIGGER_LOW | IRQF_ONESHOT, NULL, (void*)gpio_id);
	ret = gpio_get_value(gpio);
	if (ret) {
		printk("The Pin is HIGH, so set low level trigger. irq = %ld\n", irq);
		set_irq_type(irq, IRQF_TRIGGER_LOW);
	} else {
		printk("The Pin is LOW, HOME pin configuration is wrong\n");
	}
}

#if  defined (CONFIG_ARCH_SC8810)
static void int_key_init(enum EIC_TYPE_E eic_id, const int gpio_id)
{
	unsigned long err, irq, ret;
	int data = 0;

	ret = sprd_alloc_eic_irq(eic_id);
	if (ret != -1) {
		irq = ret ;
		//ret = request_threaded_irq(irq, sprd_pint_isr, NULL, IRQF_TRIGGER_LOW/* | IRQF_ONESHOT*/, NULL, (void*)gpio_id);
		ret = request_irq(irq, sprd_pint_isr, IRQF_TRIGGER_LOW, "powerkey", (void*)gpio_id);
		ret = sprd_get_eic_data(eic_id);
		if (ret) {
			printk("The Pin is HIGH, so set low level trigger. irq = %ld\n", irq);
			set_irq_type(irq, IRQF_TRIGGER_LOW);
		} else {
			printk("The Pin is LOW, HOME pin configuration is wrong\n");
		}
	}

}
#endif

static int __devinit sprd_kpad_probe(struct platform_device *pdev)
{
        struct sprd_kpad_platform_data *pdata;
        struct input_dev *input;
        int i, error, key_type;
	int key_mux_size;

	
	pdata = pdev->dev.platform_data;
	key_mux_size = pdata->keymapsize + pdata->gpio_keymapsize;
        if (!pdata->rows || !pdata->cols || !pdata->keymap) {
                dev_err(&pdev->dev, "no rows, cols or keymap from pdata\n");
               return -EINVAL;
        }
	
        if (!pdata->keymapsize ||
            pdata->keymapsize > (pdata->rows * pdata->cols)) {
                dev_err(&pdev->dev, "invalid keymapsize\n");
                return -EINVAL;
        }
	
        sprd_kpad = kzalloc(sizeof(struct sprd_kpad_t), GFP_KERNEL);
        if (!sprd_kpad)
                return -ENOMEM;
	
        platform_set_drvdata(pdev, sprd_kpad);

        /* Allocate memory for keymap followed by private LUT */
        sprd_kpad->keycode = kmalloc(key_mux_size * sizeof(unsigned short) * 2, GFP_KERNEL);
        if (!sprd_kpad->keycode) {
                error = -ENOMEM;
                goto out;
        }

        if (!pdata->keyup_test_interval)
                sprd_kpad->keyup_test_jiffies = msecs_to_jiffies(50);
        else
                sprd_kpad->keyup_test_jiffies =
                       msecs_to_jiffies(pdata->keyup_test_interval);
	
        sprd_kpad->irq = platform_get_irq(pdev, 0);
        if (sprd_kpad->irq < 0) {
                error = -ENODEV;
                goto out2;
        }
	
        /* init sprd keypad controller */
	REG_GR_SOFT_RST |= 0x2;
	mdelay(10);
	REG_GR_SOFT_RST &= ~0x2;


	//add by overlord .we should check when set cols & rows because custom maybe do sth wrong
	if(unlikely( pdata->cols < KPD_COL_MIN_NUM))
	{
		pdata->cols = KPD_COL_MIN_NUM;
	}
	else if(unlikely(pdata->cols > KPD_COL_MAX_NUM))
	{
		pdata->cols = KPD_COL_MAX_NUM;
	}
	if(unlikely(pdata->rows < KPD_ROW_MIN_NUM))
	{
		pdata->rows = KPD_ROW_MIN_NUM;
	}
	else if(unlikely(pdata->rows > KPD_ROW_MAX_NUM))
	{
		pdata->rows = KPD_ROW_MAX_NUM;
	}
	//add by overlord .we should check when set cols & rows because custom maybe do sth wrong
	key_type = ((((~(0xffffffff << (pdata->cols - KPD_COL_MIN_NUM))) << 20) | ((~(0xffffffff << (pdata->rows - KPD_ROW_MIN_NUM))) << 16)) & (KPDCTL_ROW | KPDCTL_COL));

	REG_KPD_CTRL = 0x6 | key_type;
        REG_INT_DIS = (1 << IRQ_KPD_INT);
        REG_GR_GEN0 |= BIT_8 | BIT_26;
        REG_KPD_INT_CLR = KPD_INT_ALL;
        REG_KPD_POLARITY = CFG_ROW_POLARITY | CFG_COL_POLARITY;
        REG_KPD_CLK_DIV_CNT = CFG_CLK_DIV & KPDCLK0_CLK_DIV0;
	REG_KPD_LONG_KEY_CNT = 0xc;
	REG_KPD_DEBOUNCE_CNT = 0x5;


        error = request_irq(sprd_kpad->irq, sprd_kpad_isr, 0, DRV_NAME, pdev);
        if (error) {
                dev_err(&pdev->dev, "unable to claim irq %d\n", sprd_kpad->irq);
                goto out2;
        }

        input = input_allocate_device();
        if (!input) {
                error = -ENOMEM;
                goto out3;
        }

        sprd_kpad->input = input;
        input->name = pdev->name;
        input->phys = "sprd-key/input0";
        input->dev.parent = &pdev->dev;
	input_set_drvdata(input, sprd_kpad);

        input->id.bustype = BUS_HOST;
        input->id.vendor = 0x0001;
        input->id.product = 0x0001;
        input->id.version = 0x0100;

        input->keycodesize = sizeof(unsigned short);
        input->keycodemax = key_mux_size;
        input->keycode = sprd_kpad->keycode;

        sprd_keycodecpy(sprd_kpad->keycode, pdata->keymap, pdata->keymapsize,pdata->gpio_keymap, pdata->gpio_keymapsize);
	/*printk("keycodesize = %d  keycodemax = %d\n", input->keycodesize, input->keycodemax);
	for (i = 0; i < input->keycodemax; i++) {
		printk("keycode = 0x%04x  rowcol = 0x%04x\n", sprd_kpad->keycode[i], sprd_kpad->keycode[i + input->keycodemax]);	
	}*/

        /* setup input device */
        __set_bit(EV_KEY, input->evbit);

        if (pdata->repeat)
                __set_bit(EV_REP, input->evbit);

        for (i = 0; i < input->keycodemax; i++)
		__set_bit(sprd_kpad->keycode[i], input->keybit);

        __clear_bit(KEY_RESERVED, input->keybit);

        error = input_register_device(input);
        if (error) {
                dev_err(&pdev->dev, "unable to register input device\n");
                goto out4;
        }

        device_init_wakeup(&pdev->dev, 1);
	for (i = 0; i < MAX_MUL_MUX_NUM; i++) {
		/* clear Key state */
		clear_key(&s_key[i]);
		/* create a timer to check if key is released */
		setup_timer(&s_kpd_timer[i], sprd_kpad_timer, (unsigned long) &s_key[i]);
		s_key[i].timer_id = i;						        
	}

	REG_KPD_INT_EN = KPD_INT_ALL;
	REG_INT_EN |= 1 << IRQ_KPD_INT;
	REG_KPD_CTRL |= 0x1;

	//print_kpad();

	//init gpio key
	if(pdata->gpio_keymap) {
		for (i = 0; i < pdata->gpio_keymapsize; i++)  {
			#if  defined (CONFIG_ARCH_SC8810)
			if((*(pdata->gpio_keymap[i].gpio)) == 163)//EIC163 is PBINT
			{
				int_key_init(EIC_ID_11, 0);
			}else
			#endif
			{
				gpio_key_init(*(pdata->gpio_keymap[i].gpio),i);
			}
		}
		ANA_REG_OR(ANA_INT_EN, ANA_GPIO_IRQ);		
	}


	sprd_kpad_create_sysfs(pdev);

	return 0;

out4:
        input_free_device(input);
out3:
        free_irq(sprd_kpad->irq, pdev);
out2:
        kfree(sprd_kpad->keycode);
out:
        kfree(sprd_kpad);
        platform_set_drvdata(pdev, NULL);
        return error;
}

static int __devexit sprd_kpad_remove(struct platform_device *pdev)
{
	int i;
        //struct sprd_kpad_platform_data *pdata = pdev->dev.platform_data;
        struct sprd_kpad_t *sprd_kpad = platform_get_drvdata(pdev);

	for (i = 0; i < MAX_MUL_MUX_NUM; i++)
        	del_timer_sync(&s_kpd_timer[i]);

        free_irq(sprd_kpad->irq, pdev);
        input_unregister_device(sprd_kpad->input);

        kfree(sprd_kpad->keycode);
        kfree(sprd_kpad);
        platform_set_drvdata(pdev, NULL);

        /* disable sprd keypad controller */
        REG_INT_DIS = 1 << IRQ_KPD_INT;
        REG_KPD_INT_CLR = KPD_INT_ALL;
        REG_KPD_CTRL &= ~(1 << 0);
        REG_GR_GEN0 &= ~(BIT_8 | BIT_26);

        return 0;
}

#ifdef CONFIG_PM
static int sprd_kpad_suspend(struct platform_device *pdev, pm_message_t state)
{
        struct sprd_kpad_t *sprd_kpad = platform_get_drvdata(pdev);

        return 0;
}

static int sprd_kpad_resume(struct platform_device *pdev)
{
        struct sprd_kpad_t *sprd_kpad = platform_get_drvdata(pdev);

        return 0;
}
#else
# define sprd_kpad_suspend NULL
# define sprd_kpad_resume  NULL
#endif

struct platform_driver sprd_kpad_device_driver = {
        .driver         = {
                .name   = DRV_NAME,
                .owner  = THIS_MODULE,
        },
        .probe          = sprd_kpad_probe,
        .remove         = __devexit_p(sprd_kpad_remove),
        .suspend        = sprd_kpad_suspend,
        .resume         = sprd_kpad_resume,
};

static int __init sprd_kpad_init(void)
{
        return platform_driver_register(&sprd_kpad_device_driver);
}

static void __exit sprd_kpad_exit(void)
{
        platform_driver_unregister(&sprd_kpad_device_driver);
}

module_init(sprd_kpad_init);
module_exit(sprd_kpad_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Richard Feng <Richard.Feng@spreadtrum.com>");
MODULE_DESCRIPTION("Keypad driver for spreadtrum Processors");
MODULE_ALIAS("platform:sprd-keypad");
