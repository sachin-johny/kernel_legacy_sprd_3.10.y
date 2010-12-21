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
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <mach/mfp.h>
#include "sprd_key.h"
#include "regs_kpd_sc8800g.h"
#include <mach/regs_cpc.h>

#define DRV_NAME        	"sprd-keypad"

#define INT_MASK_STS            (SPRD_INTCV_BASE + 0x0000)
#define INT_RAW_STS            	(SPRD_INTCV_BASE + 0x0004)
#define INT_EN                  (SPRD_INTCV_BASE + 0x0008)
#define INT_DIS			(SPRD_INTCV_BASE + 0x000C)

#define REG_INT_MASK_STS        (*((volatile unsigned int *)INT_MASK_STS))
#define REG_INT_RAW_STS        	(*((volatile unsigned int *)INT_RAW_STS))
#define REG_INT_EN              (*((volatile unsigned int *)INT_EN))
#define REG_INT_DIS          	(*((volatile unsigned int *)INT_DIS))


#define PIN_KEYIN5_REG					(SPRD_CPC_BASE + 0x00F4)
#define PIN_KEYIN6_REG					(SPRD_CPC_BASE + 0x00F8)
#define REG_PIN_CTL_REG          	(*((volatile unsigned int *)PIN_CTL_REG))
#define REG_PIN_KEYIN5_REG          	(*((volatile unsigned int *)PIN_KEYIN5_REG))
#define REG_PIN_KEYIN6_REG          	(*((volatile unsigned int *)PIN_KEYIN6_REG))



#define GR_GEN0                 (SPRD_GREG_BASE + 0x0008)
#define REG_GR_GEN0             (*((volatile unsigned int *)GR_GEN0))

#define KPD_ROW_MIN_NUM         4  /* when config keypad type, the value of */
#define KPD_COL_MIN_NUM         3  /* when config keypad type, the value of */

#define KPDCTL_ROW              (0xf << 16)  /* enable bit for rows(row4 --- row7) */
#define KPDCTL_COL              (0x1f << 20)  /* enable bit for cols(col3 --- col4) */

#define MAX_MUL_KEY_NUM		3
/* keypad constant */
#define TB_KPD_CONST_BASE       (0x80)
#define TB_KPD_RELEASED         (TB_KPD_CONST_BASE)
#define TB_KPD_PRESSED          (TB_KPD_CONST_BASE + 1)
#define TB_KPD_INVALID_KEY      (0x0FFFF)

#define CHECK_TIMER_EXPIRE      (15)//ms
#define AVOID_QUIVER_MIN_COUNT  (2)

/* check if this key is the same as the previous one */
#define IS_KEY_VALID(_key)      ((_key == key_ptr->key_code) ? 1 : 0)
#define SCAN2KEYVAl(codeval) ((((codeval) & 0x70) << 4) | ((codeval) & 0x7))
//The corresponding bit of KPD_STS register.
#define KPD_INT_ALL                     (0xfff)

#define KPD_PRESS_INT0                  BIT_0
#define KPD_PRESS_INT1                  BIT_1
#define KPD_PRESS_INT2                  BIT_2
#define KPD_PRESS_INT3                  BIT_3

#define KPD_RELEASE_INT0                BIT_4
#define KPD_RELEASE_INT1                BIT_5
#define KPD_RELEASE_INT2                BIT_6
#define KPD_RELEASE_INT3                BIT_7

#define KPD_LONG_KEY_INT0               BIT_8
#define KPD_LONG_KEY_INT1               BIT_9
#define KPD_LONG_KEY_INT2               BIT_10
#define KPD_LONG_KEY_INT3               BIT_11

#define KPD_COL_CNT                     0x7
#define KPD_ROW_CNT                     0x70
#define KPD1_COL_CNT                    0x7
#define KPD1_ROW_CNT                    0x70
#define KPD2_COL_CNT                    0x700
#define KPD2_ROW_CNT                    0x7000
#define KPD3_COL_CNT                    0x70000
#define KPD3_ROW_CNT                    0x700000
#define KPD4_COL_CNT                    0x7000000
#define KPD4_ROW_CNT                    0x70000000

//The corresponding bit of KPD_POLARITY register.
#define KPDPOLARITY_ROW                 (0x00FF)    // Internal row output xor with this 
// value to generate row output.
#define KPDPOLARITY_COL                 (0xFF00)    // Column input xor with this value to
#define KPDCLK0_CLK_DIV0                0xFFFF      //Clock dividor [15:0]
#define KPDCLK1_TIME_CNT                0xFFB0      //Time out counter value

#define CFG_ROW_POLARITY    (0x00FF & KPDPOLARITY_ROW)
#define CFG_COL_POLARITY    (0xFF00 & KPDPOLARITY_COL)
#define CFG_CLK_DIV         1

static const unsigned int sprd_keymap[] = {
#ifdef CONFIG_MACH_G2PHONE
        // 0 row
	KEYVAL(0, 0, KEY_BACK),
        KEYVAL(0, 1, KEY_RIGHT),
        KEYVAL(0, 2, KEY_HELP),
        KEYVAL(0, 3, KEY_SEND),
        KEYVAL(0, 4, KEY_DOWN),
        // 1 row
        KEYVAL(1, 0, KEY_UP),
	KEYVAL(1, 1, KEY_ENTER),
        KEYVAL(1, 2, KEY_3),
	KEYVAL(1, 3, KEY_1),
        KEYVAL(1, 4, KEY_2),
       // 2 row
        KEYVAL(2, 0, KEY_HELP), //KEY_HELP is not known
        KEYVAL(2, 1, KEY_LEFT),
        KEYVAL(2, 2, KEY_6),
        KEYVAL(2, 3, KEY_4),
        KEYVAL(2, 4, KEY_5),
       // 3 row
        KEYVAL(3, 0, KEY_VOLUMEUP),
        KEYVAL(3, 1, KEY_MENU),
        KEYVAL(3, 2, KEY_9),
        KEYVAL(3, 3, KEY_7),
        KEYVAL(3, 4, KEY_8),
	// 4 row
        KEYVAL(4, 0, KEY_VOLUMEDOWN),
        KEYVAL(4, 1, KEY_HELP), //KEY_POWER instead of KEY_HELP
        KEYVAL(4, 2, KEY_KPDOT), // is #
        KEYVAL(4, 3, KEY_KPASTERISK), //is *
        KEYVAL(4, 4, KEY_0),
#endif
#ifdef CONFIG_MACH_OPENPHONE
        // 0 row
	KEYVAL(0, 0, 00/*KEY_SEND*/), //dial up 1
        KEYVAL(0, 1, 01/*KEY_R*/), //R
        KEYVAL(0, 2, 02/*KEY_F*/), //F
        KEYVAL(0, 3, 03/*KEY_V*/), //V
        KEYVAL(0, 4, 04/*KEY_LEFT*/), //left
        KEYVAL(0, 5, 05/*KEY_CAMERA*/), //CAM
        KEYVAL(0, 6, 06/*KEY_Q*/), //Q
        KEYVAL(0, 7, 07/*KEY_ENTER*/), //left function
        // 1 row
        KEYVAL(1, 0, 10/*KEY_SEND*/), //dial up 2 -> no implement
	KEYVAL(1, 1, 11/*KEY_T*/), //T
        KEYVAL(1, 2, 12/*KEY_G*/), //G
	KEYVAL(1, 3, 13/*KEY_B*/), //B
        KEYVAL(1, 4, 14/*KEY_RIGHT*/), //right
        KEYVAL(1, 5, 15/*KEY_TV*/), //TV -> no implement
        KEYVAL(1, 6, 16/*KEY_W*/), //W
        KEYVAL(1, 7, 17/*KEY_BACK*/), //right function
       // 2 row
        KEYVAL(2, 0, 20/*KEY_END*/), //hang up
        KEYVAL(2, 1, 21/*KEY_Y*/), //Y
        KEYVAL(2, 2, 22/*KEY_H*/), //H
        KEYVAL(2, 3, 23/*KEY_N*/), //N
        KEYVAL(2, 4, 24/*KEY_UP*/), //UP
        KEYVAL(2, 5, 25/*KEY_MP3*/), //MP3 -> no implement
        KEYVAL(2, 6, 26/*KEY_E*/), //E
        KEYVAL(2, 7, 27/*KEY_HOMEPAGE*/), //home page -> no implement
       // 3 row
        KEYVAL(3, 0, 30/*KEY_MENU*/), //ok
        KEYVAL(3, 1, 31/*KEY_U*/), //U
        KEYVAL(3, 2, 32/*KEY_J*/), //J
        KEYVAL(3, 3, 33/*KEY_M*/), //M
        KEYVAL(3, 4, 34/*KEY_DOWN*/), //down
        KEYVAL(3, 5, 35/*KEY_HELP*/), //Enter -> no implement
        KEYVAL(3, 6, 36/*KEY_I*/), //I
        KEYVAL(3, 7, 37/*KEY_HELP*/), //notepad -> no implement
	// 4 row
        KEYVAL(4, 0, 40/*KEY_VOLUMEDOWN*/), //V-
        KEYVAL(4, 1, 41/*KEY_VOLUMEUP*/), //V+
        KEYVAL(4, 2, 42/*KEY_DELETE*/), //DEL
        KEYVAL(4, 3, 43/*KEY_HELP*/), //char ctrl -> no implement
        KEYVAL(4, 4, 44/*KEY_LEFTALT*/), //ALT
        KEYVAL(4, 5, 45/*KEY_LEFTSHIFT*/), //shift
        KEYVAL(4, 6, 46/*KEY_O*/), //O
        KEYVAL(4, 7, 47/*KEY_INFO*/), //information -> no implement
	// 5 row
        KEYVAL(5, 0, 50/*KEY_Z*/), //Z
        KEYVAL(5, 1, 51/*KEY_L*/), //L
        KEYVAL(5, 2, 52/*KEY_K*/), //K
        KEYVAL(5, 3, 53/*KEY_D*/), //D
        KEYVAL(5, 4, 54/*KEY_S*/), //S
        KEYVAL(5, 5, 55/*KEY_A*/), //A
        KEYVAL(5, 6, 56/*KEY_P*/), //P
        KEYVAL(5, 7, 57/*KEY_BACKSPACE*/), //space -> no implement
	// 6 row
        KEYVAL(6, 0, 60/*KEY_X*/), //X
        KEYVAL(6, 1, 61/*KEY_C*/), //C
        KEYVAL(6, 2, 62/*KEY_COMMA*/), // ,
        KEYVAL(6, 3, 63/*KEY_DOT*/), // .
        KEYVAL(6, 4, 64/*KEY_HELP*/), // '& -> no implement
        KEYVAL(6, 5, 65/*KEY_HELP*/), //! -> no implement
        KEYVAL(6, 6, 66/*KEY_SLASH*/), // /
        KEYVAL(6, 7, 67/*KEY_CALENDAR*/), //calendar -> no implement
	// 7 row
        KEYVAL(7, 0, 70/*KEY_HELP*/), //big black key -> no implement
        KEYVAL(7, 1, 71/*KEY_HELP*/), //poweron / power off -> no implement
#endif
};


static struct sprd_kpad_platform_data sprd_kpad_data = {
#ifdef CONFIG_MACH_G2PHONE
        .rows                   = 5,
        .cols                   = 5,
#endif
#ifdef CONFIG_MACH_OPENPHONE
        .rows                   = 8,
        .cols                   = 8,
#endif
        .keymap                 = sprd_keymap,
        .keymapsize             = ARRAY_SIZE(sprd_keymap),
        .repeat                 = 0,
        .debounce_time          = 5000, /* ns (5ms) */
        .coldrive_time          = 1000, /* ns (1ms) */
        .keyup_test_interval    = 50, /* 50 ms (50ms) */
};

static unsigned long keypad_func_cfg[] = {
	MFP_CFG_X(KEYOUT0, AF0, DS1, F_PULL_NONE, S_PULL_NONE, IO_OE),
	MFP_CFG_X(KEYOUT1, AF0, DS1, F_PULL_NONE, S_PULL_NONE, IO_OE),
	MFP_CFG_X(KEYOUT2, AF0, DS1, F_PULL_NONE, S_PULL_NONE, IO_OE),
	MFP_CFG_X(KEYOUT3, AF0, DS1, F_PULL_NONE, S_PULL_NONE, IO_OE),
	MFP_CFG_X(KEYOUT4, AF0, DS1, F_PULL_NONE, S_PULL_NONE, IO_OE),
	MFP_CFG_X(KEYOUT5, AF0, DS1, F_PULL_NONE, S_PULL_NONE, IO_OE),
	MFP_CFG_X(KEYOUT6, AF0, DS1, F_PULL_NONE, S_PULL_NONE, IO_OE),
	MFP_CFG_X(KEYOUT7, AF0, DS1, F_PULL_NONE, S_PULL_NONE, IO_OE),
	MFP_CFG_X(KEYIN0,  AF0, DS1, F_PULL_UP,   S_PULL_UP,   IO_IE),
	MFP_CFG_X(KEYIN1,  AF0, DS1, F_PULL_UP,   S_PULL_UP,   IO_IE),
	MFP_CFG_X(KEYIN2,  AF0, DS1, F_PULL_UP,   S_PULL_UP,   IO_IE),
	MFP_CFG_X(KEYIN3,  AF0, DS1, F_PULL_UP,   S_PULL_UP,   IO_IE),
	MFP_CFG_X(KEYIN4,  AF0, DS1, F_PULL_UP,   S_PULL_UP,   IO_IE),
	MFP_CFG_X(KEYIN5,  AF0, DS1, F_PULL_UP,   S_PULL_UP,   IO_IE),
	MFP_CFG_X(KEYIN6,  AF0, DS1, F_PULL_UP,   S_PULL_UP,   IO_IE),
	MFP_CFG_X(KEYIN7,  AF0, DS1, F_PULL_UP,   S_PULL_UP,   IO_IE),
};

static void sprd_config_keypad_pins(void)
{
	sprd_mfp_config(keypad_func_cfg, ARRAY_SIZE(keypad_func_cfg));
}

struct sprd_kpad_t {
        struct input_dev *input;
        int irq;
        unsigned short *keycode;
        unsigned int keyup_test_jiffies;
};

typedef struct kpd_key_tag
{
    unsigned short key_code;
    unsigned char state;
    unsigned char count;
    unsigned long time_stamp;
    unsigned long timer_id;
} kpd_key_t;

struct timer_list s_kpd_timer[MAX_MUL_KEY_NUM];
kpd_key_t s_key[MAX_MUL_KEY_NUM];
struct sprd_kpad_t *sprd_kpad;

void clear_key(kpd_key_t *key_ptr)
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
                        unsigned short keymapsize)
{
        unsigned int i;

        for (i = 0; i < keymapsize; i++) {
                keycode[i] = pdata_kc[i] & 0xffff;
                keycode[i + keymapsize] = pdata_kc[i] >> 16;
        }
}

static void print_kpad(void)
{
	printk("\n\nREG_INT_MASK_STS = 0x%08x\n", REG_INT_MASK_STS);
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
	printk("REG_PIN_KEYIN5_REG = 0x%08x\n", REG_PIN_KEYIN5_REG);
	printk("REG_PIN_KEYIN6_REG = 0x%08x\n", REG_PIN_KEYIN6_REG);
}

void change_state(kpd_key_t *key_ptr)
{   
	unsigned short rowcol, key;
    	/* Change state of the key according to it's current state */
    	if (TB_KPD_RELEASED == key_ptr->state) {
		/* Change state from TB_KPD_RELEASED to TB_KPD_PRESSED */
        	key_ptr->state  = TB_KPD_PRESSED;
        	key_ptr->count  = 1;
		mod_timer(&s_kpd_timer[key_ptr->timer_id], jiffies + CHECK_TIMER_EXPIRE);
		rowcol = SCAN2KEYVAl(key_ptr->key_code);
		key = sprd_kpad_find_key(sprd_kpad, sprd_kpad->input, rowcol);
        	input_report_key(sprd_kpad->input, key, 1);
        	input_sync(sprd_kpad->input);
		printk("%dD\n", key);
	} else {
        	/* Change state from TB_KPD_PRESSED to TB_KPD_RELEASED */
		rowcol = SCAN2KEYVAl(key_ptr->key_code);
		key = sprd_kpad_find_key(sprd_kpad, sprd_kpad->input, rowcol);		
        	input_report_key(sprd_kpad->input, key, 0);
        	input_sync(sprd_kpad->input);	
        	clear_key(key_ptr);
		printk("%dU\n", key);
    	}
}

unsigned long handle_key(unsigned short key_code, kpd_key_t *key_ptr)
{
    unsigned long status = 0;
    
    switch(key_ptr->state) {
	case TB_KPD_RELEASED:
        	/* check if it is the first INT of a key */
        	if (key_ptr->count == 0) {
            		/* Always discard the first key val */
           		key_ptr->key_code   = TB_KPD_INVALID_KEY;
            		/* Add count of INT to avoid dead loop */
            		key_ptr->count ++;
        	} else { //if (key_ptr->count == 0)
            		/* Check if this key is the same as the previous key, if same, handle it, else ignore the previous key */
            		if (IS_KEY_VALID(key_code)) {
                		/* Can't be TB_KPD_INVALID_KEY */
                		if (key_code == TB_KPD_INVALID_KEY) {
					printk("assert error : %s  %s  %d\n", __FILE__, __FUNCTION__, __LINE__);
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
	kpd_key_t *key_ptr = (kpd_key_t *)data;
	
	/* Check if the key is released, if the state is TB_KPD_PRESSED and count is 0, it means the key is released */
    	if (key_ptr->state == TB_KPD_PRESSED) {
        	if (key_ptr->count == 0) {
            		change_state(key_ptr);
        	} else {
            		key_ptr->count = 0;
			mod_timer(&s_kpd_timer[key_ptr->timer_id], jiffies + CHECK_TIMER_EXPIRE);
		}
    	}
}

static irqreturn_t sprd_kpad_isr(int irq, void *dev_id)
{
        int i;
	unsigned short  key_code = 0;
	kpd_key_t *key_ptr = NULL;
	unsigned long found = 0, status; 
	unsigned long s_int_status = REG_KPD_INT_RAW_STATUS;
	unsigned long s_key_status = REG_KPD_KEY_STATUS;

	REG_KPD_INT_CLR |= KPD_INT_ALL;
	
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
	
        return IRQ_HANDLED;
}

static int __devinit sprd_kpad_probe(struct platform_device *pdev)
{
        struct sprd_kpad_platform_data *pdata;
        struct input_dev *input;
        int i, error, key_type;
	
	pdev->dev.platform_data = &sprd_kpad_data;
	pdata = pdev->dev.platform_data;
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
        sprd_kpad->keycode = kmalloc(pdata->keymapsize * sizeof(unsigned short) * 2, GFP_KERNEL);
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
        REG_INT_DIS = (1 << IRQ_KPD_INT);
        REG_GR_GEN0 |= BIT_8 | BIT_26;
        sprd_config_keypad_pins();
        REG_KPD_INT_CLR = KPD_INT_ALL;
        REG_KPD_POLARITY = CFG_ROW_POLARITY | CFG_COL_POLARITY;
        REG_KPD_CLK_DIV_CNT = CFG_CLK_DIV & KPDCLK0_CLK_DIV0;
	REG_KPD_LONG_KEY_CNT = 0xc;
	REG_KPD_DEBOUNCE_CNT = 0x5;//0x8;0x13

	key_type = ((((~(0xffffffff << (pdata->cols - KPD_COL_MIN_NUM))) << 20) | ((~(0xffffffff << (pdata->rows - KPD_ROW_MIN_NUM))) << 16)) & (KPDCTL_ROW | KPDCTL_COL));
	REG_KPD_CTRL = 0x7 | key_type;

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
        input->phys = "sprd-keypad/input0";
        input->dev.parent = &pdev->dev;
	input_set_drvdata(input, sprd_kpad);

        input->id.bustype = BUS_HOST;
        input->id.vendor = 0x0001;
        input->id.product = 0x0001;
        input->id.version = 0x0100;

        input->keycodesize = sizeof(unsigned short);
        input->keycodemax = pdata->keymapsize;
        input->keycode = sprd_kpad->keycode;

        sprd_keycodecpy(sprd_kpad->keycode, pdata->keymap, pdata->keymapsize);
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
	for (i = 0; i < MAX_MUL_KEY_NUM; i++) {
		/* clear Key state */
		clear_key(&s_key[i]);
		/* create a timer to check if key is released */
		setup_timer(&s_kpd_timer[i], sprd_kpad_timer, (unsigned long) &s_key[i]);
		s_key[i].timer_id = i;						        
	}

	REG_KPD_INT_EN = KPD_INT_ALL;
	REG_INT_EN |= 1 << IRQ_KPD_INT;
	//print_kpad();

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

	for (i = 0; i < MAX_MUL_KEY_NUM; i++)
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
