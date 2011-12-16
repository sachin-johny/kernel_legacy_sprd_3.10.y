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
#include <mach/mfp.h>
#include "sprd_key.h"
#include "regs_kpd_sc8800g.h"
#include <mach/regs_cpc.h>

#include <mach/regs_ana.h>
#include <mach/regs_gpio.h>
#include <mach/adi_hal_internal.h>

#ifdef CONFIG_ARCH_SC8810
#include <mach/eic.h>
#endif

#ifdef CONFIG_MACH_OPENPHONE
#define DRV_NAME        	"sprd-keypad"
#endif

#ifdef CONFIG_MACH_SP6810A
#define DRV_NAME        	"sprd-keypad6810"
#endif

#ifdef CONFIG_MACH_SP8805GA
#define DRV_NAME        	"sprd-keypad8805ga"
#endif

#ifdef CONFIG_MACH_SP8810
#define DRV_NAME        	"sprd-keypad8810"
#endif

#ifdef CONFIG_MACH_SC8810OPENPHONE
#define DRV_NAME        	"sprd-keypad8810"
#endif

#ifndef DRV_NAME
#define DRV_NAME "sprd-keypad8805ga"
#endif

#define REG_INT_MASK_STS        (*((volatile unsigned int *) (SPRD_INTCV_BASE + 0x0000)))
#define REG_INT_RAW_STS        	(*((volatile unsigned int *)(SPRD_INTCV_BASE + 0x0004)))
#define REG_INT_EN              (*((volatile unsigned int *)(SPRD_INTCV_BASE + 0x0008)))
#define REG_INT_DIS          	(*((volatile unsigned int *)(SPRD_INTCV_BASE + 0x000C)))



#define REG_PIN_CTL_REG          	(*((volatile unsigned int *)PIN_CTL_REG))

#define REG_PIN_KEYOUT0_REG          	(*((volatile unsigned int *)PIN_KEYOUT0_REG))
#define REG_PIN_KEYOUT1_REG          	(*((volatile unsigned int *)PIN_KEYOUT1_REG))
#define REG_PIN_KEYOUT2_REG          	(*((volatile unsigned int *)PIN_KEYOUT2_REG))
#define REG_PIN_KEYOUT3_REG          	(*((volatile unsigned int *)PIN_KEYOUT3_REG))
#define REG_PIN_KEYOUT4_REG          	(*((volatile unsigned int *)PIN_KEYOUT4_REG))
#define REG_PIN_KEYOUT5_REG          	(*((volatile unsigned int *)PIN_KEYOUT5_REG))
#define REG_PIN_KEYOUT6_REG          	(*((volatile unsigned int *)PIN_KEYOUT6_REG))
#define REG_PIN_KEYOUT7_REG          	(*((volatile unsigned int *)PIN_KEYOUT7_REG))

#define REG_PIN_KEYIN0_REG          	(*((volatile unsigned int *)PIN_KEYIN0_REG))
#define REG_PIN_KEYIN1_REG          	(*((volatile unsigned int *)PIN_KEYIN1_REG))
#define REG_PIN_KEYIN2_REG          	(*((volatile unsigned int *)PIN_KEYIN2_REG))
#define REG_PIN_KEYIN3_REG          	(*((volatile unsigned int *)PIN_KEYIN3_REG))
#define REG_PIN_KEYIN4_REG          	(*((volatile unsigned int *)PIN_KEYIN4_REG))
#define REG_PIN_KEYIN5_REG          	(*((volatile unsigned int *)PIN_KEYIN5_REG))
#define REG_PIN_KEYIN6_REG          	(*((volatile unsigned int *)PIN_KEYIN6_REG))
#define REG_PIN_KEYIN7_REG          	(*((volatile unsigned int *)PIN_KEYIN7_REG))

#define REG_GR_GEN0             (*((volatile unsigned int *)GR_GEN0))

#define KPD_ROW_MIN_NUM         4  /* when config keypad type, the value of */
#define KPD_COL_MIN_NUM         3  /* when config keypad type, the value of */
#define KPD_ROW_MAX_NUM         8  /* when config keypad type, the value of */
#define KPD_COL_MAX_NUM         8  /* when config keypad type, the value of */

#define KPDCTL_ROW              (0xf << 16)  /* enable bit for rows(row4 --- row7) */
#define KPDCTL_COL              (0x1f << 20)  /* enable bit for cols(col3 --- col4) */

#define MAX_MUL_KEY_NUM		3
/* keypad constant */
#define TB_KPD_CONST_BASE       (0x80)
#define TB_KPD_RELEASED         (TB_KPD_CONST_BASE)
#define TB_KPD_PRESSED          (TB_KPD_CONST_BASE + 1)
#define TB_KPD_INVALID_KEY      (0x0FFFF)

#define POWRER_KEY_VAL           11
#define CHECK_TIMER_EXPIRE      (5)//ms
#define CHECK_POWER_TIMER_EXPIRE (15)//ms
#define AVOID_QUIVER_MIN_COUNT  (1)

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

#define KPDPOLARITY_ROW                 (0x00FF)
#define KPDPOLARITY_COL                 (0xFF00)

#define KPDCLK0_CLK_DIV0                0xFFFF      //Clock dividor [15:0]
#define KPDCLK1_TIME_CNT                0xFFB0      //Time out counter value

#define CFG_ROW_POLARITY    (0x00FF & KPDPOLARITY_ROW)
#define CFG_COL_POLARITY    (0xFF00 & KPDPOLARITY_COL)
#define CFG_CLK_DIV         1

#if defined(CONFIG_MACH_SP6810A) || defined(CONFIG_MACH_SP8805GA) || defined (CONFIG_MACH_SP8810) || defined(CONFIG_MACH_SC8810OPENPHONE)
#define HOME_KEY_GPIO		28
#define VOLUP_KEY_GPIO		25
#define PBINT_GPI		163

#define ANA_GPIO_IRQ        	BIT_1
#define ANA_INT_EN              (SPRD_MISC_BASE + 0x380 + 0x08)
#endif

static const unsigned int sprd_keymap[] = {
#if defined(CONFIG_MACH_G2PHONE)
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
#if defined(CONFIG_MACH_OPENPHONE)
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
	KEYVAL(7, 0, 30/*KEY_MENU*/), //ok //yunlong.wang add for ofn key detect 20110218
        KEYVAL(7, 1, 71/*KEY_HELP*/), //poweron / power off -> no implement
#endif
#if defined(CONFIG_MACH_SC8810OPENPHONE)
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
	KEYVAL(7, 0, 30/*KEY_MENU*/), //ok //yunlong.wang add for ofn key detect 20110218
        KEYVAL(7, 1, 71/*KEY_HELP*/), //poweron / power off -> no implement
#endif
#if defined(CONFIG_MACH_SP8805GA)
        // 0 row
	KEYVAL(0, 0, 22/*KEY_SEND*/), //dial up 1
        KEYVAL(0, 1, 01/*KEY_R*/), //R
        // 1 row
        KEYVAL(1, 0, 10/*KEY_SEND*/), //dial up 2 -> no implement
	KEYVAL(1, 1, 11/*KEY_T*/), //T
#endif
#if defined(CONFIG_MACH_SP6810A)
        // 0 row
	KEYVAL(0, 0, 24/*KEY_SEND*/), // 00 is changed to 24
        KEYVAL(0, 1, 01/*KEY_R*/), //R
        KEYVAL(0, 2, 02/*KEY_F*/), //F
        KEYVAL(0, 3, 03/*KEY_V*/), //V
        KEYVAL(0, 4, 04/*KEY_LEFT*/), //left
        // 1 row
        KEYVAL(1, 0, 10/*KEY_SEND*/), //dial up 2 -> no implement
	KEYVAL(1, 1, 11/*KEY_T*/), //T
        KEYVAL(1, 2, 12/*KEY_G*/), //G
	KEYVAL(1, 3, 13/*KEY_B*/), //B
        KEYVAL(1, 4, 14/*KEY_RIGHT*/), //right
       // 2 row
        KEYVAL(2, 0, 20/*KEY_END*/), //hang up
        KEYVAL(2, 1, 21/*KEY_Y*/), //Y
        KEYVAL(2, 2, 22/*KEY_H*/), //H
        KEYVAL(2, 3, 23/*KEY_N*/), //N
        KEYVAL(2, 4, 24/*KEY_UP*/), //UP
       // 3 row
        KEYVAL(3, 0, 30/*KEY_MENU*/), //ok
        KEYVAL(3, 1, 31/*KEY_U*/), //U
        KEYVAL(3, 2, 32/*KEY_J*/), //J
        KEYVAL(3, 3, 33/*KEY_M*/), //M
        KEYVAL(3, 4, 34/*KEY_DOWN*/), //down
	// 4 row
        KEYVAL(4, 0, 40/*KEY_VOLUMEDOWN*/), //V-
        KEYVAL(4, 1, 41/*KEY_VOLUMEUP*/), //V+
        KEYVAL(4, 2, 42/*KEY_DELETE*/), //DEL
        KEYVAL(4, 3, 43/*KEY_HELP*/), //char ctrl -> no implement
        KEYVAL(4, 4, 44/*KEY_LEFTALT*/), //ALT
#endif
#if defined (CONFIG_MACH_SP8810)
           						// 0 row
		KEYVAL(0, 0, 40),		//d
		KEYVAL(1, 0, 41),        //u
        KEYVAL(0, 1, 30),  //cam
		KEYVAL(1, 1, 11),
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
#ifdef CONFIG_MACH_SC8810OPENPHONE
        .rows                   = 8,
        .cols                   = 8,
#endif
#ifdef CONFIG_MACH_SP8805GA
        .rows                   = 4,
        .cols                   = 3,
#endif
#if defined (CONFIG_MACH_SP8810)
	.rows					= 2,
	.cols					= 2,
#endif
#ifdef CONFIG_MACH_SP6810A
        .rows                   = 5,
        .cols                   = 5,
#endif
        .keymap                 = sprd_keymap,
        .keymapsize             = ARRAY_SIZE(sprd_keymap),
        .repeat                 = 0,
        .debounce_time          = 5000, /* ns (5ms) */
        .coldrive_time          = 1000, /* ns (1ms) */
        .keyup_test_interval    = 50, /* 50 ms (50ms) */
};

#if defined(CONFIG_MACH_SP8810) || defined(CONFIG_MACH_SC8810OPENPHONE)
static unsigned long keypad_func_cfg[] = {
	MFP_CFG_X(KEYOUT0, AF0, DS1, F_PULL_NONE, S_PULL_NONE, IO_OE),
	MFP_CFG_X(KEYOUT1, AF0, DS1, F_PULL_NONE, S_PULL_NONE, IO_OE),
	MFP_CFG_X(KEYIN0,  AF0, DS1, F_PULL_UP,   S_PULL_UP,   IO_IE),
	MFP_CFG_X(KEYIN1,  AF0, DS1, F_PULL_UP,   S_PULL_UP,   IO_IE),
};

static void sprd_config_keypad_pins(void)
{
	sprd_mfp_config(keypad_func_cfg, ARRAY_SIZE(keypad_func_cfg));
}
#endif

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

#if defined(CONFIG_MACH_G2PHONE) || defined(CONFIG_MACH_OPENPHONE)
struct timer_list s_kpd_timer[MAX_MUL_KEY_NUM];
kpd_key_t s_key[MAX_MUL_KEY_NUM];
#elif defined(CONFIG_MACH_SP6810A) || defined(CONFIG_MACH_SP8805GA) || defined (CONFIG_MACH_SP8810) || defined(CONFIG_MACH_SC8810OPENPHONE)
struct timer_list s_kpd_timer[MAX_MUL_KEY_NUM + 3];
kpd_key_t s_key[MAX_MUL_KEY_NUM + 3];
#else
struct timer_list s_kpd_timer[MAX_MUL_KEY_NUM + 3];
kpd_key_t s_key[MAX_MUL_KEY_NUM + 3];
#endif

struct sprd_kpad_t *sprd_kpad;


//yunlong.wang add for key emulator
static unsigned char g_keycode;
static ssize_t sprd_kpad_store_emulate(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t sprd_kpad_show_emulate(struct device* cd,struct device_attribute *attr, char* buf);
static DEVICE_ATTR(emulate, S_IRUGO | S_IWUSR, sprd_kpad_show_emulate, sprd_kpad_store_emulate);
 
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

static int sprd_kpad_create_sysfs(struct platform_device *pdev)
{
	int err;
	struct device *dev = &(pdev->dev);
	
	err = device_create_file(dev, &dev_attr_emulate);

	return err;
}

//yunlong.wang add END


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
#if !defined(CONFIG_MACH_SP8810) && !defined(CONFIG_MACH_SC8810OPENPHONE) 
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

void change_state(kpd_key_t *key_ptr)
{   
	unsigned short rowcol, key;
    	/* Change state of the key according to it's current state */
    	if (TB_KPD_RELEASED == key_ptr->state) {
		/* Change state from TB_KPD_RELEASED to TB_KPD_PRESSED */
        	key_ptr->state  = TB_KPD_PRESSED;
        	key_ptr->count  = 1;
		rowcol = SCAN2KEYVAl(key_ptr->key_code);
		key = sprd_kpad_find_key(sprd_kpad, sprd_kpad->input, rowcol);
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
	unsigned short rowcol, key;
	kpd_key_t *key_ptr = (kpd_key_t *)data;
	
	/* Check if the key is released, if the state is TB_KPD_PRESSED and count is 0, it means the key is released */
    	if (key_ptr->state == TB_KPD_PRESSED) {
        	if (key_ptr->count == 0) {
            		change_state(key_ptr);
        	} else {
           		key_ptr->count = 0;
                rowcol = SCAN2KEYVAl(key_ptr->key_code);
                key = sprd_kpad_find_key(sprd_kpad, sprd_kpad->input, rowcol);
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
	kpd_key_t *key_ptr = NULL;
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
	
        return IRQ_HANDLED;
}

#if defined(CONFIG_MACH_SP6810A) || defined(CONFIG_MACH_SP8805GA) || defined(CONFIG_MACH_SP8810) || defined(CONFIG_MACH_SC8810OPENPHONE)

static int irq_is_detected  = 0;

static irqreturn_t sprd_gpio_isr(int irq, void *dev_id)
{
    	int ret, gpio;
	unsigned short  key_code = 0;
	kpd_key_t *key_ptr = NULL;
	unsigned long found = 0, status; 
	unsigned long s_int_status;
	unsigned long s_key_status;
	
	if (irq_is_detected ==  1) {
		printk("irq = %d\n", irq);
		gpio = irq; 
	}
	else {
		msleep(20);
		gpio = irq_to_gpio(irq);
	}
	//printk("%s %d  gpio = %d\n", __FUNCTION__, __LINE__, gpio);
	if (gpio == HOME_KEY_GPIO) {
		ret = gpio_get_value(gpio);
		if (ret) {
			//printk("The Pin is HIGH\n");
			s_int_status =0x00000080;
			s_key_status =0x21777777;
		} else {
			//printk("The Pin is LOW\n");
			s_int_status =0x00000008;
			s_key_status =0xa1777777;
		}

		if ((s_int_status & KPD_PRESS_INT3) || (s_int_status & KPD_LONG_KEY_INT3)) {
	       		key_code = (s_key_status  & (KPD4_ROW_CNT | KPD4_COL_CNT)) >> 24;
			/* if key_code is stored, don't seat the code again */
	    		key_ptr = &s_key[MAX_MUL_KEY_NUM];
    			if (key_ptr->key_code == key_code) {
	    			handle_key(key_code, key_ptr);	    	
	    			found = 1;
    			}
	   		if (!found) {
				/* the key_code is fresh, try to find a seat other than exceed the seat limit */
				key_ptr = &s_key[MAX_MUL_KEY_NUM];
				status = handle_key(key_code, key_ptr);
			}
		} else if (s_int_status & KPD_RELEASE_INT3) {
			if (TB_KPD_RELEASED == s_key[MAX_MUL_KEY_NUM].state) {
				clear_key(&s_key[MAX_MUL_KEY_NUM]);
			}
		}
	}//if (gpio == HOME_KEY_GPIO)


	if (gpio == VOLUP_KEY_GPIO) {
		ret = gpio_get_value(gpio);
		if (ret) {
			//printk("The Pin is HIGH\n");
			s_int_status =0x00000080;
			s_key_status =0x20777777;
		} else {
			//printk("The Pin is LOW\n");
			s_int_status =0x00000008;
			s_key_status =0xa0777777;
		}

		if ((s_int_status & KPD_PRESS_INT3) || (s_int_status & KPD_LONG_KEY_INT3)) {
	       		key_code = (s_key_status  & (KPD4_ROW_CNT | KPD4_COL_CNT)) >> 24;
			/* if key_code is stored, don't seat the code again */
	    		key_ptr = &s_key[MAX_MUL_KEY_NUM + 1];
    			if (key_ptr->key_code == key_code) {
	    			handle_key(key_code, key_ptr);	    	
	    			found = 1;
    			}
			
	   		if (!found) {
				/* the key_code is fresh, try to find a seat other than exceed the seat limit */
				key_ptr = &s_key[MAX_MUL_KEY_NUM + 1];
				status = handle_key(key_code, key_ptr);
			}
		} else if (s_int_status & KPD_RELEASE_INT3) {
			if (TB_KPD_RELEASED == s_key[MAX_MUL_KEY_NUM + 1].state) {
				clear_key(&s_key[MAX_MUL_KEY_NUM + 1]);
			}
		}
	}//if (gpio == VOLUP_KEY_GPIO)

	if (gpio == PBINT_GPI) {
#if  defined (CONFIG_MACH_SP8810) || defined(CONFIG_MACH_SC8810OPENPHONE)
		if (irq_is_detected ==  1) 
			ret = sprd_get_eic_data(EIC_ID_11);
		else
#endif
			ret = gpio_get_value(gpio);
		if (ret) {
			//printk("The Pin is HIGH\n");
			s_int_status =0x00000080;
			s_key_status =0x11777777;
		} else {
			//printk("The Pin is LOW\n");
			s_int_status =0x00000008;
			s_key_status =0x91777777;
		}

		if ((s_int_status & KPD_PRESS_INT3) || (s_int_status & KPD_LONG_KEY_INT3)) {
	       		key_code = (s_key_status  & (KPD4_ROW_CNT | KPD4_COL_CNT)) >> 24;
			/* if key_code is stored, don't seat the code again */
	    		key_ptr = &s_key[MAX_MUL_KEY_NUM + 2];
    			if (key_ptr->key_code == key_code) {
	    			handle_key(key_code, key_ptr);	    	
	    			found = 1;
    			}

	   		if (!found) {
				/* the key_code is fresh, try to find a seat other than exceed the seat limit */
				key_ptr = &s_key[MAX_MUL_KEY_NUM + 2];
				status = handle_key(key_code, key_ptr);
			}
		} else if (s_int_status & KPD_RELEASE_INT3) {
			if (TB_KPD_RELEASED == s_key[MAX_MUL_KEY_NUM + 2].state) {
				clear_key(&s_key[MAX_MUL_KEY_NUM + 2]);
			}
		}
	}//if (gpio == PBINT_GPI)


        return IRQ_HANDLED;
}

static irqreturn_t sprd_pint_isr(int irq, void *dev_id)
{	
	irqreturn_t ret = 0;
	printk("sprd_pint_isr\n");
	irq = PBINT_GPI;
	irq_is_detected = 1;
	
	ret = sprd_gpio_isr( irq, dev_id);
	irq_is_detected = 0;
	return ret;
}
static void gpio_key_init(unsigned long gpio, const char *label)
{
	unsigned long err, irq, ret;

	err = gpio_request(gpio, label);
	if (err)
		printk("can not alloc gpio for %s Key\n", label);
	else
		printk("alloc gpio for %s Key\n", label);

	gpio_direction_input(gpio);
	irq = sprd_alloc_gpio_irq(gpio);
	ret = request_threaded_irq(irq, NULL, sprd_gpio_isr, IRQF_TRIGGER_LOW | IRQF_ONESHOT, label, NULL);
	ret = gpio_get_value(gpio);
	if (ret) {
		printk("The Pin is HIGH, so set low level trigger. irq = %ld\n", irq);
		set_irq_type(irq, IRQF_TRIGGER_LOW);
	} else {
		printk("The Pin is LOW, HOME pin configuration is wrong\n");
	}
}

#if  defined (CONFIG_MACH_SP8810) || defined(CONFIG_MACH_SC8810OPENPHONE)
static void int_key_init(enum EIC_TYPE_E eic_id, const char *label)
{
	unsigned long err, irq, ret;
	int data = 0;

	ret = sprd_alloc_eic_irq(eic_id);
	if (ret != -1) {
		irq = ret ;
		ret = request_threaded_irq(irq, NULL, sprd_pint_isr, IRQF_TRIGGER_LOW | IRQF_ONESHOT, "pb_int", NULL);
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

#endif

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
#if defined(CONFIG_MACH_SP8810) || (CONFIG_MACH_SC8810OPENPHONE)	
        sprd_config_keypad_pins();
#endif
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
#if defined(CONFIG_MACH_G2PHONE) || defined(CONFIG_MACH_OPENPHONE)
        input->phys = "sprd-keypad/input0";
#elif defined(CONFIG_MACH_SP6810A)
        input->phys = "sprd-keypad6810/input0";
#elif defined(CONFIG_MACH_SP8805GA)
        input->phys = "sprd-keypad8805ga/input0";
#elif defined(CONFIG_MACH_SP8810) || defined(CONFIG_MACH_SC8810OPENPHONE)
		input->phys = "sprd-keypad8810/input0";
#endif
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
#if defined(CONFIG_MACH_G2PHONE) || defined(CONFIG_MACH_OPENPHONE)
	for (i = 0; i < MAX_MUL_KEY_NUM; i++) {
#elif defined(CONFIG_MACH_SP6810A) || defined(CONFIG_MACH_SP8805GA) || defined(CONFIG_MACH_SP8810) || defined(CONFIG_MACH_SC8810OPENPHONE)
	for (i = 0; i < (MAX_MUL_KEY_NUM + 3); i++) {
#endif
		/* clear Key state */
		clear_key(&s_key[i]);
		/* create a timer to check if key is released */
		setup_timer(&s_kpd_timer[i], sprd_kpad_timer, (unsigned long) &s_key[i]);
		s_key[i].timer_id = i;						        
#if defined(CONFIG_MACH_G2PHONE) || defined(CONFIG_MACH_OPENPHONE) || defined(CONFIG_MACH_SP6810A) || defined(CONFIG_MACH_SP8805GA) || defined(CONFIG_MACH_SP8810) || defined(CONFIG_MACH_SC8810OPENPHONE)
	}
#endif

	REG_KPD_INT_EN = KPD_INT_ALL;
	REG_INT_EN |= 1 << IRQ_KPD_INT;
	REG_KPD_CTRL |= 0x1;

	//print_kpad();

#if defined(CONFIG_MACH_SP6810A)
	gpio_key_init(HOME_KEY_GPIO, "home");
	gpio_key_init(VOLUP_KEY_GPIO, "volup");
	gpio_key_init(PBINT_GPI, "poweronoff");	
	ANA_REG_OR(ANA_INT_EN, ANA_GPIO_IRQ);
#endif

#if defined(CONFIG_MACH_SP8805GA)
	gpio_key_init(PBINT_GPI, "poweronoff");	
	ANA_REG_OR(ANA_INT_EN, ANA_GPIO_IRQ);
#endif

#if  defined (CONFIG_MACH_SP8810) || defined(CONFIG_MACH_SC8810OPENPHONE)
	int_key_init(EIC_ID_11, 0);
#endif

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

#if defined(CONFIG_MACH_G2PHONE) || defined(CONFIG_MACH_OPENPHONE)
	for (i = 0; i < MAX_MUL_KEY_NUM; i++)
#elif defined(CONFIG_MACH_SP6810A)  || defined(CONFIG_MACH_SP8805GA) || defined (CONFIG_MACH_SP8810) || defined(CONFIG_MACH_SC8810OPENPHONE)
	for (i = 0; i < (MAX_MUL_KEY_NUM + 3); i++)
#endif
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
