#ifndef _SPRD_KPAD_H
#define _SPRD_KPAD_H

#define		ANDROID_KEY_GRAVE			399
#define		ANDROID_KEY_1				2
#define		ANDROID_KEY_2				3
#define		ANDROID_KEY_3				4
#define		ANDROID_KEY_4				5
#define		ANDROID_KEY_5				6
#define 		ANDROID_KEY_6				7
#define		ANDROID_KEY_7				8
#define		ANDROID_KEY_8				9
#define		ANDROID_KEY_9				10
#define		ANDROID_KEY_0				11
#define		ANDROID_KEY_BACK			158
#define		ANDROID_KEY_SOFT_RIGHT		230
#define		ANDROID_KEY_SOFT_RIGHT1	60
#define		ANDROID_KEY_ENDCALL		107
#define		ANDROID_KEY_ENDCALL1		62
#define		ANDROID_KEY_MENU			229
#define		ANDROID_KEY_MENU1			139
#define		ANDROID_KEY_MENU2			59
#define		ANDROID_KEY_SEARCH			127
#define		ANDROID_KEY_SEARCH2		217
#define		ANDROID_KEY_POUND			228
#define		ANDROID_KEY_STAR			227
#define		ANDROID_KEY_CALL			231
#define		ANDROID_KEY_CALL1			61
#define		ANDROID_KEY_DPAD_CENTER	232
#define		ANDROID_KEY_DPAD_DOWN		108
#define		ANDROID_KEY_DPAD_UP		103
#define		ANDROID_KEY_HOME			102
#define		ANDROID_KEY_DPAD_LEFT		105
#define		ANDROID_KEY_DPAD_RIGHT		106
#define		ANDROID_KEY_VOLUME_UP		115
#define		ANDROID_KEY_VOLUME_DOWN	114
#define		ANDROID_KEY_POWER			116
#define		ANDROID_KEY_CAMERA			212
#define		ANDROID_KEY_Q				16
#define		ANDROID_KEY_W				17
#define		ANDROID_KEY_E				18
#define		ANDROID_KEY_R				19
#define		ANDROID_KEY_T				20
#define		ANDROID_KEY_Y				21
#define		ANDROID_KEY_U				22
#define		ANDROID_KEY_I				23
#define		ANDROID_KEY_O				24
#define		ANDROID_KEY_P				25
#define		ANDROID_KEY_LEFT_BRACKET	26
#define		ANDROID_KEY_RIGHT_BRACKET	27
#define		ANDROID_KEY_BACKSLASH		43
#define		ANDROID_KEY_A				30
#define		ANDROID_KEY_S				31
#define		ANDROID_KEY_D				32
#define		ANDROID_KEY_F				33
#define		ANDROID_KEY_G				34
#define		ANDROID_KEY_H				35
#define		ANDROID_KEY_J				36
#define		ANDROID_KEY_K				37
#define		ANDROID_KEY_L				38
#define		ANDROID_KEY_SEMICOLON		39
#define		ANDROID_KEY_APOSTROPHE	40
#define		ANDROID_KEY_DEL				14
#define		ANDROID_KEY_Z				44
#define		ANDROID_KEY_X				45
#define		ANDROID_KEY_C				46
#define		ANDROID_KEY_V				47
#define		ANDROID_KEY_B				48
#define		ANDROID_KEY_N				49
#define		ANDROID_KEY_M				50
#define		ANDROID_KEY_COMMA			51
#define		ANDROID_KEY_PERIOD			52
#define		ANDROID_KEY_SLASH			53
#define		ANDROID_KEY_ENTER			28
#define		ANDROID_KEY_ALT_LEFT		56
#define		ANDROID_KEY_ALT_RIGHT		100
#define		ANDROID_KEY_SHIFT_LEFT		42
#define		ANDROID_KEY_SHIFT_RIGHT		54
#define		ANDROID_KEY_TAB			15
#define		ANDROID_KEY_SPACE			57
#define		ANDROID_KEY_EXPLORER		150
#define 		ANDROID_KEY_ENVELOPE		155
#define		ANDROID_KEY_MINUS			12
#define		ANDROID_KEY_EQUALS			13
#define		ANDROID_KEY_AT				215

#define		ANDROID_KEY_HOME_PAGE		300
#define 		ANDROID_KEY_HELP				301



struct sprd_gpio_keys {
        int *gpio;
	int key_value;	
};


struct sprd_kpad_platform_data {
        int rows;
        int cols;
        const unsigned int *keymap;
        unsigned short keymapsize;
        struct sprd_gpio_keys *gpio_keymap;
        unsigned short gpio_keymapsize;
        unsigned short repeat;
        u32 debounce_time;      /* in ns */
        u32 coldrive_time;      /* in ns */
        u32 keyup_test_interval; /* in ms */
};

#if (defined CONFIG_KEYBOARD_SC8800G || defined CONFIG_KEYBOARD_SC8810)
#define KEYVAL(row, col, val) (((row) << 20) | ((col) << 16) | (val))
#define GPIO_KEYVAL(gpio, val) (((gpio) << 16) | (val))
#endif

#ifdef CONFIG_KEYBOARD_SC8800S
#define KEYVAL(col, row, val) (((col) << 24) | ((row) << 16) | (val))
#endif

#endif
