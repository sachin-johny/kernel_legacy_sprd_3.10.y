#include <linux/module.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <mach/mfp.h>
#include <mach/sprd_key.h>
#ifdef CONFIG_ARCH_SC8810
#include <mach/eic.h>
#endif

#define CUSTOM_KEYPAD_ROWS		3
#define CUSTOM_KEYPAD_COLS		3

static const unsigned int sprd_keymap[] = {
	// 0 row
	KEYVAL(0, 0,	ANDROID_KEY_VOLUME_DOWN),   
	KEYVAL(0, 1,	ANDROID_KEY_CAMERA),
	// 1 row
	KEYVAL(1, 0,	ANDROID_KEY_VOLUME_UP),       
};


int sprd_3rdparty_gpio_key0 = 163;
/*note: now driver support max 3 gpio keys*/
static struct sprd_gpio_keys sprd_gpio_keymap[] = 
{
	{&sprd_3rdparty_gpio_key0	,ANDROID_KEY_POWER},
};

struct sprd_kpad_platform_data sprd_kpad_data = {
	.rows					=	CUSTOM_KEYPAD_ROWS,
	.cols					=	CUSTOM_KEYPAD_COLS,
        .keymap					=	sprd_keymap,
        .keymapsize				=	ARRAY_SIZE(sprd_keymap),
        .gpio_keymap			= 	sprd_gpio_keymap,
        .gpio_keymapsize		=	ARRAY_SIZE(sprd_gpio_keymap),
        .repeat					=	0,
        .debounce_time			=	5000,	/* ns (5ms) */
        .coldrive_time			=	1000,	/* ns (1ms) */
        .keyup_test_interval		=	50,		/* 50 ms (50ms) */
};


