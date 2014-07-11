#ifndef __LINUX_MSG2133A_H__
#define __LINUX_MSG2133A_H__
 
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <asm/uaccess.h>
#include <linux/earlysuspend.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/hrtimer.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/vmalloc.h>
#include <linux/io.h>
#include <mach/gpio.h>
#include <linux/irq.h>
#include <mach/regulator.h>
#include <linux/regulator/consumer.h>
#include <mach/board.h>

#define SPRD_DRV_GST_SUPPORT_TP_PROXIMITY
#ifdef SPRD_DRV_GST_SUPPORT_TP_PROXIMITY
#include <linux/spinlock.h>

#define TP_PROXIMITY_SENSOR
#define PROXIMITY_BUF_MAX_SIZE 0xF
#endif
#define TS_DEBUG_MSG	1
#define VIRTUAL_KEYS	1

#define CONFIG_GST_TP_COMPATIBLE
#ifdef CONFIG_GST_TP_COMPATIBLE
#define GST_TP_COMPATIBLE

//update fireware by sdcard
#define	__FIRMWARE_UPDATE__

#endif

#define SWAP_XY(x,y) {x=x+y; y=x-y;x=x-y;}
#define TS_WIDTH_MAX            320
#define TS_HEIGHT_MAX           480

#define PRINT_KEY_Y (TS_HEIGHT_MAX + 50)

#define MSG2133_BUS_NUM         3
#define MSG2133_TS_NAME         "msg2133a"
#define MSG2133_TS_ADDR         0x26

#define TPD_OK					0
#define TPD_REG_BASE            0x00
#define TPD_SOFT_RESET_MODE     0x01
#define TPD_OP_MODE             0x00
#define TPD_LOW_PWR_MODE        0x04
#define TPD_SYSINFO_MODE        0x10

#define GET_HSTMODE(reg)  ((reg & 0x70) >> 4)  // in op mode or not
#define GET_BOOTLOADERMODE(reg) ((reg & 0x10) >> 4)  // in bl mode


struct ts_event {
	u16	x1;
	u16	y1;
	u16	x2;
	u16	y2;
    u8  touch_point;
};

struct msg2133_ts_data {
	struct input_dev	    *input_dev;
	struct ts_event		    event;
	struct work_struct	    pen_event_work;
	struct workqueue_struct *ts_workqueue;
	struct early_suspend	early_suspend;
};
struct msg2133a_ts_platform_data {
	int irq_gpio_number;
	int reset_gpio_number;
	const char *vdd_name;
};

struct TouchScreenInfo_t{
    unsigned char nTouchKeyMode;
    unsigned char nTouchKeyCode;
    // unsigned char nFingerNum;
};

#endif
