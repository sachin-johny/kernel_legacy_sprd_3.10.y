#ifndef __LCD_CTL_H__
#define __LCD_CTL_H__

#define IN_KERNEL				1
#define IN_UBOOT				0

#define LCD_DEBUG				0

#if (IN_KERNEL == 1) && (IN_UBOOT == 1)
#error found "IN_KERNEL" and "IN_UBOOT" value confliction
#elif (IN_KERNEL == 0) && (IN_UBOOT == 0)
#error one of "IN_KERNEL" and "IN_UBOOT" must be set to take effect
#endif // (IN_KERNEL == 1) && (IN_UBOOT == 1)

#if IN_KERNEL
#include <linux/kernel.h>
#include <mach/lcd.h>
#if LCD_DEBUG
#define LCD_PRINT(format, args...) \
		do{printk(KERN_INFO "%s() _%d_: " format, __FUNCTION__ , __LINE__, ## args);}while(0)
#else
#define LCD_PRINT(format, args...) \
        do{}while(0)
#endif // LCD_DEBUG

#define LCD_DelayMS(x)				LCD_MS_DELAY(x)
#define LCD_MS_DELAY(x)				mdelay(x)
#endif // IN_KERNEL

#if IN_UBOOT
#include <asm/arch/sc8810_lcd.h>
#define mdelay(x)					LCD_MS_DELAY(x)
#define LCD_MS_DELAY(x)				LCD_DelayMS(x)
#define printk(format, args...)	\
		do{}while(0)
#define LCD_PRINT(format, args...) \
	  	do{}while(0)							
#endif // IN_UBOOT

#endif // __LCD_CTL_H__
