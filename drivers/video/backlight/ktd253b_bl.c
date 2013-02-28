/*************** maintained by customer ***************************************/
#if defined(CONFIG_MACH_MINT)
#define CONFIG_BACKLIGHT_CORI 1
#elif defined(CONFIG_MACH_NEVISTD)
#define CONFIG_BACKLIGHT_NEVISTD 1
#else
#define CONFIG_BACKLIGHT_AMAZING 1
#endif
/*
 * linux/drivers/video/backlight/s2c_bl.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
* 	@file	drivers/video/backlight/s2c_bl.c
*
* Unless you and Broadcom execute a separate written software license agreement
* governing use of this software, this software is licensed to you under the
* terms of the GNU General Public License version 2, available at
* http://www.gnu.org/copyleft/gpl.html (the "GPL").
*
* Notwithstanding the above, under no circumstances may you combine this
* software in any way with any other Broadcom software provided under a license
* other than the GPL, without Broadcom's express prior written consent.
*******************************************************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/ktd253b_bl.h>
//#include <plat/gpio.h>
#include <mach/gpio.h>
#include <linux/delay.h>
//#include <linux/broadcom/lcd.h>
#include <linux/spinlock.h>
//#include <linux/broadcom/PowerManager.h>

int current_intensity;
#if defined(CONFIG_MACH_MINT) || defined(CONFIG_MACH_NEVISTD)
static int backlight_pin = 138;
#else
static int backlight_pin = 136;
#endif

static DEFINE_SPINLOCK(bl_ctrl_lock);
int real_level = 1;
EXPORT_SYMBOL(real_level);

#ifdef CONFIG_HAS_EARLYSUSPEND
/* early suspend support */
//extern int gLcdfbEarlySuspendStopDraw;
#endif
static int backlight_mode=1;

#define MAX_BRIGHTNESS_IN_BLU	33

#define DIMMING_VALUE		32
#define MAX_BRIGHTNESS_VALUE	255
#define MIN_BRIGHTNESS_VALUE	20
#define BACKLIGHT_DEBUG 1
#define BACKLIGHT_SUSPEND 0
#define BACKLIGHT_RESUME 1

#if BACKLIGHT_DEBUG
#define BLDBG(fmt, args...) printk(fmt, ## args)
#else
#define BLDBG(fmt, args...)
#endif

struct ktd253b_bl_data {
	struct platform_device *pdev;
	unsigned int ctrl_pin;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend_desc;
#endif
};

struct brt_value{
	int level;				// Platform setting values
	int tune_level;			// Chip Setting values
};

#if defined(CONFIG_MACH_MINT)
struct brt_value brt_table_ktd[] = {
        { MIN_BRIGHTNESS_VALUE,  32 }, // Min pulse 32
        { 32,  31 },
        { 46,  30 },
        { 60,  29 },
        { 73,  28 },
        { 86,  27 },
        { 98,  26 },
        { 105,  25 },
        { 110,  24 },
        { 115,  23 },
        { 120,  22 },
        { 125,  21 },
        { 130,  20 }, //default value
        { 151,  19 },
        { 172,  18 },
        { 193,  17 },
        { 214,  16 },
        { 235,  15 },
        { MAX_BRIGHTNESS_VALUE,  14 },// Max pulse 1

};
#elif defined(CONFIG_BACKLIGHT_AMAZING)
struct brt_value brt_table_ktd[] = {
		{ MIN_BRIGHTNESS_VALUE,  1 }, // Min pulse
		{ 39,  2 }, 
		{ 47,  3 }, 
		{ 55,  4 },
		{ 63,  5 }, 
		{ 72,  6 }, 
		{ 81,  7 }, 
		{ 90,  8 }, 
		{ 99,  9 },  
		{ 107,  10 }, 
		{ 115,	11 }, 
		{ 123,	12 },	
		{ 131,	13 }, 	
		{ 139,	14 }, 		
		{ 147,	15 }, //default value  
		{ 153,	16 },  		
		{ 160,	17 },  
		{ 166,	18 },
		{ 173,	19 },
		{ 180,	20 },  
		{ 187,	21 },  		
		{ 194,	22 }, 
		{ 201,	23 },  		
		{ 208,	24 }, 
		{ 214,	25 },
		{ 220,	26 },  
		{ 227,	27 },  		
		{ 234,	28 }, 
		{ 241,	29 },  		
		{ 248,	30 }, 	
		{ MAX_BRIGHTNESS_VALUE,  31 }, // Max pulse

};

#elif defined(CONFIG_BACKLIGHT_KYLETD)
struct brt_value brt_table_ktd[] = {
	{ MIN_BRIGHTNESS_VALUE,  1 }, // Min pulse
	{ 38,  2 }, 
	{ 47,  3 }, 
	{ 55,  4 },
	{ 63,  5 }, 
	{ 71,  6 }, 
	{ 79,  7 }, 
	{ 87,  8 }, 
	{ 95,  9 },  
	{ 103,	10 }, 
	{ 111,	11 }, 
	{ 120,	12 },	
	{ 129,	13 }, 
	{ 138,	14 }, 
	{ 147,	15 }, //default value  
	{ 154,	15 },
	{ 162,	16 },
	{ 170,	16 },  
	{ 177,	17 },  
	{ 185,	17 },
	{ 193,	18 },
	{ 200,	18 }, 
	{ 208,	19 },
	{ 216,	20 },  
	{ 224,	22 }, 
	{ 231,	24 }, 
	{ 239,	26 },
	{ 247,	28 },  
	{ MAX_BRIGHTNESS_VALUE,  29 }, // Max pulse
};
#elif defined(CONFIG_BACKLIGHT_NEVISTD)
struct brt_value brt_table_ktd[] = {
	{ MIN_BRIGHTNESS_VALUE,  1 }, // Min pulse
	{ 38,  2 }, 
	{ 47,  3 }, 
	{ 55,  4 },
	{ 63,  5 }, 
	{ 71,  6 }, 
	{ 79,  7 }, 
	{ 87,  8 }, 
	{ 95,  9 },  
	{ 103,	10 }, 
	{ 111,	11 }, 
	{ 120,	12 },	
	{ 129,	13 }, 
	{ 138,	14 }, 
	{ 147,	15 }, //default value  
	{ 154,	15 },
	{ 162,	16 },
	{ 170,	16 },  
	{ 177,	17 },  
	{ 185,	17 },
	{ 193,	18 },
	{ 200,	18 }, 
	{ 208,	19 },
	{ 216,	20 },  
	{ 224,	22 }, 
	{ 231,	24 }, 
	{ 239,	26 },
	{ 247,	28 },  
	{ MAX_BRIGHTNESS_VALUE,  29 }, // Max pulse
};
#else
struct brt_value brt_table_ktd[] = {
		{ MIN_BRIGHTNESS_VALUE,  1 }, // Min pulse
		{ 38,  2 }, 
		{ 47,  3 }, 
		{ 55,  4 },
		{ 63,  5 }, 
		{ 71,  6 }, 
		{ 79,  7 }, 
		{ 87,  8 }, 
		{ 95,  9 },  
		{ 103,  10 }, 
		{ 111,	11 }, 
		{ 120,	12 },	
		{ 129,	13 }, 
		{ 138,	14 }, 
		{ 147,	15 }, //default value  
		{ 154,  15 },
		{ 162,	16 },
		{ 170,	16 },  
		{ 177,	17 },  
		{ 185,	17 },
		{ 193,	18 },
		{ 200,	18 }, 
		{ 208,	19 },
		{ 216,	20 },  
		{ 224,	22 }, 
		{ 231,	24 }, 
		{ 239,	26 },
		{ 247,	28 },  
		{ MAX_BRIGHTNESS_VALUE,  29 }, // Max pulse

};

#endif

#define MAX_BRT_STAGE_KTD (int)(sizeof(brt_table_ktd)/sizeof(struct brt_value))


static void lcd_backlight_control(int num)
{
	int limit;

	limit = num;

	spin_lock(&bl_ctrl_lock);
	for(;limit>0;limit--)
	{
		udelay(2);
		gpio_set_value(backlight_pin,0);
		udelay(2); 
		gpio_set_value(backlight_pin,1);		
	}
	spin_unlock(&bl_ctrl_lock);

}

/* input: intensity in percentage 0% - 100% */
static int ktd253b_backlight_update_status(struct backlight_device *bd)
{

	int user_intensity = bd->props.brightness;
    	int tune_level = 0;
	int pulse;
	int i;
	
	printk("[BACKLIGHT] ktd253b_backlight_update_status ==> user_intensity  : %d\n", user_intensity);
	//BLDBG("[BACKLIGHT] ktd253b_backlight_update_status ==> user_intensity  : %d\n", user_intensity);

	
      if(backlight_mode==BACKLIGHT_RESUME){

    		if(user_intensity > 0) {
			if(user_intensity < MIN_BRIGHTNESS_VALUE) {
				tune_level = DIMMING_VALUE; //DIMMING
			} else if (user_intensity == MAX_BRIGHTNESS_VALUE) {
				tune_level = brt_table_ktd[MAX_BRT_STAGE_KTD-1].tune_level;
			} else {
				for(i = 0; i < MAX_BRT_STAGE_KTD; i++) {
					if(user_intensity <= brt_table_ktd[i].level ) {
						tune_level = brt_table_ktd[i].tune_level;
						break;
					}
				}
			}
		}

        BLDBG("[BACKLIGHT] ktd259b_backlight_update_status ==> tune_level : %d\n", tune_level);

    if (real_level==tune_level)
    {
        return 0;
	}
    else
    {
/*	    if(real_level == 0)
	    {
	mdelay(200);
    }
*/   
	    if(tune_level<=0)
	    {
                gpio_set_value(backlight_pin,0);
                mdelay(3); 

	    }
	    else
	    {
    		if( real_level<=tune_level)
    		{
    			pulse = tune_level - real_level;
    		}
		else
		{
			pulse = 32 - (real_level - tune_level);
		}

    		//pulse = MAX_BRIGHTNESS_IN_BLU -tune_level;
            if (pulse==0)
            {
                return 0;
            }

            lcd_backlight_control(pulse); 
    }

    real_level = tune_level;
	return 0;
    }

}
       return 0;
}

static int ktd253b_backlight_get_brightness(struct backlight_device *bl)
{
	BLDBG("[BACKLIGHT] ktd253b_backlight_get_brightness\n");
    
	return current_intensity;
}

static struct backlight_ops ktd253b_backlight_ops = {
	.update_status	= ktd253b_backlight_update_status,
	.get_brightness	= ktd253b_backlight_get_brightness,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ktd253b_backlight_earlysuspend(struct early_suspend *desc)
{
    //struct ktd253b_bl_data *ktd253b = container_of(desc, struct ktd253b_bl_data, early_suspend_desc);
	//struct backlight_device *bl = platform_get_drvdata(ktd253b->pdev);
	
	backlight_mode=BACKLIGHT_SUSPEND;
	gpio_set_value(backlight_pin,0);
  mdelay(3); 
	real_level=0;
    printk("[BACKLIGHT] ktd253b_backlight_earlysuspend\n");
}

static void ktd253b_backlight_earlyresume(struct early_suspend *desc)
{
	struct ktd253b_bl_data *ktd253b = container_of(desc, struct ktd253b_bl_data, early_suspend_desc);
	struct backlight_device *bl = platform_get_drvdata(ktd253b->pdev);

	backlight_mode=BACKLIGHT_RESUME;
    printk("[BACKLIGHT] ktd253b_backlight_earlyresume\n");
    
       msleep(200); 	
    backlight_update_status(bl);
}
#else
#ifdef CONFIG_PM
static int ktd253b_backlight_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct ktd253b_bl_data *ktd253b = dev_get_drvdata(&bl->dev);
    
	printk("[BACKLIGHT] ktd253b_backlight_suspend\n");
        
	return 0;
}

static int ktd253b_backlight_resume(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);

	printk("[BACKLIGHT] ktd253b_backlight_resume\n");
        
	backlight_update_status(bl);
        
	return 0;
}
#else
#define ktd253b_backlight_suspend  NULL
#define ktd253b_backlight_resume   NULL
#endif
#endif

static int ktd253b_backlight_probe(struct platform_device *pdev)
{
	struct platform_ktd253b_backlight_data *data = pdev->dev.platform_data;
	struct backlight_device *bl;
	struct ktd253b_bl_data *ktd253b;
	struct backlight_properties props;
	
	int ret;

	printk("[BACKLIGHT] ktd253b_backlight_probe\n");
    
	if (!data) 
	{
		dev_err(&pdev->dev, "failed to find platform data\n");
		return -EINVAL;
	}

	ktd253b = kzalloc(sizeof(*ktd253b), GFP_KERNEL);
	if (!ktd253b) 
	{
		dev_err(&pdev->dev, "no memory for state\n");
		ret = -ENOMEM;
		goto err_alloc;
	}

	ktd253b->ctrl_pin = data->ctrl_pin;
    
	memset(&props, 0, sizeof(struct backlight_properties));
	props.max_brightness = data->max_brightness;
	props.type = BACKLIGHT_RAW;

	bl = backlight_device_register(pdev->name, &pdev->dev, ktd253b, &ktd253b_backlight_ops, &props);
	if (IS_ERR(bl)) 
	{
		dev_err(&pdev->dev, "failed to register backlight\n");
		ret = PTR_ERR(bl);
		goto err_bl;
	}

	gpio_request(backlight_pin,"lcd_backlight");
#ifdef CONFIG_HAS_EARLYSUSPEND
	ktd253b->pdev = pdev;
	ktd253b->early_suspend_desc.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	ktd253b->early_suspend_desc.suspend = ktd253b_backlight_earlysuspend;
	ktd253b->early_suspend_desc.resume = ktd253b_backlight_earlyresume;
	register_early_suspend(&ktd253b->early_suspend_desc);
#endif

	bl->props.max_brightness = data->max_brightness;
	bl->props.brightness = data->dft_brightness;

	platform_set_drvdata(pdev, bl);

	ktd253b_backlight_update_status(bl);
    
	return 0;

err_bl:
	kfree(ktd253b);
err_alloc:
	return ret;
}

static int ktd253b_backlight_remove(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct ktd253b_bl_data *ktd253b = dev_get_drvdata(&bl->dev);

	backlight_device_unregister(bl);


#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ktd253b->early_suspend_desc);
#endif

	kfree(ktd253b);

	gpio_free(backlight_pin);

	return 0;
}


static struct platform_driver ktd253b_backlight_driver = {
	.driver		= {
		.name	= "sprd_backlight",
		.owner	= THIS_MODULE,
	},
	.probe		= ktd253b_backlight_probe,
	.remove		= ktd253b_backlight_remove,

#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend        = ktd253b_backlight_suspend,
	.resume         = ktd253b_backlight_resume,
#endif

};

static int __init ktd253b_backlight_init(void)
{
	return platform_driver_register(&ktd253b_backlight_driver);
}
module_init(ktd253b_backlight_init);

static void __exit ktd253b_backlight_exit(void)
{
	platform_driver_unregister(&ktd253b_backlight_driver);
}
module_exit(ktd253b_backlight_exit);

MODULE_DESCRIPTION("ktd253b based Backlight Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ktd253b-backlight");
