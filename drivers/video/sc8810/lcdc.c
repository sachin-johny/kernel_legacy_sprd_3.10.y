/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
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
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/semaphore.h>
#include <mach/bits.h>
#include <mach/clock_common.h>
#include <mach/lcd.h>
#include <mach/io.h>

#include "sprd_fb.h"
#include "lcdc_reg.h"

/*
//#define  LCDC_DEBUG
#ifdef LCDC_DEBUG
#define LCDC_PRINT printk
#else
#define LCDC_PRINT(...)
#endif
*/


extern int sprdfb_debug_level;

#define LCDC_PRINT(level, args)  do { \
	if((level) <=  sprdfb_debug_level)\
        {printk args; } \
	} while (0)

#define LCD_PARTIAL_UPDATE 1
#define LCD_PANEL_NORMAL        0
#define LCD_MAIN_PNALE_SUSPEND  (1 << 1)
#define LCD_SUB_PANEL_SUSPEND   (1 << 2)
#define LCD_PANEL_DEEP_SLEEP    (1 << 3)

#define LCD_PANEL_SUSPEND(id)   ( 1 << (id+1))

typedef struct sprd_lcd_controller {
	/* only one device can work one time */
	struct sprdfb_device  *dev;
	uint32_t  init;
	uint32_t  state;

	struct sprdfb_device  *main_dev;

	/* overlay */
	uint32_t  overlay_state;

	/* ahb_clk : ? MHz */
	uint32_t  ahb_clk;
	uint32_t  lcd_id[2];

	struct clk *clk_lcdc;

	wait_queue_head_t  vsync_queue;
	uint32_t	   vsync_done;

	struct semaphore   waitlock;
} sprd_lcd_controller;

static sprd_lcd_controller lcdc;

static int32_t lcm_send_cmd (uint32_t cmd)
{
	/* wait for that the ahb enter idle state */
	while(lcdc_read(LCM_CTRL) & BIT(20));

	lcdc_write(cmd, LCM_CMD);
	return 0;
}

static int32_t lcm_send_cmd_data (uint32_t cmd, uint32_t data)
{
	/* wait for that the ahb enter idle state */
	while(lcdc_read(LCM_CTRL) & BIT(20));

	lcdc_write(cmd, LCM_CMD);

	/* wait for that the ahb enter idle state */
	while(lcdc_read(LCM_CTRL) & BIT(20));

	lcdc_write(data, LCM_DATA);
	return 0;
}

static int32_t lcm_send_data (uint32_t data)
{
	/* wait for that the ahb enter idle state */
	 while(lcdc_read(LCM_CTRL) & BIT(20));

	lcdc_write(data, LCM_DATA);
	return 0;
}

static uint32_t lcm_read_data (void)
{
	/* wait for that the ahb enter idle state */
	while(lcdc_read(LCM_CTRL) & BIT(20));
	lcdc_write(1 << 24, LCM_DATA);
	udelay(50);
	return lcdc_read(LCM_RDDATA);
}

static struct ops_mcu lcm_mcu_ops = {
	.send_cmd = lcm_send_cmd,
	.send_cmd_data = lcm_send_cmd_data,
	.send_data = lcm_send_data,
	.read_data = lcm_read_data,
};

/* panel reset */
static int32_t panel_reset(struct lcd_spec *self)
{
	lcdc_write(0x0, LCM_RSTN);
	udelay(10);
	lcdc_write(0x1, LCM_RSTN);
	msleep(5);
	return 0;
}

static uint32_t lcdc_calculate_lcm_timing(struct timing_mcu *timing)
{
	uint32_t  rcss, rlpw, rhpw, wcss, wlpw, whpw;
	uint32_t  ahb_clk = lcdc.ahb_clk;

	/************************************************
	* we assume : t = ? ns, AHB = ? MHz   so
        *      1ns  cycle  :  AHB /1000
	*      tns  cycles :  t * AHB / 1000
	*
	*****************************************/
	#define MAX_LCDC_TIMING_VALUE	15
	#define LCDC_CYCLES(ns) (( (ns) * ahb_clk + 1000 - 1)/ 1000)

	/* ceiling */
	rcss = LCDC_CYCLES(timing->rcss);
	if (rcss > MAX_LCDC_TIMING_VALUE) {
		rcss = MAX_LCDC_TIMING_VALUE ;
	}

	rlpw = LCDC_CYCLES(timing->rlpw);
	if (rlpw > MAX_LCDC_TIMING_VALUE) {
		rlpw = MAX_LCDC_TIMING_VALUE ;
	}

	rhpw = LCDC_CYCLES(timing->rhpw);
	if (rhpw > MAX_LCDC_TIMING_VALUE) {
		rhpw = MAX_LCDC_TIMING_VALUE ;
	}

	wcss = LCDC_CYCLES(timing->wcss);
	if (wcss > MAX_LCDC_TIMING_VALUE) {
		wcss = MAX_LCDC_TIMING_VALUE ;
	}

	wlpw = LCDC_CYCLES(timing->wlpw);
	if (wlpw > MAX_LCDC_TIMING_VALUE) {
		wlpw = MAX_LCDC_TIMING_VALUE ;
	}

	whpw = LCDC_CYCLES(timing->whpw) - 1;
	if (whpw > MAX_LCDC_TIMING_VALUE) {
		whpw = MAX_LCDC_TIMING_VALUE ;
	}

	return (whpw | (wlpw << 4) | (wcss << 8)
			| (rhpw << 16) |(rlpw << 20) | (rcss << 24));
}

static void lcdc_mcu_init(void)
{
	uint32_t reg_val;

	/* LCDC module enable */
	reg_val =  (1 << 0);

	/* FMARK pol */
	reg_val |= (1 << 2);

	/* dithering enable*/
	reg_val |= (1 << 4);
	lcdc_write(reg_val, LCDC_CTRL);

	/* set background*/
	lcdc_write(0, LCDC_BG_COLOR);
}

/* LCD soft reset */
static void hw_lcdc_reset(void)
{
	lcdc_set_bits(BIT(3), AHB_SOFT_RST);
	udelay(1);
	lcdc_clear_bits(BIT(3), AHB_SOFT_RST);

	lcdc_set_bits(BIT(0), LCDC_IRQ_CLR);
	lcdc_clear_bits(BIT(0), LCDC_IRQ_EN);
	lcdc_mcu_init();
}

static int set_lcdc_layer(struct sprdfb_device *dev)
{
	uint32_t reg_val = 0;

	if (dev->id == 0) {
		/* color key */
		reg_val |= (1 << 1);

		/* alpha mode select */
		reg_val |= (1 << 2);
		/* data format */
		if (dev->bpp == 32) {
			/* ABGR */
			reg_val |= (3 << 3);
			/* rb switch */
		 	reg_val |= (1 << 9);
		} else {
			/* RGB565 */
			reg_val |= (5 << 3);
			/* B2B3B0B1 */
			reg_val |= (2 << 7);
		}
		lcdc_write(reg_val, LCDC_OSD1_CTRL);
		/* OSD1 layer size */
		reg_val = ((dev->width & 0xfff) | (dev->height << 16));
		lcdc_write(reg_val, LCDC_OSD1_SIZE_XY);

		/* OSD1 layer pitch */
		lcdc_write((dev->width & 0xfff), LCDC_OSD1_PITCH);
		/* OSD1 color_key value */
		lcdc_write(0, LCDC_OSD1_CK);

		/* OSD1 layer alpha value */
		lcdc_write(0xFF, LCDC_OSD1_ALPHA);
	} else {

		/* color key */
		reg_val |= (1 << 1);

		/* alpha mode select */
		reg_val |= (1 << 2);
		/* data format */
		if (dev->bpp == 32) {
			/* ABGR */
			reg_val |= (3 << 3);
			/* rb switch */
		 	reg_val |= (1 << 9);
		} else {
			/* RGB565 */
			reg_val |= (5 << 3);
			/* B2B3B0B1 */
			reg_val |= (2 << 7);
		}
		lcdc_write(reg_val, LCDC_OSD2_CTRL);
		/* OSD2 layer size */
		reg_val = ((dev->width & 0xfff) | (dev->height << 16));
		lcdc_write(reg_val, LCDC_OSD2_SIZE_XY);

		/* OSD2 layer pitch */
		lcdc_write((dev->width & 0xfff), LCDC_OSD2_PITCH);
		/* OSD2 color_key value */
		lcdc_write(0, LCDC_OSD2_CK);

		/* OSD2 layer alpha value */
		lcdc_write(0xFF, LCDC_OSD2_ALPHA);
	}
	return 0;
}

static uint32_t lcdc_lcm_configure(struct sprdfb_device *dev)
{
	struct lcd_spec *panel = dev->panel;
	uint32_t reg_val = 0;

	if (dev->id == 0) {
		/* CS0 bus mode [BIT0]: 8080/6800 */
		switch (panel->info.mcu->bus_mode) {
		case LCD_BUS_8080:

			break;
		case LCD_BUS_6800:
			reg_val  |= 1;
			break;
		default:
			break;
		}
		/* CS0 bus width [BIT2:1] */
		switch (panel->info.mcu->bus_width) {
		case 8:
			break;
		case 9:
			reg_val  |= ((1 << 1) | (1 << 4));
			break;
		case 16:
			reg_val  |= (2 << 1);
			break;
		case 18:
			reg_val  |= ((3 << 1) | (1 << 4));
			break;
		case 24:
			reg_val  |= ((4 << 1) | (2 << 4));
			break;
		default:
			reg_val  |= (2 << 1);
			break;
		}
	} else {
		/* CS1 bus mode [BIT8]: 8080/6800 */
		switch (panel->info.mcu->bus_mode) {
		case LCD_BUS_8080:

			break;
		case LCD_BUS_6800:
			reg_val  |= (1 << 8);
			break;
		default:
			break;
		}
		/* CS1 bus width [BIT10:9] */
		switch (panel->info.mcu->bus_width) {
		case 8:
			break;
		case 9:
			reg_val  |= ((1 << 9) | (1 << 12));
			break;
		case 16:
			reg_val  |= (2 << 9);
			break;
		case 18:
			reg_val  |= ((3 << 9) | (1 << 12));
			break;
		case 24:
			reg_val  |= ((4 << 9) | (2 << 12));
			break;
		default:
			reg_val  |= (2 << 9);
			break;

		}
		reg_val  |= (1 << 16);
	}

	return reg_val;
}

static void lcdc_update_lcm_path0(struct sprdfb_device *dev)
{
	lcdc_write(dev->reserved[0],LCM_CTRL);
	lcdc_write(dev->reserved[1],LCM_TIMING0);
}

static void lcdc_update_lcm_path1(struct sprdfb_device *dev)
{
	lcdc_write(dev->reserved[0],LCM_CTRL);
	lcdc_write(dev->reserved[1],LCM_TIMING1);
}

typedef void (*update_lcm_func)(struct sprdfb_device *dev);
update_lcm_func update_lcm[] = {
	lcdc_update_lcm_path0,
	lcdc_update_lcm_path1,
};

void hw_init(struct sprdfb_device *dev)
{
	/* only MCU mode is supported currently */
	if (LCD_MODE_RGB == dev->panel->mode)
		return;

	/* panel reset */
	if (dev->need_reinit) {
		panel_reset(NULL);
	}

	/* set lcdc-lcd interface parameters */
	dev->reserved[0] = lcdc_lcm_configure(dev);
	dev->update_lcm = update_lcm[dev->id];

	/* set timing parameters for LCD */
	dev->reserved[1] = dev->timing[0];
	dev->update_lcm(dev);
}

void hw_later_init(struct sprdfb_device *dev)
{
	/* init mounted lcd panel */
	if(dev->need_reinit) {
	    dev->panel->ops->lcd_init(dev->panel);
	}

	set_lcdc_layer(dev);
	lcdc_set_bits(BIT(0), LCDC_IRQ_EN);
}

irqreturn_t lcdc_isr(int irq, void *data)
{
	uint32_t val ;
        sprd_lcd_controller *lcdc = (sprd_lcd_controller *)data;
	struct sprdfb_device *dev = lcdc->dev;

	val = lcdc_read(LCDC_IRQ_STATUS);

	if (val & 1) { /* lcdc done isr */
		lcdc_write(1, LCDC_IRQ_CLR);

		lcdc->vsync_done = 1;
		if (dev->vsync_waiter) {
			dev->vsync_waiter = 0;
			wake_up_interruptible(&(lcdc->vsync_queue));
		}
	}

	return IRQ_HANDLED;
}

static int mount_panel(struct sprdfb_device *dev, struct lcd_spec *panel)
{
	dev->panel = panel;

	panel->info.mcu->ops = dev->ops;

	panel->ops->lcd_reset = panel_reset;

	dev->width = panel->width;
	dev->height = panel->height;

	dev->bpp = panel->bpp = 32;

	{
		struct timing_mcu *timing = panel->info.mcu->timing;
		dev->timing[0] = lcdc_calculate_lcm_timing(timing);

		if (!(dev->panel->cap & LCD_CAP_UNIQUE_TIMING)) {
			timing++;
			dev->timing[1] = lcdc_calculate_lcm_timing(timing);
		}
	}

#ifdef LCD_PARTIAL_UPDATE
	if (!(panel->cap & LCD_CAP_NOT_PARTIAL_UPDATE) &&
			panel->ops->lcd_invalidate_rect != NULL) {
		dev->fb->fix.reserved[0] = 0x6f76;
		dev->fb->fix.reserved[1] = 0x6572;
	}
#endif
	if (panel->cap & LCD_CAP_NOT_TEAR_SYNC) {
		dev->run = lcdc_read(LCDC_CTRL) | BIT(1) | BIT(3);
	} else {
		dev->run = lcdc_read(LCDC_CTRL) | BIT(2) | BIT(3);
	}
	printk("mount panel %d, %d, %d!!\n",dev->id, dev->width, dev->height);
	return 0;
}

static int32_t find_adapt_from_uboot(uint32_t device_id,
				struct  sprd_lcd_platform_data* platform_data)
{
	int32_t i;
	struct lcd_panel_cfg  *lcd_panel = platform_data->lcd_panel_ptr;
	uint32_t lcd_panel_size = platform_data->lcd_panel_size;

	for(i = 0;i < lcd_panel_size; i++) {
		if(device_id == lcd_panel[i].lcd_id) {
			return i;
		}
	}

	printk(KERN_ERR " sprdfb can not get lcd id from uboot!!\n");
	return -1;
}

static uint32_t lcd_readid_default(struct lcd_spec *self)
{
	uint32_t id = 0;

	/* default id reg is 0 */
	self->info.mcu->ops->send_cmd(0x0);

	if(self->info.mcu->bus_width == 8) {
		id = (self->info.mcu->ops->read_data())&0xff;
		id <<= 8;
		id |= (self->info.mcu->ops->read_data())&0xff;
	} else {
		id = self->info.mcu->ops->read_data();
	}

	return id;
}

static int32_t find_adapt_from_readid(struct sprdfb_device *dev,struct  sprd_lcd_platform_data*platform_data)
{
	int32_t i;
	uint32_t id;
	struct lcd_panel_cfg  *lcd_panel = platform_data->lcd_panel_ptr;
	uint32_t lcd_panel_size = platform_data->lcd_panel_size;

	for(i = 0; i < lcd_panel_size; i++) {
		/* first ,try mount */
		mount_panel(dev,lcd_panel[i].panel);
		/* hw init to every panel */
		hw_init(dev);
		/* readid */
		if(dev->panel->ops->lcd_readid) {
			id = dev->panel->ops->lcd_readid(dev->panel);
		} else {
			id = lcd_readid_default(dev->panel);
		}
		//if the id is right?
		if(id == lcd_panel[i].lcd_id) {
			LCDC_PRINT (2, ("sc8810fb : this lcd id is %d\n", id));
			return i;
		}
	}
	printk(KERN_ERR "sc8810fb may be wrong , can't get lcd id %x!!\n", id);
	return -1;
}

static int sprd_lcdc_sync(void)
{
	int ret = wait_event_interruptible_timeout(lcdc.vsync_queue,
			          lcdc.vsync_done, msecs_to_jiffies(100));
	if (!ret) { /* time out */
		printk(KERN_ERR "lcdc: sprd_lcdc_sync time out!!!!!\n");
		lcdc.vsync_done = 1;
		return -1;
	}
	return 0;
}

int lcdc_early_init(struct sprd_lcd_platform_data* platform_data,struct sprdfb_device *dev)
{
	struct lcd_panel_cfg* lcd_panel = platform_data->lcd_panel_ptr;
	int lcd_adapt = 0;

	dev->ops = &lcm_mcu_ops;

	lcd_adapt = find_adapt_from_uboot(dev->device_id, platform_data);
	LCDC_PRINT(2, ("sprdfb :lcd_adapt %d, %d\n", lcd_adapt, dev->device_id));
	if (lcd_adapt == -1) {
		/* can not reach here; we get device id from u-boot */
		dev->need_reinit = 1;
		lcd_adapt = find_adapt_from_readid(dev,platform_data);
	}

	if (lcd_adapt < 0) { /* invalid index */
		printk(KERN_ERR " can not read device id, and we will not refresh!\n");
		dev->fb_state = FB_NO_REFRESH;
		lcd_adapt = 0;
	}

	mount_panel(dev, lcd_panel[lcd_adapt].panel);

	return 0;
}

int lcdc_init(struct sprdfb_device *dev)
{
	hw_init(dev);
	hw_later_init(dev);

	/* we assume that the first fb is the main lcd */
	if (lcdc.main_dev == 0) {
		lcdc.main_dev = dev;
	}
	return 0;
}

static void lcdc_enable_device (int id)
{
	if (id == 0) {
		lcdc_set_bits(BIT(0),LCDC_OSD1_CTRL);
	} else {
		lcdc_set_bits(BIT(0),LCDC_OSD2_CTRL);
	}

}

static void lcdc_disable_device (int id)
{
	if (id == 0) {
		lcdc_clear_bits(BIT(0),LCDC_OSD1_CTRL);
	} else {
		lcdc_clear_bits(BIT(0),LCDC_OSD2_CTRL);
	}

}

int lcdc_open(struct sprdfb_device *dev)
{
	if (dev->open == 1) {
		down(&lcdc.waitlock);
		lcdc_enable_device(dev->id);
		up(&lcdc.waitlock);
	}
	return 0;
}

int lcdc_close(struct sprdfb_device *dev)
{
	if (dev->open == 0) {
		/* disable the osd layer */
		down(&lcdc.waitlock);
		lcdc_disable_device (dev->id);
		up(&lcdc.waitlock);
	}
	return 0;
}

int lcdc_sync(void)
{
	int ret;
	down(&lcdc.waitlock);
	ret = sprd_lcdc_sync();
	up(&lcdc.waitlock);
	return ret;
}

static void real_set_layer_base(struct sprdfb_device *dev)
{
	struct fb_info *fb = dev->fb;

	uint32_t reg_val = (fb->var.yoffset == 0)?fb->fix.smem_start:
		              (fb->fix.smem_start + fb->fix.smem_len/2);

	if (dev->id == 0) {
		lcdc_write(reg_val, LCDC_OSD1_BASE_ADDR);
	} else {
		lcdc_write(reg_val, LCDC_OSD2_BASE_ADDR);
	}
}

#ifdef LCD_PARTIAL_UPDATE
static int lcdc_update_rect(struct fb_info *fb)
{
	uint32_t reg_val = (fb->var.yres << 16) | (fb->var.xres);

	lcdc_write(fb->var.reserved[1], LCDC_LCM_START);

	lcdc_write(fb->var.reserved[2] , LCDC_LCM_SIZE);

	lcdc_write(reg_val , LCDC_DISP_SIZE);
	return 0;
}
#endif

static inline int lcdc_update_whole(struct fb_info *fb)
{
	uint32_t reg_val = (fb->var.yres << 16) | (fb->var.xres);

	lcdc_write(0, LCDC_LCM_START);

	lcdc_write(reg_val , LCDC_LCM_SIZE);
	lcdc_write(reg_val , LCDC_DISP_SIZE);
	return 0;
}

static inline void lcdc_start(uint32_t run)
{
	lcdc_set_bits(run, LCDC_CTRL);     /* start refresh */
}

static int do_lcdc_refresh(struct sprdfb_device *dev)
{
	struct fb_info *fb = dev->fb;

	LCDC_PRINT(3, ("lcdc: [%s]\n", __FUNCTION__));

	if (sprd_lcdc_sync() != 0) {
		printk(KERN_ERR "sprdfb can not do pan_display !!!!\n");
		return 0;
	}

	if (dev->device_id != 0) {
		lcdc.dev = dev;

		real_set_layer_base(dev);

		if (!(dev->panel->cap & LCD_CAP_MANUAL_REFRESH)) {
			lcdc.vsync_done = 0;
			dev->vsync_waiter ++;
		}
		dev->reserved[1] = dev->timing[0];
		dev->update_lcm(dev);

	#ifdef LCD_PARTIAL_UPDATE
		if (fb->var.reserved[0] == 0x6f766572) {
			uint16_t left, top, width, height;

			left   = fb->var.reserved[1] &  0xffff;
			top    = fb->var.reserved[1] >> 16;
			width  = fb->var.reserved[2] &  0xffff;
			height = fb->var.reserved[2] >> 16;
			lcdc_update_rect(fb);

			dev->panel->ops->lcd_invalidate_rect(dev->panel,
						left, top, left+width-1, top+height-1);

			if (!(dev->panel->cap & LCD_CAP_MANUAL_REFRESH)) {
				if (!(dev->panel->cap & LCD_CAP_UNIQUE_TIMING)) {
					dev->reserved[1] = dev->timing[1];
					dev->update_lcm(dev);
				}
				lcdc_start(dev->run);
			}
		} else
	#endif
		{
			lcdc_update_whole(fb);
			dev->panel->ops->lcd_invalidate(dev->panel);
			if (!(dev->panel->cap & LCD_CAP_MANUAL_REFRESH)) {
				if (!(dev->panel->cap & LCD_CAP_UNIQUE_TIMING)) {
					dev->reserved[1] = dev->timing[1];
					dev->update_lcm(dev);
				}
				if (!(dev->panel->cap & LCD_CAP_MANUAL_REFRESH)) {
					lcdc_start(dev->run);
				}
			}
		}
	} else {
		struct sprdfb_device *real_dev= lcdc.dev;

		real_set_layer_base(dev);
		if (real_dev && (real_dev->fb_state == FB_NORMAL)) {
			lcdc_update_whole(real_dev->fb);
			real_dev->panel->ops->lcd_invalidate(real_dev->panel);
			if (!(real_dev->panel->cap & LCD_CAP_MANUAL_REFRESH)) {
				if (!(real_dev->panel->cap & LCD_CAP_UNIQUE_TIMING)) {
					real_dev->reserved[1] = real_dev->timing[1];
					real_dev->update_lcm(real_dev);
				}
				lcdc_start(real_dev->run);
			}
		} else {
			printk(KERN_ERR "sprdfb simulator can not do pan_display !!!!\n");
		}
	}

	LCDC_PRINT(3, ("[%s] LCDC_CTRL: 0x%x\n", __FUNCTION__, lcdc_read(LCDC_CTRL)));
	LCDC_PRINT(4, ("[%s] LCDC_DISP_SIZE: 0x%x\n", __FUNCTION__, lcdc_read(LCDC_DISP_SIZE)));
	LCDC_PRINT(4, ("[%s] LCDC_LCM_START: 0x%x\n", __FUNCTION__, lcdc_read(LCDC_LCM_START)));
	LCDC_PRINT(4, ("[%s] LCDC_LCM_SIZE: 0x%x\n", __FUNCTION__, lcdc_read(LCDC_LCM_SIZE)));
	LCDC_PRINT(4,("[%s] LCDC_BG_COLOR: 0x%x\n", __FUNCTION__, lcdc_read(LCDC_BG_COLOR)));

	LCDC_PRINT(4,("[%s] LCM_CTRL: 0x%x\n", __FUNCTION__, lcdc_read(LCM_CTRL)));
	LCDC_PRINT(4,("[%s] LCM_TIMING0: 0x%x\n", __FUNCTION__, lcdc_read(LCM_TIMING0)));
	LCDC_PRINT(4, ("[%s] LCM_TIMING1: 0x%x\n", __FUNCTION__, lcdc_read(LCM_TIMING1)));

	return 0;
}

int lcdc_refresh(struct sprdfb_device *dev)
{
	int ret;
	down(&lcdc.waitlock);
	ret = do_lcdc_refresh(dev);
	up(&lcdc.waitlock);
	return ret;
}

#if 0
int lcdc_ioctl(struct sprdfb_device *dev, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int ret;

	switch (cmd) {
	case SPRDFB_WAITFORVSYNC:
		down(&lcdc.waitlock);
		sprd_lcdc_sync();
		up(&lcdc.waitlock);
		break;
	case SPRDFB_GET_UPDATE_MODE:
		//mdp->set_grp_disp(mdp, arg);
		break;
	case SPRDFB_SET_UPDATE_MODE:

		break;
	case SPRDFB_CHANGE_STATE:
		//mdp->set_grp_disp(mdp, arg);
		break;
	case SPRDFB_SET_COLOR_KEY:

		break;
	case SPRDFB_GET_COLOR_KEY:

		break;
	case SPRDPFB_UPDATE_WINDOW:

		break;
	default:
		printk(KERN_INFO "sprdb lcdc unknown ioctl: %d\n", cmd);
		return -EINVAL;
	}
	return ret;
}
#endif

int lcdc_suspend(struct sprdfb_device *dev)
{
	down(&lcdc.waitlock);

	//printk("lcdc_suspend %d, %d!\n", dev->device_id, lcdc.state);

	if (dev->device_id != 0) {
		if (!(dev->panel->cap & LCD_CAP_MANUAL_REFRESH)
			 && sprd_lcdc_sync() != 0) {
			up(&lcdc.waitlock);
			printk(KERN_ERR "sprdfb can not do pan_display !!!!\n");
			return 0;
		}

		if (dev->panel->ops->lcd_enter_sleep != NULL) {
			dev->reserved[1] = dev->timing[0];
			dev->update_lcm(dev);
			dev->panel->ops->lcd_enter_sleep(dev->panel,1);
		}
	}

#ifdef CONFIG_FB_DUAL_DISPLAY
	lcdc.state |= LCD_PANEL_SUSPEND(dev->id);
	if ((lcdc.state & LCD_MAIN_PNALE_SUSPEND)
			&& (lcdc.state & LCD_SUB_PANEL_SUSPEND) ){
		printk("clk_disable\n");
		clk_disable(lcdc.clk_lcdc);
	}
#else
	clk_disable(lcdc.clk_lcdc);
#endif
	up(&lcdc.waitlock);
	return 0;
}

int lcdc_resume(struct sprdfb_device *dev)
{
	down(&lcdc.waitlock);

	LCDC_PRINT(2, ("lcdc_suspend %d, %d!\n", dev->device_id, lcdc.state));

	if (lcdc.clk_lcdc->usecount == 0) {
		clk_enable(lcdc.clk_lcdc);
	}

	if (lcdc_read(LCDC_CTRL) == 0) {
		lcdc.state |= LCD_PANEL_DEEP_SLEEP;
		hw_lcdc_reset();
		panel_reset(NULL);
	}

	if (lcdc.state & LCD_PANEL_DEEP_SLEEP) {
		LCDC_PRINT(2, ("sprdfb resume from deep sleep \n"));
		if (dev->device_id != 0) {
			hw_init(dev);
			dev->reserved[1] = dev->timing[0];
			dev->update_lcm(dev);
	    		dev->panel->ops->lcd_init(dev->panel);			
			hw_later_init(dev);
			
		}
	} else {
		LCDC_PRINT(2, ("sprdfb resume from sleep \n"));
		if(dev->device_id != 0 ) {
			dev->reserved[1] = dev->timing[0];
			dev->update_lcm(dev);
			dev->panel->ops->lcd_enter_sleep(dev->panel,0);
		}
	}

	if (dev->open > 0) {
		lcdc_enable_device(dev->id);
	}

#ifdef CONFIG_FB_DUAL_DISPLAY
	lcdc.state &= (~LCD_PANEL_SUSPEND(dev->id));
	if ((lcdc.state & (LCD_MAIN_PNALE_SUSPEND | LCD_SUB_PANEL_SUSPEND)) == 0) {
		lcdc.state = LCD_PANEL_NORMAL;
	}
#else
	lcdc.state = LCD_PANEL_NORMAL;
#endif

	lcdc.vsync_done = 1;
	up(&lcdc.waitlock);
	return 0;
}

int lcdc_hardware_init(void)
{
	int ret;
	struct clk * clk;

	memset(&lcdc, 0, sizeof(lcdc));

	/* get current clk_lcdc */
	clk = clk_get(NULL,"clk_lcdc");
	if (clk == NULL) {
		panic("can not get clk_lcdc!!!!!\n");
	}
	clk_enable(clk);
	lcdc.clk_lcdc = clk;

	/* get current clk_ahb rate : ? MHz */
	clk = clk_get(NULL,"clk_ahb");
	lcdc.ahb_clk = clk_get_rate(clk) / 1000000;

	LCDC_PRINT(2, ("[%s] ahb_clk: 0x%x MHz\n", __FUNCTION__, lcdc.ahb_clk));

	lcdc.state = LCD_PANEL_NORMAL;

	sema_init(&lcdc.waitlock, 1);

	lcdc.vsync_done = 1;
	init_waitqueue_head(&(lcdc.vsync_queue));

	/* register isr */
	ret = request_irq(IRQ_LCDC_INT, lcdc_isr, IRQF_DISABLED, "LCDC", &lcdc);
	if (ret) {
		printk(KERN_ERR "lcdc: failed to request irq!\n");
		return -ENOMEM;
	}

	lcdc_mcu_init();

	printk("lcdc_hardware_init \n");
	return 0;
}

int lcdc_hardware_close(void)
{
	if (lcdc.clk_lcdc->usecount > 0) {
		clk_disable(lcdc.clk_lcdc);
	}
	printk("lcdc_close \n");
	return 0;
}


#include "mach/overlay.h"

int overlay_open(void)
{
	down(&lcdc.waitlock);
	lcdc.overlay_state = 1;
	up(&lcdc.waitlock);
}
EXPORT_SYMBOL(overlay_open);

int overlay_configure(int type, overlay_rect *rect, unsigned char *buffer)
{
	uint32_t reg_value;

	if (type >= SPRD_DATA_TYPE_LIMIT) {
		return -1;
	}
	lcdc_write(((type << 3) | (1 << 0)), LCDC_IMG_CTRL);
	lcdc_write((uint32_t)buffer, LCDC_IMG_Y_BASE_ADDR);
	if (type < SPRD_DATA_TYPE_RGB888) {
		uint32_t size = rect->w * rect->h;
		lcdc_write((uint32_t)(buffer + size), LCDC_IMG_Y_BASE_ADDR);
	}

	reg_value = (rect->h << 16) | (rect->w);
	lcdc_write(reg_value, LCDC_IMG_SIZE_XY);

	lcdc_write(rect->w, LCDC_IMG_PITCH);

	reg_value = (rect->y << 16) | (rect->x);
	lcdc_write(reg_value, LCDC_IMG_DISP_XY);
	return 0;
}

static int overlay_update_rect(overlay_rect *rect)
{
	uint32_t reg_value;
	reg_value = (rect->y << 16) | (rect->x);
	lcdc_write(reg_value, LCDC_LCM_START);

	reg_value = (rect->h << 16) | (rect->w);
	lcdc_write(reg_value , LCDC_LCM_SIZE);
	return 0;
}

/* we assume that the main lcd is exist and can invalidate from lcdc */
static int overlay_do_update(overlay_rect *rect)
{
	struct sprdfb_device *dev = lcdc.main_dev;
	struct fb_info *fb = dev->fb;

	if (dev == 0 || dev->fb_state != FB_NORMAL) {
		printk(KERN_ERR "overlay can not invalidate !!!!\n");
		return -1;
	}

#ifdef LCD_PARTIAL_UPDATE
	if (fb->var.reserved[0] == 0x6f766572) {
		overlay_update_rect(rect);
		dev->panel->ops->lcd_invalidate_rect(dev->panel,
				rect->x, rect->y,
				rect->x + rect->w - 1, rect->y + rect->h - 1);
		if (!(dev->panel->cap & LCD_CAP_UNIQUE_TIMING)) {
			dev->reserved[1] = dev->timing[1];
			dev->update_lcm(dev);
		}
		lcdc_start(dev->run);
	} else
#endif
	{
		lcdc_update_whole(dev->fb);
		dev->panel->ops->lcd_invalidate(dev->panel);

		if (!(dev->panel->cap & LCD_CAP_UNIQUE_TIMING)) {
			dev->reserved[1] = dev->timing[1];
			dev->update_lcm(dev);
		}
		lcdc_start(dev->run);
	}
}

int overlay_update(int type, overlay_rect *rect, unsigned char *buffer)
{
	int ret;
	down(&lcdc.waitlock);
	if (lcdc.overlay_state != 1 || sprd_lcdc_sync() != 0) {
		up(&lcdc.waitlock);
		printk(KERN_ERR "sprdfb can not do pan_display !!!!\n");
		return 0;
	}
	overlay_configure(type, rect, buffer);
	ret = overlay_do_update(rect);
	up(&lcdc.waitlock);
	return ret;
}

EXPORT_SYMBOL(overlay_update);

int overlay_close(void)
{
	down(&lcdc.waitlock);
	if (lcdc.overlay_state != 1 || sprd_lcdc_sync() != 0) {
		up(&lcdc.waitlock);
		printk(KERN_ERR "overlay_close error!\n");
		return 0;
	}
	lcdc_set_bits(BIT(0), LCDC_IMG_CTRL);	/* disable the image layer */
	lcdc.overlay_state = 0;
	up(&lcdc.waitlock);
}

EXPORT_SYMBOL(overlay_close);


