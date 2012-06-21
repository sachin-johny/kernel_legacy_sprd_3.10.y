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

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/fb.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/io.h>
#include <linux/workqueue.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <mach/bits.h>
#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/clock_common.h>
#include "sprd_fb.h"
#include "lcdc_reg.h"

#if CONFIG_CPU_FREQ
#include <linux/cpufreq.h>
#include <linux/notifier.h>
#endif

/*
//#define  FB_DEBUG
#ifdef FB_DEBUG
#define FB_PRINT printk
#else
#define FB_PRINT(...)
#endif
*/

/* Module parameter to control log level */
int sprdfb_debug_level = 1;
module_param(sprdfb_debug_level, int, S_IRUSR | S_IWUSR | S_IWGRP | S_IRGRP | S_IROTH); /* rw-rw-r-- */
MODULE_PARM_DESC(sprdfb_debug_level, "Higher number, more dmesg output");


#define FB_PRINT(level, args)  do { \
	if((level) <=  sprdfb_debug_level)\
        {printk args; } \
	} while (0)

extern int lcdc_early_init(struct sprd_lcd_platform_data* platform_data,
				struct sprdfb_device *dev);
extern int lcdc_init(struct sprdfb_device *dev);
extern int lcdc_open(struct sprdfb_device *dev);
extern int lcdc_close(struct sprdfb_device *dev);
extern int lcdc_sync(void);
extern int lcdc_refresh(struct sprdfb_device *dev);
extern int lcdc_suspend(struct sprdfb_device *dev);
extern int lcdc_resume(struct sprdfb_device *dev);
//extern int lcdc_ioctl(struct sprdfb_device *dev, unsigned int cmd, unsigned long arg);
extern int lcdc_hardware_init(void);
extern int lcdc_hardware_close(void);

/* Panel device id from u-boot */
static uint32_t panel_id[2];


static int sprdfb_open(struct fb_info *info, int user)
{
	struct sprdfb_device *dev = info->par;

	if (user) {
		dev->open++;
		lcdc_open(dev);
	}
	printk("sprdfb_open fb%d count: %d\n", dev->id, dev->open);
	return 0;
}

static int sprdfb_release(struct fb_info *info, int user)
{
	struct sprdfb_device *dev = info->par;

	if (user) {
		dev->open--;
		/* msleep(1); */
		lcdc_close(dev);

	}
	printk ("sprdfb_release fb%d count:%d\n", dev->id, dev->open);
	return 0;
}

static int sprdfb_check_var(struct fb_var_screeninfo *var, struct fb_info *fb)
{
	if ((var->xres != fb->var.xres) ||
		(var->yres != fb->var.yres) ||
		(var->xres_virtual != fb->var.xres_virtual) ||
		(var->yres_virtual != fb->var.yres_virtual) ||
		(var->xoffset != fb->var.xoffset) ||
		(var->bits_per_pixel != fb->var.bits_per_pixel) ||
		(var->grayscale != fb->var.grayscale))
			return -EINVAL;
	return 0;
}

static int sprdfb_pan_display(struct fb_var_screeninfo *var, struct fb_info *fb)
{
	struct sprdfb_device *dev = fb->par;

	FB_PRINT(3, ("lcdc: [%s]\n", __FUNCTION__));

	if (dev->fb_state != FB_NORMAL) {
		printk(KERN_ERR "sprdfb can not do pan_display!!\n");
		return 0;
	}
	lcdc_refresh(dev);

	return 0;
}

static int sprdfb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
#if 0
	struct sprdfb_device *dev = info->par;
	return lcdc_ioctl(dev, cmd, arg);
#endif
	return 0;
}

static struct fb_ops sprdfb_ops = {
	.owner = THIS_MODULE,
	.fb_open = sprdfb_open,
	.fb_release = sprdfb_release,
	.fb_check_var = sprdfb_check_var,
	.fb_pan_display = sprdfb_pan_display,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
	.fb_ioctl = sprdfb_ioctl,
};

#if CONFIG_CPU_FREQ
/*
*  @nb, structure "notifier_ block" defined in your drivers
*  @val,CPUFREQ_PRECHANGE or CPUFREQ_POSTCHANGE
*  @data, pointer which point to  strcuture "cpufreq_freqs"
*/
static int
sprdfb_freq_transition(struct notifier_block *nb, unsigned long val, void *data)
{
	struct cpufreq_freqs *f = data;
	struct sprdfb_device *dev = container_of(nb, struct sprdfb_device, freq_transition);

	switch(val){
	case CPUFREQ_PRECHANGE:
		dev->fb_state = FB_NO_REFRESH;
		lcdc_sync();
		FB_PRINT(2, ("lcdfb cpufreq notify: CPUFREQ_PRECHANGE\n"));
	        FB_PRINT(2, ("lcdfb cpufreq notify: old_freq:%u, new_freq:%u\n", f->old, f->new));
		break;
	case CPUFREQ_POSTCHANGE:
		dev->fb_state = FB_NORMAL;
		FB_PRINT(2, ("lcdfb cpufreq notify: CPUFREQ_POSTCHANGE\n"));
		FB_PRINT(2, ("lcdfb cpufreq notify: old_freq:%u, new_freq:%u\n", f->old, f->new));
		break;
	}
	return 0;
}

/*
*  @nb, structure "notifier_ block" defined in your drivers
*  @val,CPUFREQ_ADJUST, CPUFREQ_INCOMPATIBLE, CPUFREQ_NOTIFY
*  @data, pointer which point to  strcuture "cpufreq_policy"
*/
static int
sprdfb_freq_policy(struct notifier_block *nb, unsigned long val,
			 void *data)
{
	struct cpufreq_policy *policy = data;

	switch (val) {
	case CPUFREQ_ADJUST:
		FB_PRINT(2, ("lcdfb cpufreq nofity: CPUFREQ_ADJUST\n"));
		break;
	case CPUFREQ_INCOMPATIBLE:
		FB_PRINT(2, ("lcdfb cpufreq nofity: CPUFREQ_INCOMPATIBLE\n"));
		break;
	case CPUFREQ_NOTIFY:
		FB_PRINT(2, ("lcdfb cpufreq nofity: CPUFREQ_NOTIFY\n"));
		break;
	}
	return 0;
}
#endif


static int setup_fbmem(struct sprdfb_device *dev, struct platform_device *pdev)
{
	char *addr;

	struct lcd_spec *panel= dev->panel;

	panel->mem_len = panel->width * panel->height * (dev->bpp / 8) * 2;

	addr = (char*)alloc_pages_exact(panel->mem_len, GFP_KERNEL|__GFP_ZERO);
	if(!addr) {
		panic("sprdfb setup_fbmem error!\n");
	}
	panel->screen_base = addr;
	dev->fb->fix.smem_start = __pa(addr);
	dev->fb->fix.smem_len = panel->mem_len;
	dev->fb->screen_base  = addr;

	printk("sprdfb->fb->fix.smem_start=0x%x\n",(uint32_t)dev->fb->fix.smem_start);

	return 0;
}

static unsigned PP[16];
static void setup_fb_info(struct sprdfb_device *dev)
{
	struct fb_info *fb = dev->fb;
	int ret, i;

	fb->fbops = &sprdfb_ops;
	fb->flags = FBINFO_DEFAULT;

	/* finish setting up the fb_info struct */
	strncpy(fb->fix.id, "sprdfb", 16);
	fb->fix.ypanstep = 1;
	fb->fix.type = FB_TYPE_PACKED_PIXELS;
	fb->fix.visual = FB_VISUAL_TRUECOLOR;
	fb->fix.line_length = dev->width * dev->bpp/ 8;

	fb->var.xres = dev->width;
	fb->var.yres = dev->height;
	fb->var.width = dev->width;
	fb->var.height = dev->height;
	fb->var.xres_virtual = dev->width;
	fb->var.yres_virtual = dev->height * 2;
	fb->var.bits_per_pixel = dev->bpp;

	/* fake pixel clock to avoid divide 0 */
	fb->var.pixclock = 45000;
	fb->var.accel_flags = 0;

	fb->var.yoffset = 0;

	/* only support two pixel format */
	if (dev->bpp == 32) {
		fb->var.red.offset     = 24;
		fb->var.red.length     = 8;
		fb->var.red.msb_right  = 0;
		fb->var.green.offset   = 16;
		fb->var.green.length   = 8;
		fb->var.green.msb_right = 0;
		fb->var.blue.offset    = 8;
		fb->var.blue.length    = 8;
		fb->var.blue.msb_right = 0;
	} else {
		fb->var.red.offset     = 11;
		fb->var.red.length     = 5;
		fb->var.red.msb_right  = 0;
		fb->var.green.offset   = 5;
		fb->var.green.length   = 6;
		fb->var.green.msb_right = 0;
		fb->var.blue.offset    = 0;
		fb->var.blue.length    = 5;
		fb->var.blue.msb_right = 0;
	}
	ret = fb_alloc_cmap(&fb->cmap, 16, 0);
	if (ret) {
		panic("fb_alloc_cmap error!\n");
	}
	fb->pseudo_palette = PP;

	PP[0] = 0;
	for ( i = 1; i < 16; i++)
		PP[i] = 0xffffffff;
}

static void fb_free_resources(struct sprdfb_device *dev)
{
	if (dev == NULL) {
		return;
	}

	if (dev->panel != NULL && dev->panel->ops->lcd_close != NULL) {
		dev->panel->ops->lcd_close(dev->panel);
	}

	lcdc_close(dev);

	if (&dev->fb->cmap != NULL) {
		fb_dealloc_cmap(&dev->fb->cmap);
	}
	if (dev->fb->screen_base) {
		free_pages_exact ((void *)dev->fb->screen_base,
				(dev->fb->fix.smem_len));
	}
	unregister_framebuffer(dev->fb);
	framebuffer_release(dev->fb);
}


#ifdef CONFIG_HAS_EARLYSUSPEND
static void sprdfb_early_suspend (struct early_suspend* es)
{
	struct sprdfb_device *dev = container_of(es, struct sprdfb_device, early_suspend);

	dev->fb_state = FB_NO_REFRESH;
	lcdc_suspend(dev);
	
	FB_PRINT(2, ("lcdc: [%s]\n", __FUNCTION__));
}

static void sprdfb_late_resume (struct early_suspend* es)
{
	struct sprdfb_device *dev = container_of(es, struct sprdfb_device, early_suspend);
	
	lcdc_resume(dev);
	dev->fb_state = FB_NORMAL;

	FB_PRINT(2, ("lcdc: [%s]\n", __FUNCTION__));
}
#endif

static int sprdfb_probe(struct platform_device *pdev)
{
	struct fb_info *fb;
	struct sprdfb_device *dev;
	struct sprd_lcd_platform_data* platform_data = pdev->dev.platform_data;

	int32_t ret;

	printk("sprdfb initialize!\n");

	fb = framebuffer_alloc(sizeof(struct sprdfb_device), &pdev->dev);
	if (!fb) {
		ret = -ENOMEM;
		goto err0;
	}

	dev = fb->par;
	dev->fb = fb;
	dev->fb_state = FB_NORMAL;
	dev->id = pdev->id;

	dev->device_id = panel_id[pdev->id];

	ret = lcdc_early_init(platform_data, dev);
	if (ret) {
		goto cleanup;
	}
	//return 0;
	ret = setup_fbmem(dev, pdev);
	if (ret) {
		goto cleanup;
	}

	setup_fb_info(dev);

	ret = register_framebuffer(fb);
	if (ret) {
		goto cleanup;
	}

	if (dev->fb_state == FB_NORMAL) {
		lcdc_init(dev);
	}

	// after init hardware
	dev->need_reinit = 0;

	platform_set_drvdata(pdev, dev);

#ifdef CONFIG_HAS_EARLYSUSPEND
	dev->early_suspend.suspend = sprdfb_early_suspend;
	dev->early_suspend.resume  = sprdfb_late_resume;
	dev->early_suspend.level   = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	register_early_suspend(&dev->early_suspend);
#endif

#if CONFIG_CPU_FREQ
	dev->freq_transition.notifier_call = sprdfb_freq_transition;
	cpufreq_register_notifier(&dev->freq_transition, CPUFREQ_TRANSITION_NOTIFIER);

	dev->freq_policy.notifier_call = sprdfb_freq_policy;
	cpufreq_register_notifier(&dev->freq_policy, CPUFREQ_POLICY_NOTIFIER);
#endif

	return 0;

cleanup:
	fb_free_resources(dev);
err0:
	dev_err(&pdev->dev, "failed to probe sprdfb\n");
	return ret;
}

#ifndef CONFIG_HAS_EARLYSUSPEND
static int sprdfb_suspend(struct platform_device *pdev,pm_message_t state)
{
	struct sprdfb_device *dev = platform_get_drvdata(pdev);
	lcdc_suspend(dev);
	FB_PRINT(2, ("deep sleep: [%s]\n", __FUNCTION__));
	return 0;
}

static int sprdfb_resume(struct platform_device *pdev)
{
	struct sprdfb_device *dev = platform_get_drvdata(pdev);

	lcdc_resume(dev);

	FB_PRINT(2, ("deep sleep: [%s]\n", __FUNCTION__));
	return 0;
}
#endif

static struct platform_driver sprdfb_driver = {
	.probe = sprdfb_probe,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = sprdfb_suspend,
	.resume = sprdfb_resume,
#endif
	.driver = {
		.name = "sprdfb",
		.owner = THIS_MODULE,
	},
};



#define OPT_EQUAL(opt, name) (!strncmp(opt, name, strlen(name)))
#define OPT_INTVAL(opt, name) simple_strtoul(opt + strlen(name) + 1, NULL, 0)
#define OPT_STRVAL(opt, name) (opt + strlen(name))

static __inline__ char * get_opt_string(const char *this_opt, const char *name)
{
	const char *p;
	int i;
	char *ret;

	p = OPT_STRVAL(this_opt, name);
	i = 0;
	while (p[i] && p[i] != ' ' && p[i] != ',')
		i++;
	ret = kmalloc(i + 1, GFP_KERNEL);
	if (ret) {
		strncpy(ret, p, i);
		ret[i] = '\0';
	}
	return ret;
}

static __inline__ int get_opt_int(const char *this_opt, const char *name,
				  int *ret)
{
	if (!ret)
		return 0;

	if (!OPT_EQUAL(this_opt, name))
		return 0;

	*ret = OPT_INTVAL(this_opt, name);
	return 1;
}

static __inline__ int get_opt_bool(const char *this_opt, const char *name,
				   int *ret)
{
	if (!ret)
		return 0;

	if (OPT_EQUAL(this_opt, name)) {
		if (this_opt[strlen(name)] == '=')
			*ret = simple_strtoul(this_opt + strlen(name) + 1,
					      NULL, 0);
		else
			*ret = 1;
	} else {
		if (OPT_EQUAL(this_opt, "no") && OPT_EQUAL(this_opt + 2, name))
			*ret = 0;
		else
			return 0;
	}
	return 1;
}

static int __init sprdfb_setup(char *options)
{
	char *this_opt;

	if (!options || !*options) {
		printk(KERN_ERR "no options\n");
		return 0;
	} else {
		FB_PRINT(2,( "options: %s\n", options));
	}

	/*
	 * The syntax is:
	 *
	 *    video=sprdfb:[<param>=<val>][,<param>=<val>] ...

	 */

	while ((this_opt = strsep(&options, ","))) {
		if (!*this_opt)
			continue;
		else if (get_opt_int(this_opt, "fb0_id", &panel_id[0]))
			;

		else if (get_opt_int(this_opt, "fb1_id", &panel_id[1]))
			;
	}
	return 0;
}

static int __init sprdfb_init(void)
{
	char *option = NULL;
	if (fb_get_options("sprdfb", &option))
		return -ENODEV;
	sprdfb_setup(option);

	lcdc_hardware_init();
	return platform_driver_register(&sprdfb_driver);
}

module_init(sprdfb_init);


