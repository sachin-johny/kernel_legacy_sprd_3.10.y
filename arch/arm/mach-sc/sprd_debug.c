/*
 *  sprd_debug.c
 *
 */

#include <linux/errno.h>
#include <linux/ctype.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/sysrq.h>
//#include <mach/regs-pmu.h>

/* FIXME: This is temporary solution come from Midas */
#include <asm/outercache.h>
#include <asm/cacheflush.h>
#include <asm/io.h>
#include <linux/sched.h>
#include <linux/smp.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/module.h>

#include <mach/hardware.h>
#include <mach/system.h>
#include <mach/sprd_debug.h>
#include <linux/fs.h>
#include <linux/seq_file.h>
#include <mach/pm_debug.h>


/* enable/disable sprd_debug feature
 * level = 0 when enable = 0 && enable_user = 0
 * level = 1 when enable = 1 && enable_user = 0
 * level = 0x10001 when enable = 1 && enable_user = 1
 * The other cases are not considered
 */
union sprd_debug_level_t sprd_debug_level = { .en.kernel_fault = 1, };

module_param_named(enable, sprd_debug_level.en.kernel_fault, ushort, 0644);
module_param_named(enable_user, sprd_debug_level.en.user_fault, ushort, 0644);
module_param_named(level, sprd_debug_level.uint_val, uint, 0644);

static const char *gkernel_sprd_build_info_date_time[] = {
	__DATE__,
	__TIME__
};

static char gkernel_sprd_build_info[100];


void sprd_debug_check_crash_key(unsigned int code, int value)
{
	static bool volup_p;
	static bool voldown_p;
	static int loopcount;

	if (!sprd_debug_level.en.kernel_fault)
		return;

#if 0
	/* Must be deleted later */
	pr_info("Test %s:key code(%d) value(%d),(up:%d,down:%d)\n", __func__, code, value, volup_p, voldown_p);
#endif

	/* Enter Force Upload
	 *  Hold volume down key first
	 *  and then press power key twice
	 *  and volume up key should not be pressed
	 */
	if (value) {
		if (code == KEY_VOLUMEUP)
			volup_p = true;
		if (code == KEY_VOLUMEDOWN)
			voldown_p = true;
		if (volup_p && voldown_p) {
			if (code == KEY_POWER) {
				pr_info("%s: Crash key count : %d\n", __func__, ++loopcount);
				if (loopcount == 2)
					panic("Crash Key");
			}
		}
	} else {
		if (code == KEY_VOLUMEUP)
			volup_p = false;
		if (code == KEY_VOLUMEDOWN) {
			loopcount = 0;
			voldown_p = false;
		}
	}
}

static void sprd_debug_set_build_info(void)
{
	char *p = gkernel_sprd_build_info;
	sprintf(p, "Kernel Build Info : ");
	strcat(p, " Date:");
	strncat(p, gkernel_sprd_build_info_date_time[0], 12);
	strcat(p, " Time:");
	strncat(p, gkernel_sprd_build_info_date_time[1], 9);
}

__init int sprd_debug_init(void)
{
	if (!sprd_debug_level.en.kernel_fault)
		return -1;

	sprd_debug_set_build_info();

	return 0;
}

int get_sprd_debug_level(void)
{
	return sprd_debug_level.uint_val;
}


