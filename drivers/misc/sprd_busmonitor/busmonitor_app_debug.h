#ifndef _BUSMONITOR_APP_DEBUG_H_
#define _BUSMONITOR_APP_DEBUG_H_

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <mach/arch_misc.h>
#include <mach/sci.h>
#include <mach/hardware.h>
#include <mach/sci_glb_regs.h>
#include <mach/irqs.h>
#include <mach/busmonitor.h>
#include <linux/sched.h>
#include <linux/workqueue.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/types.h>
#include <linux/moduleparam.h>
#include <linux/pci.h>
#include <asm/unistd.h>
#include <linux/device.h>
#include <linux/mm_types.h>
#include <asm/ioctl.h>



#define MONITOR_LENTH 128
#define MONITOR_TIMES 10

#define BUSMON_MAJOR 256
#define BUSMON_MINOR 0
#define BUSMON_BUF_SIZE PAGE_SIZE<<2

#define BUSMON_IOC_MAGIC  'M'

#define BUSMON_CLR               _IOW(BUSMON_IOC_MAGIC, 1, unsigned int)
#define BUSMON_SET               _IOW(BUSMON_IOC_MAGIC, 2, unsigned int)
#define BUSMON_WRITE_ARRAY       _IOW(BUSMON_IOC_MAGIC, 3, unsigned int)
#define BUSMON_WRITE_VARIABLE    _IOW(BUSMON_IOC_MAGIC, 4, unsigned int)

#define CONFIG_SHOW_BUSMON_LOG

#ifdef CONFIG_SHOW_BUSMON_LOG
#define BUSMON_PRINT printk
#else
#define BUSMON_PRINT
#endif

struct saved_thread_info {
        unsigned u_id;
        char name[16];
};

struct busmon_match_data {
	unsigned int data;
	struct saved_thread_info thread_info;
};

struct busmon_match {
    unsigned int addr;
    short cur_times, all_times;
    struct busmon_match_data match_data[MONITOR_TIMES];
};

struct busmon_read_data {
	unsigned int read_data;
	unsigned int read_addr;
	unsigned int base_addr;
	unsigned int max_addr;
	struct saved_thread_info read_thread_info;
};

struct sci_axi_bm_reg {
	u32 intc;
	u32 cfg;
	u32 addr_min;
	u32 addr_max;
	u32 addr_msk;
	u32 data_min_l;
	u32 data_min_h;
	u32 data_max_l;
	u32 data_max_h;
	u32 data_msk_l;
	u32 data_msk_h;
	u32 cnt_win_len;
	u32 peak_win_len;
	u32 match_addr;
	u32 match_cmd;
	u32 match_data_l;
	u32 match_data_h;
	u32 rtrans_in_win;
	u32 rbw_in_win;
	u32 rlatency_in_win;
	u32 wtrans_in_win;
	u32 wbw_in_win;
	u32 wlatency_in_win;
	u32 peakbw_in_win;
};

struct busmon_read_data g_match_data = {0x0};
struct busmon_match *g_data =NULL;
unsigned int monitor_lenth = 0;
char *user_buf = NULL;

struct work_struct read_match_data_wq = {0x0};

struct saved_thread_info thread_info = {0x0};
bool change_flag = false;

struct cdev *busmon_cdev = NULL;
struct class *busmon_class = NULL;

static void bm_ddr_callback();


#endif
