#ifndef SPRD_DEBUG_H
#define SPRD_DEBUG_H

#include <linux/sched.h>
#include <linux/semaphore.h>



#if defined(CONFIG_SPRD_DEBUG)

#define MAGIC_ADDR			(SPRD_IRAM_BASE + 0x00003f64)
#define SPRD_INFORM0			(MAGIC_ADDR + 0x00000004)
#define SPRD_INFORM1			(SPRD_INFORM0 + 0x00000004)
#define SPRD_INFORM2			(SPRD_INFORM1 + 0x00000004)
#define SPRD_INFORM3			(SPRD_INFORM2 + 0x00000004)
#define SPRD_INFORM4			(SPRD_INFORM3 + 0x00000004)
#define SPRD_INFORM5			(SPRD_INFORM4 + 0x00000004)
#define SPRD_INFORM6			(SPRD_INFORM5 + 0x00000004)

union sprd_debug_level_t {
	struct {
		u16 kernel_fault;
		u16 user_fault;
	} en;
	u32 uint_val;
};

extern union sprd_debug_level_t sprd_debug_level;

extern int sprd_debug_init(void);

extern void sprd_debug_check_crash_key(unsigned int code, int value);

extern void sprd_getlog_supply_fbinfo(void *p_fb, u32 res_x, u32 res_y, u32 bpp,
				     u32 frames);
extern void sprd_getlog_supply_loggerinfo(void *p_main, void *p_radio,
					 void *p_events, void *p_system);
extern void sprd_getlog_supply_kloginfo(void *klog_buf);

extern void sprd_gaf_supply_rqinfo(unsigned short curr_offset,
				  unsigned short rq_offset);

extern void sprd_debug_save_pte(void *pte, int task_addr);

#else
static inline int sprd_debug_init(void)
{
	return 0;
}

static inline void sprd_debug_check_crash_key(unsigned int code, int value)
{
}

static inline void sprd_getlog_supply_fbinfo(void *p_fb, u32 res_x, u32 res_y,
					    u32 bpp, u32 frames)
{
}

static inline void sprd_getlog_supply_meminfo(u32 size0, u32 addr0, u32 size1,
					     u32 addr1)
{
}

static inline void sprd_getlog_supply_loggerinfo(void *p_main,
						void *p_radio, void *p_events,
						void *p_system)
{
}

static inline void sprd_getlog_supply_kloginfo(void *klog_buf)
{
}

static inline void sprd_gaf_supply_rqinfo(unsigned short curr_offset,
					 unsigned short rq_offset)
{
}

void sprd_debug_save_pte(void *pte, unsigned int faulttype );
{
}

#endif


#endif /* SPRD_DEBUG_H */
