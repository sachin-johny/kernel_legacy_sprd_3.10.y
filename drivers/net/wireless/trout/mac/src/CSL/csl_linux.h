/*****************************************************************************/
/*                                                                           */
/*                     Ittiam 802.11 MAC SOFTWARE                            */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2005                               */
/*                                                                           */
/*  This program  is  proprietary to  Ittiam  Systems  Private  Limited  and */
/*  is protected under Indian  Copyright Law as an unpublished work. Its use */
/*  and  disclosure  is  limited by  the terms  and  conditions of a license */
/*  agreement. It may not be copied or otherwise  reproduced or disclosed to */
/*  persons outside the licensee's organization except in accordance with the*/
/*  terms  and  conditions   of  such  an  agreement.  All  copies  and      */
/*  reproductions shall be the property of Ittiam Systems Private Limited and*/
/*  must bear this notice in its entirety.                                   */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/
/*                                                                           */
/*  File Name         : csl_linux.h                                          */
/*                                                                           */
/*  Description       : This file contains all declarations and functions    */
/*                      related to the chip support library.                 */
/*                                                                           */
/*  List of Functions : None                                                 */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef OS_LINUX_CSL_TYPE

#ifndef CSL_LINUX_H
#define CSL_LINUX_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include <linux/irq.h>
#include <asm/system.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/if_arp.h>
#include <linux/timer.h>
#include <linux/spinlock.h>
#include <linux/spi/spi.h>
#include <linux/version.h>
#include <mach/io.h>
/*leon liu added for cfg80211*/
#ifdef CONFIG_CFG80211
#include <net/cfg80211.h>
#endif

#include "csl_types.h"

#ifdef TROUT_WIFI_NPI
//#include "trout_rf_test_mode.h"
#endif

/* enable error interrupt by zhao 6-25 2013 */
#define ERROR_INT_ENABLE
#ifdef TROUT_TRACE_DBG
#define SPRD_TROUT_GPIO_TEST	90
#endif	/* TROUT_TRACE_DBG */
/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/
#define VIRT_IMMRBAR 0       //add by Hugh
#define TROUT_RF_SPI_REG   (0x1000U << 2)//dumy add for RF 20120618
#define VIRT_ITM_MWLANBAR (0x2000U << 2)  //add by Hugh
#define TROUT_COMM_REG    (0x4000U << 2)  //add by Hugh

/*****************************************************************************/
/* Comman Registers                                                                 */
/*****************************************************************************/

/* RF SPI Registers   */
#define rCOMM_RF_SPI_CTRL   (TROUT_COMM_REG + (0x0012<<2)) //dumy add for RF 20120618
#define rCOMM_RF_SPI_DATA  (TROUT_RF_SPI_REG + (0x0000<<2)) //dumy add for RF 20120618

/* Interrupt Registers   */
#define rCOMM_INT_STAT                 (TROUT_COMM_REG + (0x0004<<2))
#define rCOMM_WIFI_ERR_MSK		(TROUT_COMM_REG + (0x0003<<2))
#define rCOMM_INT_MASK                 (TROUT_COMM_REG + (0x0002<<2))
#define rCOMM_INT_CLEAR                (TROUT_COMM_REG + (0x0000<<2))

// caisf add for HOST POWER SLEEP REG
//#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#define rCOMM_GEN_INT_2ARM7            (TROUT_COMM_REG + (0x000A<<2))
#define rCOMM_INFO0_FROM_HOST          (TROUT_COMM_REG + (0x0060<<2))
#define rCOMM_INFO1_FROM_HOST          (TROUT_COMM_REG + (0x0061<<2))
#define rCOMM_INFO2_FROM_HOST          (TROUT_COMM_REG + (0x0062<<2))
#define rCOMM_INFO3_FROM_HOST          (TROUT_COMM_REG + (0x0063<<2))
#define rCOMM_ARM2HOST_INFO0           (TROUT_COMM_REG + (0x0064<<2))
#define rCOMM_ARM2HOST_INFO1           (TROUT_COMM_REG + (0x0065<<2))
#define rCOMM_ARM2HOST_INFO2           (TROUT_COMM_REG + (0x0066<<2))
#define rCOMM_ARM2HOST_INFO3           (TROUT_COMM_REG + (0x0067<<2))
#ifdef TROUT2_WIFI_IC
#define rSYSREG_CLK_CTRL0              ((0x0043<<2))
#define rSYSREG_CLK_CTRL1              ((0x0044<<2))
#define rSYSREG_POWER_CTRL             ((0x0045<<2))
#else
#define rSYSREG_CLK_CTRL0              ((0x0020<<2))
#define rSYSREG_CLK_CTRL1              ((0x0021<<2))
#define rSYSREG_POWER_CTRL             ((0x0022<<2))
#endif

#define rSYSREG_HOST2ARM_INFO0         ((0x005A<<2))
#define rSYSREG_HOST2ARM_INFO1         ((0x005B<<2))
#define rSYSREG_HOST2ARM_INFO2         ((0x005C<<2))
#define rSYSREG_HOST2ARM_INFO3         ((0x005D<<2))
#define rSYSREG_INFO0_FROM_ARM         ((0x005E<<2))
#define rSYSREG_INFO1_FROM_ARM         ((0x005F<<2))
#define rSYSREG_INFO2_FROM_ARM         ((0x0060<<2))
#define rSYSREG_INFO3_FROM_ARM         ((0x0061<<2))

#define rSYSREG_GEN_ISR_2_ARM7         ((0x01FF<<2))
//#endif


#ifdef MWLAN
#define DPSRAM0_ADDR        (VIRT_ITM_MWLANBAR + 0x00100000)
#define DPSRAM_VBASE_ADDR   (0xFC000000)
#define DPSRAM_PBASE_ADDR   (0xF0000000)
#define PLD0_ADDR           (VIRT_ITM_MWLANBAR)
#define PA_BASE             (PLD0_ADDR + 0x00000000)    // (0x2000U << 2)+0x0
#define CE_BASE             (PLD0_ADDR + 0x00002000)	//for fpage version52, and "(PLD0_ADDR + 0x00000C00)" for ver18.

//masked by Hugh
//#define CE_BASE             (PLD0_ADDR + 0x00010000)
//#define DMA_BASE            (PLD0_ADDR + 0x00020000)
//#define ARB_BASE            (PLD0_ADDR + 0x00030000)
//#define IMMR_BASE           (VIRT_IMMRBAR)
#endif /* MWLAN */


#ifdef MWLAN
#ifdef MAC_ADDRESS_FROM_FLASH
#define MWLAN_FLASH_BASE_ADDR 0xFC080000
#endif /* MAC_ADDRESS_FROM_FLASH */
#endif /* MWLAN */

#define OS_ISR_HANDLED      1
#define OS_ISR_CALL_DSR     2

#define ATTRIB_SECTION(buf) __attribute__ ((section(buf)))

#define SIOCSIWESSID    0x8B1A      /* set ESSID (network name)      */
#define SIOCGIWESSID    0x8B1B      /* get ESSID                     */
#define SIOCSIWRATE     0x8B20      /* set default bit rate (bps)    */
#define SIOCGIWRATE     0x8B21      /* get default bit rate (bps)    */
#define SIOCSIWRTS      0x8B22      /* set RTS/CTS threshold (bytes) */
#define SIOCGIWRTS      0x8B23      /* get RTS/CTS threshold (bytes) */
#define SIOCSIWFRAG     0x8B24      /* set fragmentation thr (bytes) */
#define SIOCGIWFRAG     0x8B25      /* get fragmentation thr (bytes) */
#define SIOCSIWTXPOW    0x8B26      /* set transmit power (dBm)      */
#define SIOCGIWTXPOW    0x8B27      /* get transmit power (dBm)      */
#define SIOCSIWRETRY    0x8B28      /* set retry limits and lifetime */
#define SIOCGIWRETRY    0x8B29      /* get retry limits and lifetime */
#define SIOCSIWFREQ     0x8B04      /* set channel/frequency (Hz)    */
#define SIOCGIWFREQ     0x8B05      /* get channel/frequency (Hz)    */

#ifdef DEBUG_MODE
#define MAC_PA_WRITE_FLAG  0
#define MAC_PA_READ_FLAG   1
#define MAC_CE_WRITE_FLAG  2
#define MAC_CE_READ_FLAG   3
#define MAC_DEBUG_FLAG     4
#define PHY_WRITE_FLAG     5
#define PHY_READ_FLAG      6
#define MAC_COM_READ_FLAG  7
#define MAC_COM_WRITE_FALG 8
#define MAC_SYS_READ_FALG  9
#define MAC_SYS_WRITE_FLAG 10
#define MAC_RF_WRITE_FLAG  11
#define MAC_RF_READ_FLAG   12
#define FM_REG_WRITE_FLAG  13
#define FM_REG_READ_FLAG   14
// modify by Ke.Li at 2013-04-11 for itm_config read ram
#define TROUT_RAM_READ_FLAG 15
#define MAX_COMMAND_FLAG    16
// end modify by Ke.Li at 2013-04-11 for itm_config read ram

#endif /* DEBUG_MODE */

/*****************************************************************************/
/* Data Types                                                                */
/*****************************************************************************/

typedef struct
{
    ISR_FN_T *isr;
    DSR_FN_T *dsr;
} isr_t;

typedef isr_t ISR_T;

typedef struct {
    WORD32   addr;
    WORD32   read_write;
    WORD32   count;
    UWORD32  data[36];
} mem_access_info_t;

typedef enum
{
    SHARED_TO_LOCAL  = 0,
    LOCAL_TO_SHARED  = 1,
    LOCAL_TO_LOCAL   = 2,
    SHARED_TO_SHARED = 3
} DMA_DIRECTION_T;

#ifdef TROUT_SDIO_INTERFACE
struct sdio_trout_data{
	struct sdio_func * func;
	struct mutex sdio_mutex;
};

#else

struct spidev_data
{
    struct spi_device *spi;
    struct mutex buf_lock;
    u8  *buffer;
};
#endif

#ifdef TROUT_B2B_TEST_MODE
typedef int (*B2B_TX_FN)(__user char *pkt, int pkt_len);
typedef int (*B2B_RX_FN)(__user char *buf, int buf_len);

struct b2b_test_qm
{
    struct mutex qm_lock;
    struct list_head rx_pkt_list;
    struct list_head tx_pkt_list;

    UWORD32 tx_cnt;
    UWORD32 rx_cnt;
    UWORD32 tx_list_pkt_cnt;
    UWORD32 rx_list_pkt_cnt;

    wait_queue_head_t rx_wait_queue;
};

typedef struct
{
	struct list_head pkt_node;
	char *pkt;
	UWORD32 len;
}b2b_rx_pkt_s;
#endif	/* TROUT_B2B_TEST_MODE */

typedef enum
{
	TROUT_MEM_STATE_0 = 0,		//tx&rx busy/free, default status.
	TROUT_MEM_STATE_1,			//tx busy, rx free.
	TROUT_MEM_STATE_2			//tx free, rx busy.
}TROUT_MEM_STATE_T;

struct trout_private{
	struct net_device   *dev;
#ifdef CONFIG_CFG80211
	/*leon liu added for cfg80211 2013-2-21*/
	struct wireless_dev *wdev;
#endif
	struct rw_semaphore rst_semaphore;
	//struct mutex	rst_mutex;	//reset_mac need a mutex by zhao
	struct mutex	iphy_mutex;	//initialize_phy need a mutex by zhao
	struct mutex	txsr_mutex;	//tx suspend & resume need a mutex by zhao
	struct mutex    rst_wid_mutex; //xuan yang, 2013-8-23, declare wid mutex
	struct mutex    sm_mutex; //wifi suspend & mac_isr_work need a mutex by zhao
	
    struct workqueue_struct *event_wq;
    struct work_struct  event_work;
    
#ifdef TROUT_SDIO_INTERFACE
    struct sdio_trout_data  sdio_data;
#else
    struct spidev_data  spi_data;
#endif
	//caisf add 2013-02-15
    struct net_device_stats mac_net_stats;

#ifdef TROUT_B2B_TEST_MODE
    struct b2b_test_qm b2b_qm;
#endif

#ifdef WAKE_LOW_POWER_POLICY
	struct mutex ps_mutex;	//add by chwg, 2013.12.7, used for suspend/resume & enter/exit low power mode protect.
#endif

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
//#ifndef WIFI_SLEEP_POLICY
    struct mutex cur_run_mutex; //xuan.yang, 2013-09-17, protect atomic varibles
	u32    resume_complete_state;
//#endif
#endif
//begin by modified junwei.jiang bug232001 2013-12-16
u8 bssid[ETH_ALEN];
u8 ssid[IEEE80211_MAX_SSID_LEN];
size_t ssid_len;
//end by modified junwei.jiang bug232001 2013-12-16
};

typedef enum{
     ALARM_CREATE=0,
     ALARM_ADD,
     ALARM_RUN_START,
     ALARM_RUN_MID,
     ALARM_RUN_END,
     ALARM_STOP,
     ALARM_DEL,
     ALARM_WORK_ENTRY,
     ALARM_WORK_EXIT
 }ALARM_STATE;

typedef struct {
    struct timer_list timer;
    struct work_struct work;
    u32 data;   
	u32 work_proceed_flag; //jiangtao.yi 2013-10-10. Add for marking whether the work associated with the timer has been proceed. BTRUE:had been proceed; BFALSE:not yet.
    atomic_t used;   //jiangtao, add for counting user
    u32 wid;         //work id
	u32 state;       //record time state for debug.
}trout_timer_struct;


//chenq add for test cfg info 2013-01-10
typedef struct {
#ifdef BSS_ACCESS_POINT_MODE
UWORD8 ap_mac[20];
UWORD8 ap_channel;
#else
UWORD8 sta_mac[20];
#endif
} trout_wifi_info_t;

/*  this buffer is just used for tx,rx by zhao */
struct sb_buff{
	unsigned long 	ps;
	unsigned long	size;
	
	void		*tx_start;
	unsigned long	tx_size;

	void		*td_start;
	unsigned long	td_size;
	
	void		*rx_start;
	unsigned long	rx_size;
};

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/
/* TX barrier status by zhao */
#define RST_BARRIER_TX	0
#define RST_BARRIER_HRX	1
#define RST_BARRIER_NRX	2

extern unsigned long	tx_barrier;
//extern struct tasklet_struct *g_event_tasklet;
extern struct net_device *g_mac_dev;
//caisf add 2013-02-15
extern struct net_device_stats *g_mac_net_stats;

extern TROUT_MEM_STATE_T g_trout_mem_state;	//add by chengwg.

#ifdef MWLAN
#ifdef MAC_ADDRESS_FROM_FLASH
extern UWORD32 g_virt_flash_base;
#endif /* MAC_ADDRESS_FROM_FLASH */
#endif /* MWLAN */
extern struct sb_buff	*gsb;

#ifdef IBSS_BSS_STATION_MODE
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
/* remove duplicate defination form many c files, 
 * it should be in h file by zhao */
#define TROUT_AWAKE	1
#define TROUT_DOZE	0
#define TROUT_SLEEP	2

#endif//TROUT_WIFI_POWER_SLEEP_ENABLE
#endif//IBSS_BSS_STATION_MODE


#ifdef WAKE_LOW_POWER_POLICY
typedef struct
{
	UWORD32 tx_pkt_num;
	UWORD32 rx_pkt_num;
	UWORD32 total_pkt_num_record[3];
}LOW_POWER_FLOW_CTRL_T;

typedef void (*wakeup_from_low_power_mode)(bool flag);
extern wakeup_from_low_power_mode wake_low_power_fn;
extern ALARM_HANDLE_T *g_flow_detect_timer;
extern LOW_POWER_FLOW_CTRL_T g_low_power_flow_ctrl;
int enter_low_power_mode(void);
int exit_low_power_mode(BOOL_T restart);
void restart_flow_detect_timer(ALARM_HANDLE_T **hdl, UWORD32 time, UWORD32 data);
INLINE void low_power_init(void)
{
	printk("low power struct init!\n");

	memset(&g_low_power_flow_ctrl, '\0', sizeof(LOW_POWER_FLOW_CTRL_T));
	g_flow_detect_timer = NULL;
}

INLINE void clear_history_flow_record(void)
{
	g_low_power_flow_ctrl.total_pkt_num_record[0] = 0;
	g_low_power_flow_ctrl.total_pkt_num_record[1] = 0;
	g_low_power_flow_ctrl.total_pkt_num_record[2] = 0;
}

#endif
/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern int send_packet_to_stack_lnx(void *msdu_desc);
extern UWORD8 mem_dma(void *dstv, void *srcv, UWORD32 len, DMA_DIRECTION_T dir);
extern void print_banner_lnx(void);
extern void get_phy_mutex(void);
extern void put_phy_mutex(void);
extern unsigned char *get_local_ipadd(void);
extern void delete_alarm(ALARM_HANDLE_T **);
extern void delete_alarm_self(ALARM_HANDLE_T **);
extern void stop_alarm(ALARM_HANDLE_T *);
extern void stop_alarm_self(ALARM_HANDLE_T *);
extern unsigned char *get_local_ipadd(void);

#ifdef TROUT_B2B_TEST_MODE
int b2b_rx_pkt_add(b2b_rx_pkt_s *rx_pkt);
#endif

#ifdef MAC_P2P
extern void schedule_mod_switch(void);
#endif /* MAC_P2P */

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

#if 0	//modified by chengwg.
/* This function converts the virtual address to physical address */
INLINE UWORD32 virt_to_phy_addr(UWORD32 addr)
{
    UWORD32 temp = 0;

    if(addr == 0)
        return 0;

#ifdef MWLAN
    temp = ((addr - DPSRAM_VBASE_ADDR) + DPSRAM_PBASE_ADDR);
#endif /* MWLAN */


    return temp;
}

/* This function converts the physical address to virtual address */
INLINE UWORD32 phy_to_virt_addr(UWORD32 addr)
{
    UWORD32 temp = 0;

    if(addr == 0)
        return 0;

#ifdef MWLAN
    temp = ((addr - DPSRAM_PBASE_ADDR) + DPSRAM_VBASE_ADDR);
#endif /* MWLAN */


    return temp;
}
#else
INLINE UWORD32 virt_to_phy_addr(UWORD32 addr)
{
    return addr;
}

/* This function converts the physical address to virtual address */
INLINE UWORD32 phy_to_virt_addr(UWORD32 addr)
{
    return addr;
}
#endif

INLINE void mac_event_schedule(void)
{
    if(g_mac_dev != NULL)
    {
        struct trout_private *tp = netdev_priv(g_mac_dev);
        
        if((tp != NULL) && (tp->event_wq != NULL))
            queue_work_on(0,tp->event_wq, &tp->event_work);
    }
}

INLINE TROUT_MEM_STATE_T get_trout_mem_state(void)
{
	return g_trout_mem_state;
}

INLINE void set_trout_mem_state(TROUT_MEM_STATE_T state)
{
	g_trout_mem_state = state;
}

#ifdef TROUT_TRACE_DBG
//Hugh: add timer for trace.
#define SPRD_TIMER_BASE 0xE001a000
#define TIMER_REG(off)  (SPRD_TIMER_BASE + (off))
#define TIMER2_LOAD     TIMER_REG(0x0040)
#define TIMER2_VALUE    TIMER_REG(0x0044)
#define TIMER2_CONTROL  TIMER_REG(0x0048)
#define TIMER2_CLEAR    TIMER_REG(0x004C)
#define GREG_REG(off)  (0xE0033000 + (off))
#define GREG_GEN0		GREG_REG(0x0008)
#define TIMER2_INT     7
#define TIMER2_INTERVAL 165UL    //165s
#define TIMER2_CLK_SPEED_MHZ 26    //MHz

#define TIMER2_LOAD_VALUE (TIMER2_CLK_SPEED_MHZ * 1000000 * TIMER2_INTERVAL)



static inline unsigned int algin_up(unsigned int x, unsigned int u)
{
	return 	(x + u - 1) & ~(u - 1);
}


INLINE UWORD32 sprd_timer2_count_get(void)
{
/* 
 * leonliu modified timer2 value retrieving
 * return  (TIMER2_LOAD_VALUE - (UWORD32)__raw_readl(TIMER2_VALUE));	//38.46 ns
 */
   return 0;
}

INLINE void sprd_nstime_get(struct timespec *timeval)
{
    unsigned long temp64;
    unsigned long temp32;
    
    temp64 = sprd_timer2_count_get();
    timeval->tv_sec = temp64 / (TIMER2_CLK_SPEED_MHZ * 1000000);
	temp32 = temp64 % (TIMER2_CLK_SPEED_MHZ * 1000000);

	temp64 = temp32 / TIMER2_CLK_SPEED_MHZ;			//us.
	timeval->tv_nsec = temp64 * 1000;

	temp64 = temp32 % TIMER2_CLK_SPEED_MHZ;
    timeval->tv_nsec += (temp64 * 1000) / TIMER2_CLK_SPEED_MHZ;
    //timeval->tv_nsec = ((unsigned long long)(temp64 % (TIMER2_CLK_SPEED_MHZ * 1000000)) * 1000) / TIMER2_CLK_SPEED_MHZ;
}
#endif /* TROUT_TRACE_DBG */

#endif /* CSL_LINUX_H */

#endif /* OS_LINUX_CSL_TYPE */
