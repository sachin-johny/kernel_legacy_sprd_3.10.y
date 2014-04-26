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
/*  File Name         : csl_linux.c                                          */
/*                                                                           */
/*  Description       : This file contains all declarations and functions    */
/*                      related to the chip support library for Linux OS.    */
/*                                                                           */
/*  List of Functions : None                                                 */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef OS_LINUX_CSL_TYPE

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "iconfig.h"
#include "controller_mode_if.h"
#include "core_mode_if.h"
#include "csl_if.h"
#include "fsm.h"
#include "host_if.h"
#include "proc_if.h"
#include "mac_init.h"
#include "itypes.h"
#include "maccontroller.h"
#include "mh.h"
#include "buff_desc.h"
#include "common.h"
#include "test_config.h"
#include "mib.h"
#include "runmode.h"
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/err.h>

#include <linux/inetdevice.h>

#include "trout_wifi_rx.h"
#include "spi_interface.h"
#include "sta_prot_if.h"

/*leon liu added gpio related headers*/
#include <mach/pinmap.h>
#include <asm/io.h>
#include <mach/hardware.h>

/*#ifndef DV_SIM
#include <mach/mfp.h>
#endif*/

#include "trout_trace.h"
#include "wireless.h"
#include "qmu_if.h"
#include "prot_if.h"
#include "core_mode_if.h"
#include "rw_register.h"
#include "meter.h"
#include "qmu_tx.h"

#include <linux/platform_device.h>

#define TROUT_WIFI_VERSION "v0.63"
/*leon liu added header for cfg80211*/
#ifdef CONFIG_CFG80211
#include "trout_cfg80211.h"
extern struct sdio_func *trout_get_sdio_func();
#endif

/*leon liu added GPIO register configuration macro*/
#define SPRD_GPIO_REG_WRITEL(val, reg)    do{__raw_writel(val, CTL_PIN_BASE + (reg));}while(0)


#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
//zhou add
#include <linux/earlysuspend.h>
#include "itm_wifi_iw.h"
#include "cglobals.h"
/*leon liu added for powersave timer 2013-4-1*/
#include "ps_timer.h"
#include "pm_sta.h"
#endif
//zhuyg add
#ifdef TROUT_WIFI_NPI
#include <linux/wakelock.h>
static struct wake_lock wifi_npi_power_lock;
extern void npi_tx_data_init(void);
extern void npi_tx_data_release(void);
#endif

/*bing li added for NPI support*/
#ifdef TROUT_WIFI_NPI
#include "trout_wifi_npi.h"
#include "npi_interface.h"
#endif

static struct platform_device *pdev;
#ifdef TROUT2_WIFI_IC

#define WIFI_MODULE	(1<<0)
extern bool Set_Trout_PowerOn( unsigned int MODE_ID );
extern bool Set_Trout_PowerOff(unsigned int MODE_ID);
//extern bool Set_Trout_Download_BT_Code(void);
//extern bool Set_Trout_RF_Start(unsigned int mode);
//extern bool Set_Trout_RF_Stop(unsigned int mode);
#endif
//chenq add
extern struct iw_handler_def itm_iw_handler_def;
extern unsigned char * g_itm_config_buf;

/*extern int sprd_3rdparty_gpio_wifi_reset;
extern int sprd_3rdparty_gpio_wifi_pwd;
extern int sprd_3rdparty_gpio_wifi_irq;*/
//chenq mod a flag itm_mac_close1_open2
//extern int itm_mac_close_flag;
int itm_mac_close1_open2 = 0;
unsigned long	tx_barrier = 0;


trout_wifi_info_t trout_wifi_info;

UWORD8 trout_wifi_cfg_path[] = "/system/lib/modules/trout_wifi.cfg";
UWORD8 trout_rand_mac_path[] = "/data/rand_mac.txt";


#ifdef IBSS_BSS_STATION_MODE
//chenq add 2012-10-29
extern int itm_scan_flag;
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
//#define TROUT_AWAKE  1
typedef void (*sdio_trout_awake)(bool flag);
extern unsigned char g_trout_state;
extern sdio_trout_awake trout_awake_fn;
/*leon liu added, detail info about sta_wakeup_combo is in pm_sta.c 2013-4-4*/
extern sta_wakeup_combo(UWORD8 );
#endif
#endif
#ifdef CONFIG_ARCH_SC7710
#define SPRD_TROUT_INT_GPIO 152
#elif defined(CONFIG_ARCH_SC8825)
#define SPRD_TROUT_INT_GPIO 52
#elif defined(CONFIG_MACH_SP6821A)
#define SPRD_TROUT_INT_GPIO 142
#else
#ifdef USE_TROUT_PHONE
#define SPRD_TROUT_INT_GPIO 141
#else
#define SPRD_TROUT_INT_GPIO 136
#endif
#endif

/*leon liu modified sprd_3rdparty_gpio_wifi_irq to static int*/
static int sprd_3rdparty_gpio_wifi_irq = SPRD_TROUT_INT_GPIO;

#define MAC_CREATE_DATE __DATE__ ", " __TIME__	//caisf mod //add by chengwg.

//module params.
char *usrmac = NULL;
bool txmode = 1;     //0: rx mode; 1:txmode; only used in B2B test.
struct task_struct *mac_int_task = NULL;

extern BOOL_T switch_flag;
extern UWORD8 g_enable_debug_print;
extern void delete_mac_interrupts(void);
extern int itm_set_power_type(UWORD8 type);

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifdef WIFI_SLEEP_POLICY
struct wake_lock  scan_lock;
#endif
#endif

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
    #define TROUT_MODULE_FAIL               0
    #define TROUT_MODULE_LOADING            1
    #define TROUT_MODULE_LOADED             2
    #define TROUT_MODULE_UNLOADING          3
    #define TROUT_MODULE_UNLOADED           4
    #define TROUT_MODULE_IDLE               5
	
    extern void set_trout_module_state(unsigned int st);
    extern unsigned int check_trout_module_state(unsigned int st);
#endif
//libing add for bugger: driver can't unload.
extern int itm_get_dhcp_status();

DEFINE_MUTEX(open_mutex);
static int driver_exit =0;

//chenq add for use external sdio spi if 2012-12-14
//#define USE_INTERNAL_SDIO_SPI_IF

module_param(usrmac, charp, S_IRUGO);
module_param(txmode, bool, S_IRUGO);

/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/
static int  mac_init_fn(struct net_device *dev);
static int  mac_xmit(struct sk_buff *skb, struct net_device *dev);
static int  mac_open(struct net_device *dev);
static int  mac_close(struct net_device *dev);
static void mac_tx_timeout(struct net_device *dev);	//add by chengwg!
static int  mac_ioctl(struct net_device *dev, struct ifreq *req, int cmd);
static void mac_multicast_list(struct net_device *dev);
//caisf add 2013-02-15
static struct net_device_stats *mac_net_get_stats(struct net_device *dev);
extern void mac_isr_work(struct work_struct *work);
extern void select_channel_rf(UWORD8 channel, UWORD8 sec_offset);
extern int get_tx_pwr_from_nv(UWORD32* tx_pwr_80211b,UWORD32* tx_pwr_80211g,UWORD32* tx_pwr_80211n, UWORD32* delta_0x57);

//static void mac_tasklet(unsigned long inp);

/*****************************************************************************/
/* Global Variables Definition                                               */
/*****************************************************************************/
static const struct net_device_ops trout_netdev_ops =
{
    .ndo_open = mac_open,
    .ndo_stop = mac_close,
    .ndo_start_xmit = mac_xmit,
    .ndo_do_ioctl = mac_ioctl,
    .ndo_set_multicast_list = mac_multicast_list,
    .ndo_get_stats = mac_net_get_stats
    //.ndo_tx_timeout = mac_tx_timeout
};

/* Net device structure, initializing all the functions to MAC functions */

struct net_device *g_mac_dev = NULL;
struct net_device_stats *g_mac_net_stats = NULL; // caisf add for wifi statistics 0212

struct workqueue_struct *g_event_wq;
struct work_struct g_event_work;

//struct tasklet_struct *g_event_tasklet;
/* Declaring the tasklet for processing events. */
//DECLARE_TASKLET(g_wlan_tasklet, mac_tasklet, 0);
struct sb_buff   sbuf;
struct sb_buff	*gsb = &sbuf;

/* In STA mode: share_tx_pkt_mem_size + share_tx_dscr_mem_size about 250kb, and host
    normal rx buffer size is about 256kb, when trout share ram is 144kb, all mem is
    250+256=506kb; In AP mode: share_tx_pkt_mem_size + share_tx_dscr_mem_size about
    284kb, and host normal rx buffer size is about 256kb when trout share ram is
    144kb, all mem is 284+256=540kb, so for compatible sta and ap mode, the buffer
    set to 540kb is necessary.
*/
#if 0
void alloc_sb_buf(void)
{
	char *pc, *pb;
	unsigned int sz;

	pc = (char *)gsb->ps;
	pb = pc;
	if(gsb->size > 0x40000)
		sz = ((gsb->size >> 2) * 3);
	else	
		sz = gsb->size >> 1;

	printk("%s sz = %x\n", __func__, sz);
	if(sz >= algin_up(SHARED_PKT_MEM_SIZE + SHARED_DSCR_MEM_SIZE, 4)){
		TROUT_DBG4("%s: alloc mem succeed!\n", __func__);
		gsb->tx_start = (void *)gsb->ps;
		gsb->tx_size = SHARED_PKT_MEM_SIZE;

		pc += algin_up(SHARED_PKT_MEM_SIZE, 4);
		gsb->td_start = (void *)pc;
		gsb->td_size = SHARED_DSCR_MEM_SIZE;

		pc = pb + sz;
		gsb->rx_start = (void *)pc;
		gsb->rx_size = gsb->size - sz;
	}else{
		TROUT_DBG4("%s: alloc mem failed, use default way!\n", __func__);
		gsb->tx_start = NULL;
		gsb->tx_size = 0;
		gsb->td_start = NULL;
		gsb->td_size = 0;
		gsb->rx_start = (void *)gsb->ps;
		gsb->rx_size = gsb->size - sz;
	}

}

#else
void alloc_sb_buf(void)
{
	char *pc, *pb;
	unsigned int sz;

	pc = (char *)gsb->ps;

	sz = algin_up(SHARED_PKT_MEM_SIZE + SHARED_DSCR_MEM_SIZE, 4);
	if(gsb->size - sz >= MAX_HOST_RX_BUF_SIZE)
	{
		TROUT_DBG4("%s: alloc mem succeed!\n", __func__);
		gsb->tx_start = (void *)gsb->ps;
		gsb->tx_size = SHARED_PKT_MEM_SIZE;

		pc += algin_up(SHARED_PKT_MEM_SIZE, 4);
		gsb->td_start = (void *)pc;
		gsb->td_size = SHARED_DSCR_MEM_SIZE;
		
		pc += algin_up(SHARED_DSCR_MEM_SIZE, 32);
		gsb->rx_start = (void *)pc;
		gsb->rx_size = (unsigned long)((void *)(gsb->ps + gsb->size) - (void *)pc);
	}
	else
	{
		TROUT_DBG4("%s: alloc mem failed, use default way!\n", __func__);
		gsb->tx_start = NULL;
		gsb->tx_size = 0;
		gsb->td_start = NULL;
		gsb->td_size = 0;
		gsb->rx_start = (void *)gsb->ps;
		gsb->rx_size = gsb->size;
	}

	TROUT_DBG4("%s: total buf size: 0x%x\n", __func__, gsb->size);
	TROUT_DBG4("%s: tx_start: 0x%p, tx_size: 0x%x\n", __func__, gsb->tx_start, gsb->tx_size);
	TROUT_DBG4("%s: td_start: 0x%p, td_size: 0x%x\n", __func__, gsb->td_start, gsb->td_size);
	TROUT_DBG4("%s: rx_start: 0x%p, rx_size: 0x%x\n", __func__, gsb->rx_start, gsb->rx_size);
}
#endif



#ifdef MWLAN
#ifdef MAC_ADDRESS_FROM_FLASH
UWORD32 g_virt_flash_base = 0;
#endif /* MAC_ADDRESS_FROM_FLASH */
#endif /* MWLAN */

/*****************************************************************************/
/*                                                                           */
/*  Function Name : mac_tasklet                                              */
/*                                                                           */
/*  Description   : This function provides the does the background event     */
/*                  processing.                                              */
/*                                                                           */
/*  Inputs        : Integer Argument                                         */
/*                                                                           */
/*  Globals       : None                                                     */
/*  Processing    : None                                                     */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void mac_event_work(struct work_struct *work)
{
	TROUT_FUNC_ENTER;
    process_all_events();
    TROUT_FUNC_EXIT;
}
/*****************************************************************************/
/*                                                                           */
/*  Function Name : mac_multicast_list                                       */
/*                                                                           */
/*  Description   : This is a dummy function and does nothing.               */
/*                                                                           */
/*  Inputs        : Network device driver structure                          */
/*                                                                           */
/*  Globals       : None                                                     */
/*  Processing    : None                                                     */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void mac_multicast_list(struct net_device *dev)
{
    /* Do Nothing */
}

/* this function used to get loacl ip address, but caller must guarantee
 * this is function called when wifi connection was established.
 */
unsigned char *get_local_ipadd(void)
{
	struct in_device *id;
	struct in_ifaddr *iif;
	unsigned char *cp = NULL;
	int ix = 0;

	if(!g_mac_dev)
		return NULL;

	id = (struct in_device *)g_mac_dev->ip_ptr;
	if(!id){
		printk("has no in_device\n");
		return NULL;
	}

	iif = (struct in_ifaddr *)id->ifa_list;
	if(!iif){
		printk("ifa_list is NULL\n");
		return NULL;
	}


	/* find the interface with name "wlan", then get it's IP add */
	while(memcmp(iif->ifa_label, "wlan0", 5)){
		iif = iif->ifa_next;
		if(!iif)
			break;
	}
	if(!iif){
		printk("BUG! no interface named 'wlan0'\n");
		return NULL;
	}
#if 0
	cp = (unsigned char *)&iif->ifa_local;
	printk("IP-address:\n");
	for(ix = 0; ix < 4; ix++)
		printk("%02X ", *cp++);
	printk("\n");
#endif
	/* here, we get correct IP address */
	return (unsigned char *)&iif->ifa_local;

}
//////////////////////////////////////////////////////////
TROUT_MEM_STATE_T g_trout_mem_state = TROUT_MEM_STATE_0;
UWORD32 g_tx_pkt_count = 0;
UWORD32 g_tx_times = 0;
UWORD32 g_rx_pkt_count = 0;
UWORD32 g_rx_times = 0;


// Ke.Li add read rand_mac.txt or create rand mac at 2013-03-21
//chenq add for trout wifi cfg 2013-01-10
void init_from_cfg(void)
{
	struct file * fp = 0; 
	mm_segment_t fs; 
	loff_t *pos;

	UWORD8 file_data[64] = {0};
	UWORD8 * tmp_p = NULL;
    WORD32 i = 0;
	UWORD8 rand_char;
	WORD32 all_zero = 1;

	memset(&trout_wifi_info,0x00,sizeof(trout_wifi_info));
	fp = filp_open(trout_wifi_cfg_path, O_RDONLY, 0);
	if(IS_ERR(fp)) 
	{
        // get rand_mac.txt
	    fp = filp_open(trout_rand_mac_path, O_RDONLY, 0);
	    if(IS_ERR(fp)) 
	    { 
            goto create_rand_mac;
	    }
        else
        {
	        fs = get_fs(); 
	        set_fs(KERNEL_DS);
	        pos = &(fp->f_pos); 
	        vfs_read(fp, file_data,sizeof(file_data), pos);
		/*leon liu added for bug 237898 -- Turn on/off SOFTAP 100000, crash*/
		filp_close(fp, NULL);
		set_fs(fs);
            if(strlen((WORD8*)file_data) >= 35)
            {
                tmp_p = file_data + 18;
		        #ifdef BSS_ACCESS_POINT_MODE
		        memcpy(trout_wifi_info.ap_mac,tmp_p,17);
		        usrmac = trout_wifi_info.ap_mac;
    		    #else
    		    memcpy(trout_wifi_info.sta_mac,tmp_p,17);
    		    usrmac = trout_wifi_info.sta_mac;
    		    #endif                
				printk("get wifi mac from rand_mac.txt: %s\n", usrmac);
            }
            else
            {
                goto create_rand_mac;
            }
        }
	}
    else
    {
	    fs = get_fs(); 
	    set_fs(KERNEL_DS);

	    pos = &(fp->f_pos); 
	    vfs_read(fp, file_data,sizeof(file_data), pos);

	    if((tmp_p = strstr(file_data,"mac=")) != NULL )
	    {
		    tmp_p += strlen("mac=");
		    #ifdef BSS_ACCESS_POINT_MODE
		    memcpy(trout_wifi_info.ap_mac,tmp_p,17);
		    usrmac = trout_wifi_info.ap_mac;
		    #else
		    memcpy(trout_wifi_info.sta_mac,tmp_p,17);
		    usrmac = trout_wifi_info.sta_mac;
		    #endif
			printk("get wifi mac from trout_wifi.cfg: %s\n", usrmac);
	    }

	    if((tmp_p = strstr(file_data,"channel=")) != NULL )
	    {
		    tmp_p += strlen("channel=");
		    #ifdef BSS_ACCESS_POINT_MODE
		    if((((*tmp_p - '0') >= 1) && ((*tmp_p - '0') <= 11))
		        &&(((*(tmp_p+1) - '0') >= 1) && ((*(tmp_p+1) - '0') <= 11)))
		    {
			    trout_wifi_info.ap_channel = 
				        (*tmp_p - '0')*10 + (*(tmp_p+1) - '0');
		    }
		    else
		    {
		        trout_wifi_info.ap_channel = 
				        (((*tmp_p - '0') >= 1) && ((*tmp_p - '0') <= 11)) ?
		                (*tmp_p - '0') : 0;
		    }

		    //printk("chenq debug channel=%d\n",trout_wifi_info.ap_channel);
		    #else
		    ;
		    #endif
	    }

	    filp_close(fp, NULL);
	    set_fs(fs);
    }
    return;

create_rand_mac:

    for(i=0; i<17; i++)
    {
        if(i==0 || i==1)
        {
            file_data[i] = '0';
        }
        else if(i==2 || i==5 || i==8 || i==11 || i==14)
        {
            file_data[i] = ':';
        }
        else
        {
			get_random_bytes(&rand_char, 1);
			file_data[i] = (rand_char % 16) + 48;
			if(file_data[i] > 57)
			{
				file_data[i] += 7;
				all_zero = 0;
			}
			else if(file_data[i] != 48)
			{
				all_zero = 0;
			}
			else if(i == 16 && all_zero == 1)
			{
				file_data[i] = 49;
			}
        }
    }
	#ifdef BSS_ACCESS_POINT_MODE
	memcpy(trout_wifi_info.ap_mac,file_data,17);
	usrmac = trout_wifi_info.ap_mac;
	#else
	memcpy(trout_wifi_info.sta_mac,file_data,17);
	usrmac = trout_wifi_info.sta_mac;
	#endif
	printk("create rand wifi mac: %s\n", usrmac);

}
// END: Ke.Li add read rand_mac.txt or create rand mac at 2013-03-21


/*****************************************************************************/
/*                                                                           */
/*  Function Name : mac_init_fn                                              */
/*                                                                           */
/*  Description   : This function is called when the driver is initialized.  */
/*                                                                           */
/*  Inputs        : Network device driver structure                          */
/*                                                                           */
/*  Globals       : None                                                     */
/*  Processing    : None                                                     */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

int mac_init_fn(struct net_device *dev)
{
    UWORD32 temp32;
#ifdef MWLAN
#ifdef MAC_ADDRESS_FROM_FLASH
    g_virt_flash_base = (UWORD32)(ioremap(MWLAN_FLASH_BASE_ADDR, 32*1024)); /* Flash */
#endif /* MAC_ADDRESS_FROM_FLASH */
#endif /* MWLAN */

	TROUT_FUNC_ENTER;
    /* Detect the presence of the correct MAC H/w by reading the required    */
    /* MAC H/w register. If not detected print a message and exit.           */
    /* Note that at least memory should be mapped for this detection to work */
    //if(rMAC_HW_ID != MAC_HW_ID_INIT_VALUE)
   if((temp32=host_read_trout_reg((UWORD32)rMAC_HW_ID)) != MAC_HW_ID_INIT_VALUE)
    {
        TROUT_DBG1("trout MAC H/w not detected(got:%x). Exiting.\n", temp32);
        TROUT_FUNC_EXIT;
        return -EINVAL;
    }

    TROUT_DBG4("trout: vid=0x%08x.\n", temp32);
    
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
	// caisf add for power sleep top setup, 20130129
	/*host_write_trout_reg((UWORD32)0x1241C, (UWORD32)rSYSREG_CLK_CTRL1); [>done by FM<] */
	//host_write_trout_reg((UWORD32)0x700, (UWORD32)rSYSREG_POWER_CTRL); //[>done by SDIO<]

    TROUT_DBG4("PS: system reg CLK_CTRL0, CLK_CTRL1, POWER_CTRL = %#x, %#x, %#x\n",
        host_read_trout_reg((UWORD32)rSYSREG_CLK_CTRL0),
        host_read_trout_reg((UWORD32)rSYSREG_CLK_CTRL1),
        host_read_trout_reg((UWORD32)rSYSREG_POWER_CTRL));
#endif


#ifdef LOCALMEM_TX_DSCR
    /* Request for Local Memory from Linux. */
    g_local_mem_dma = kmalloc(LOCAL_DSCR_MEM_SIZE, GFP_DMA);
    if(g_local_mem_dma == NULL)
    {
        TROUT_DBG1("Local Memory Allocation Error. Exiting.\n");
        TROUT_FUNC_EXIT;
        return -EINVAL;
    }
#endif /* LOCALMEM_TX_DSCR */

    /* Assign the tasklets to a global pointer */
    //g_event_tasklet = &g_wlan_tasklet;

#ifndef MAC_HW_UNIT_TEST_MODE
    /* The following function does all the required initialization */
    if(main_function() < 0)
    	return -1;
#else /* MAC_HW_UNIT_TEST_MODE */
    test_main_function();
#endif /* MAC_HW_UNIT_TEST_MODE */

    /* now initialize few of the fields in dev structure */
    memcpy(dev->dev_addr, mget_StationID(), 6);

    if(!is_valid_ether_addr(dev->dev_addr))
    {
        TROUT_DBG1("Error: Wrong MAC address\n");
        TROUT_FUNC_EXIT;
        return -EINVAL;
    }

	TROUT_FUNC_EXIT;
    return 0;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : mac_open                                                 */
/*                                                                           */
/*  Description   : This function is called when the device is activated.    */
/*                                                                           */
/*  Inputs        : Network device driver structure                          */
/*                                                                           */
/*  Globals       : None                                                     */
/*  Processing    : None                                                     */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
#ifdef TROUT_TRACE_DBG
static int gpio_test_init(void)
{
/*leon liu stripped gpio_test_init*/
#if 0
	unsigned long gpio_cfg = MFP_CFG_X(RFCTL0, AF3, DS1, F_PULL_UP, S_PULL_UP, IO_OE);
	int status;
	
	sprd_mfp_config(&gpio_cfg, 1);
	status =gpio_request(SPRD_TROUT_GPIO_TEST, "trout test");
	if(status)
	{
		printk("cannot alloc gpio for trout test\n");
		return -1;
	}
	
	gpio_direction_output(SPRD_TROUT_GPIO_TEST, 0);
	return status;
#endif
	return 0;
}
#endif /*TROUT_TRACE_DBG*/

#if 0
int mac_open(struct net_device *dev)
{
    /* This fn is called, when this device is opened */
    struct trout_private *tp = netdev_priv(dev);
    
	TROUT_FUNC_ENTER;
	if(itm_mac_close1_open2 != 2)
	{
		TROUT_DBG4("itm mac_open end not do anything!\n");
		return 0;
	}
#ifdef BSS_ACCESS_POINT_MODE
	send_mac_status(MAC_AP_UP);
#endif

	if(tp->event_wq == NULL)
	{
    tp->event_wq = create_workqueue("event_wq");
    INIT_WORK(&tp->event_work, mac_event_work);
	}
    /* Call this function, it will set up all the required flags and bind    */
    /* the driver with higher layer                                          */
    ether_setup(dev);

#ifndef MAC_HW_UNIT_TEST_MODE
    //config_op_params(&g_mac);
#else /* MAC_HW_UNIT_TEST_MODE */
    enable_operation(&g_mac);

    /* Once all initializations are complete the PA control register is      */
    /* programmed to enable PHY and MAC H/w.                                 */
    enable_machw_phy_and_pa();

    /* Start MAC controller task */
    start_mac_controller();
#endif /* MAC_HW_UNIT_TEST_MODE */

    /* Start the network interface queue for this device */
    netif_start_queue(dev);

#ifdef TROUT_B2B_TEST_MODE
    netif_stop_queue(dev);
    
    select_channel_rf(8, 0);        //set channel 9.
#endif

    start_mac_daemons();


#ifdef TROUT2_WIFI_FPGA
#ifdef TROUT2_WIFI_FPGA_RF55
host_write_trout_reg(0x800000, (((UWORD32)0x62)<<2));//dumy add for liusy test 0705  :: 07/11 tx inv enable
host_write_trout_reg(0x20, (((UWORD32)0x63)<<2));//dumy add for liusy test 0705  :: 07/11 rx inv enable
#ifdef TROUT_SDIO_INTERFACE
    //host_write_trout_reg(0x0A, (((UWORD32)0xc8)<<2));//liusy test 0904  :: rx 0 phase / tx async use 180 phase
	//chenq mod 2013-01-25 for BT context
 	host_write_trout_reg(0x4000000A, (((UWORD32)0xc8)<<2));//liusy test 0904  :: rx 0 phase / tx async use 180 phase
	host_write_trout_reg(0x02,(((UWORD32)0x4015)<<2)); //delete arm7 reset ?
#endif
#endif /*TROUT2_WIFI_FPGA_RF55*/

#ifdef TROUT2_WIFI_FPGA_RF2829
	host_write_trout_reg(0x00000010, (((UWORD32)0xc8)<<2));
#endif /*TROUT2_WIFI_FPGA_RF2829*/
#endif/*TROUT2_WIFI_FPGA*/

#ifdef TROUT2_WIFI_IC
#ifdef TROUT_SDIO_INTERFACE
    //host_write_trout_reg(0x0A, (((UWORD32)0xc8)<<2));//liusy test 0904  :: rx 0 phase / tx async use 180 phase
	//chenq mod 2013-01-25 for BT context
	host_write_trout_reg(0x02,(((UWORD32)0x4015)<<2)); //delete arm7 reset ?
#endif
#endif

#ifdef TROUT_WIFI_FPGA
//for FPGA-bit:trout_sdio_u2_ext40_tg1_newrf_1023.bit.
#ifndef TROUT_SDIO_INTERFACE
    host_write_trout_reg(0x02, (((UWORD32)0xc8)<<2));//liusy test 0904  :: rx 0 phase / tx async use 180 phase
#else
    host_write_trout_reg(0x02, (((UWORD32)0xc8)<<2));//liusy test 0904  :: rx / tx async use 180 phase
#endif /* TROUT_SDIO_INTERFACE */
#endif

#ifdef TROUT_WIFI_EVB
//for FPGA-bit:trout_sdio_u2_ext40_tg1_newrf_1023.bit.
#ifndef TROUT_SDIO_INTERFACE
    host_write_trout_reg(0x02, (((UWORD32)0xc8)<<2));//liusy test 0904  :: rx 0 phase / tx async use 180 phase
#else
    host_write_trout_reg(0x02, (((UWORD32)0xc8)<<2));//liusy test 0904  :: rx / tx async use 180 phase
#endif /* TROUT_SDIO_INTERFACE */
#endif


#ifdef IBSS_BSS_STATION_MODE

#ifndef TROUT_B2B_TEST_MODE
	//chenq add for auto scan
	//set_start_scan_req(1);
#endif

#endif

#ifdef BSS_ACCESS_POINT_MODE
	//chenq add for auto start
	//restart_mac(&g_mac,0);
#endif

    TROUT_DBG4("%s opened!\n", dev->name);

	//start_mem_adjust_task();
#ifdef TROUT_TRACE_DBG
    //start_trace_info_show_thread();
    //trace_func_init();
    
    //gpio_test_init();
#endif

	//chenq add 2012-10-18
	itm_mac_close1_open2 = 0;
	
    TROUT_FUNC_EXIT;
    return 0;
}
#else

#ifdef IBSS_BSS_STATION_MODE
extern UWORD32 g_nv_11b_tx_pwr[4];
extern UWORD32 g_nv_11g_tx_pwr[4];
extern UWORD32 g_nv_11n_tx_pwr[4];
extern UWORD32 g_0x57_delta;
/*
void set_tx_pwr_for_frame(void){    
    host_write_trout_reg((g_nv_11n_tx_pwr<< 16 | g_nv_11g_tx_pwr << 8 | g_nv_11b_tx_pwr), (UWORD32)rMAC_PHY_TX_PWR_SET_REG);
    host_write_trout_reg((host_read_trout_reg(rMAC_PROT_RATE) & MASK_INVERSE(8, 8)) | (g_nv_11g_tx_pwr << 8), (UWORD32)rMAC_PROT_RATE);
    host_write_trout_reg((host_read_trout_reg(rMAC_BEACON_TX_PARAMS) & MASK_INVERSE(8, 24)) | (g_nv_11g_tx_pwr << 24), (UWORD32)rMAC_BEACON_TX_PARAMS);
}
*/
#endif

int mac_open(struct net_device *dev)
{
    /* This fn is called, when this device is opened */
    //struct trout_private *tp = netdev_priv(dev);
    
	TROUT_FUNC_ENTER;

	//libing add for fix switch bugger.
	mutex_lock(&open_mutex);
	if(driver_exit == 1){
		mutex_unlock(&open_mutex);
		printk("%s:driver is unload ,return -1\n",__FUNCTION__);
		return -1;
	}

#ifdef BSS_ACCESS_POINT_MODE
	send_mac_status(MAC_AP_UP);
#endif

    /* Start the network interface queue for this device */
    if(!netif_queue_stopped(dev))
      	netif_start_queue(dev);
	else
		netif_wake_queue(dev);

#ifdef TROUT_B2B_TEST_MODE
    netif_stop_queue(dev);
    
    select_channel_rf(8, 0);        //set channel 9.
#endif

    start_mac_daemons();



#ifdef IBSS_BSS_STATION_MODE
	get_tx_pwr_from_nv(g_nv_11b_tx_pwr, g_nv_11g_tx_pwr, g_nv_11n_tx_pwr, &g_0x57_delta);
	printk("[%s] get g_nv_11b_tx_pwr = %x,%x,%x,%x\n", __FUNCTION__, g_nv_11b_tx_pwr[0], g_nv_11b_tx_pwr[1], g_nv_11b_tx_pwr[2],g_nv_11b_tx_pwr[3]);
	printk("[%s] get g_nv_11g_tx_pwr = %x,%x,%x,%x\n", __FUNCTION__, g_nv_11g_tx_pwr[0], g_nv_11g_tx_pwr[1], g_nv_11g_tx_pwr[2],g_nv_11g_tx_pwr[3]);
	printk("[%s] get g_nv_11n_tx_pwr = %x,%x,%x,%x\n", __FUNCTION__, g_nv_11n_tx_pwr[0], g_nv_11n_tx_pwr[1], g_nv_11n_tx_pwr[2],g_nv_11n_tx_pwr[3]);
	printk("[%s] get g_0x57_delta = %x\n", __FUNCTION__, g_0x57_delta);
      //  	set_tx_pwr_for_frame();
#endif

#ifdef TROUT_WIFI_NPI 
	restart_mac(&g_mac,0);
	set_mac_state(WAIT_SCAN);
	//set_start_scan_req(1);
#endif
	mutex_unlock(&open_mutex);
    TROUT_DBG4("%s opened!\n", dev->name);
	
    TROUT_FUNC_EXIT;
    return 0;
}
#endif


//add by chengwg.
static void mac_tx_timeout(struct net_device *dev)
{
	if(mac_int_task != NULL)
	{
		TROUT_DBG4("wake up wifi intterupt process func!\n");
		wake_up_process(mac_int_task);
	}
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : mac_xmit                                                 */
/*                                                                           */
/*  Description   : This function is called from the bridge/stack when a     */
/*                  packet needs to be transmitted.                          */
/*                                                                           */
/*  Inputs        : 1) Structure SK buffer                                   */
/*                  2) Network device driver structure                       */
/*                                                                           */
/*  Globals       : None                                                     */
/*  Processing    : None                                                     */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

int mac_xmit(struct sk_buff *skb, struct net_device *dev)
{
#ifdef ETHERNET_HOST
    UWORD8  *buffer = 0;
    UWORD16 offset  = 0;
    UWORD8 eth_hdr_offset = 0;
    UWORD8 *eth_hdr;
    UWORD16 eth_type = 0;
    UWORD8 q_num = 0;
    wlan_tx_req_t wlan_tx_req;
    
	TROUT_FUNC_ENTER;
	
   /* Stop the network interface queue */
    netif_stop_queue(dev);
   
    if(skb->dev != g_mac_dev)
    {
        TROUT_DBG2("Packet not destined to this device\n");
        TROUT_FUNC_EXIT;
        return 0;
    }

	pro_call_mac_xmit_count++;

#if 1	//add by chengwg, when txq full, memory copy pkt is not needed.
	//eth_hdr_offset = get_eth_hdr_offset(ETHERNET_HOST_TYPE);
	//eth_hdr = skb->data + eth_hdr_offset;
	eth_hdr = skb->data;
	
	eth_type = ((eth_hdr[ETH_PKT_TYPE_OFFSET] << 8) | eth_hdr[ETH_PKT_TYPE_OFFSET + 1]); 
	wlan_tx_req.priority = get_priority(eth_hdr, eth_type);

#ifdef IBSS_BSS_STATION_MODE
	if(change_priority_if(&wlan_tx_req) == BFALSE)
	{
		goto out;
	}
#endif	
	q_num = get_txq_num(wlan_tx_req.priority);
	
	//printk("mac_xmit: eth_type = 0x%x, priority = %d, q_num = %d\n", eth_type, wlan_tx_req.priority, q_num);

	if(is_this_txq_full_test(q_num))
	{
		//printk("txq %d full!\n", q_num);
		//goto out;
		return NETDEV_TX_BUSY;
	}
#endif

    // caisf add for debug, filter the specical mac 
#ifdef DV_SIM
    if(1)
    {
        UWORD8  spc_mac1[6] = { 0x33, 0x33, 0x00, 0x00, 0x00, 0xfd };
        UWORD8  spc_mac2[6] = { 0x01, 0x00, 0x5e, 0x00, 0x00, 0xfb };
        // check destination mac 
        if((memcmp(skb->data, spc_mac1, 5) == 0) || (memcmp(skb->data, spc_mac2, 5) == 0))
        {
            dev_kfree_skb(skb);
            netif_wake_queue(dev);
    	    printk("%s-%d: skb len: %d, xmit skip this mac pkt.\n", __FUNCTION__,__LINE__,skb->len);
            return -1;
        }
    }
#endif
    
    /* allocate memory from MAC Shared memory (DPSRAM) */
    buffer = pkt_mem_alloc(MEM_PRI_TX);/* This size is fixed */
    if(buffer == NULL)
    {
        /* If there are no buffers, free all the allocated memory and return */
        goto out;
    }

   /* Copy the data to shared memory */

    /* Check (unit test) which needs to be done for correct data endianness */
#ifdef USE_PROCESSOR_DMA
    /* Use DMA to copy the buffer */
    {
        UWORD32 skb_algn  = (UWORD32)(skb->data) & 0x1F;

        /* Check this should be modified for Multiple host interface */
        UWORD32 smem_algn = ((UWORD32)buffer +
                            get_eth_hdr_offset(ETHERNET_HOST_TYPE)) & 0x1F;

        /* The Offset in the shared memory is fixed such that the Shared Memory */
        /* Buffer alignment is same as the SKB alignment                        */
        offset = (skb_algn >= smem_algn)? (skb_algn - smem_algn) :
                                                     (skb_algn + 32 - smem_algn);
    }

    mem_dma(buffer + get_eth_hdr_offset(ETHERNET_HOST_TYPE) +
                        offset, skb->data, skb->len, LOCAL_TO_SHARED);
#else /* USE_PROCESSOR_DMA */
#ifdef SWAP_HOST_BYTE_ORDER
    memcpy32_le(buffer + get_eth_hdr_offset(ETHERNET_HOST_TYPE), skb->data, skb->len);
#else /* SWAP_HOST_BYTE_ORDER */
	memset(buffer, 0, skb->len + 34);
    memcpy(buffer + get_eth_hdr_offset(ETHERNET_HOST_TYPE), skb->data, skb->len);
#endif /* SWAP_HOST_BYTE_ORDER */
#endif /* USE_PROCESSOR_DMA */

    /* Call the funtion which sends the packet to MAC hardware */
#ifndef MAC_HW_UNIT_TEST_MODE
#ifdef DEBUG_MODE
	g_mac_stats.tx_data_frame_len += skb->len;
	g_mac_stats.tx_data_frame_count++;
#endif	/* DEBUG_MODE */
	//caisf add 2013-02-15
    if(g_mac_net_stats){
        g_mac_net_stats->tx_bytes += skb->len;
        g_mac_net_stats->tx_packets++;
    }

	if(receive_from_host(buffer, skb->len, offset, ETHERNET_HOST_TYPE) == -1)
	{
		return NETDEV_TX_BUSY;
    }
    
#else  /* MAC_HW_UNIT_TEST_MODE */
    {
        host_rx_event_msg_t host_rx = {0};

        host_rx.buffer_addr = buffer;
        host_rx.rx_pkt_len  = skb->len;
        host_rx.pkt_ofst    = offset;
        host_rx.host_if_type = ETHERNET_HOST_TYPE;
        process_host_rx_event((UWORD32)&host_rx);
    }
#endif /* MAC_HW_UNIT_TEST_MODE */

out:
    /* Free the SK Buffer, its work is done */
    dev_kfree_skb(skb);
    
    /* Wake up the network interface queue */
    netif_wake_queue(dev);

#endif /* ETHERNET_HOST */

	TROUT_FUNC_EXIT;
	//printk("[%s] mac have received tx data from stack, ready to transmit\n", __FUNCTION__);
    return NETDEV_TX_OK;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : mac_close                                                */
/*                                                                           */
/*  Description   : This is a dummy function and does nothing.               */
/*                                                                           */
/*  Inputs        : Network device driver structure                          */
/*                                                                           */
/*  Globals       : None                                                     */
/*  Processing    : None                                                     */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
#ifndef DV_SIM

/* this is a walk around slution, this can handle race condiction between reset_mac,
 * -start_mac_and_phy and  mac_driver_exit. by zhao
 */
void ward_mac_reset(mac_struct_t *mac)
{
	struct trout_private *tp;

	//xuan yang, 2013-8-23, add wid mutex lock
	tp = netdev_priv(g_mac_dev);
	mutex_lock(&tp->rst_wid_mutex);

	g_reset_mac_in_progress = BTRUE;
	reset_mac(mac, BTRUE);
	resolve_wid_conflicts(mac);
	g_reset_mac_in_progress = BFALSE;

	mutex_unlock(&tp->rst_wid_mutex);
}

int mac_close(struct net_device *dev)
{
#ifdef IBSS_BSS_STATION_MODE
	int mac_state;
#endif
    //struct trout_private *tp = netdev_priv(dev);

	TROUT_DBG4("trout close\n");
	//libing add for fix reboot bugger. when driver is removed ,app will mac_close sometimes
	if(mutex_trylock(&open_mutex) == 0 ){
		printk("%s:can't get open_mutex ,return -1\n",__FUNCTION__);
		return -1;
	}
	if(driver_exit == 1){
		mutex_unlock(&open_mutex);
		printk("%s:driver is unload ,return -1\n",__FUNCTION__);
		return 0;
	}
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifdef WIFI_SLEEP_POLICY
	if (mutex_is_locked(&suspend_mutex) || (g_wifi_suspend_status != wifi_suspend_nosuspend)) {
		pr_info("We can't do %s during suspending, g_wifi_suspend_status = %d\n", __func__, g_wifi_suspend_status);
		mutex_unlock(&open_mutex);
		return 0;
	}
#else

#if 0
	if (g_wifi_suspend_status == wifi_suspend_suspend) {
		pr_info("We can't do %s now: g_wifi_suspend_status = %d\n", __func__, g_wifi_suspend_status);\
		return 0;
	}
#else
	struct trout_private *stp;

	//xuan.yang, 2013-09-17, get run mutex lock
	stp = netdev_priv(dev);
    mutex_lock(&stp->cur_run_mutex);

	//check the suspend state
	if (g_wifi_suspend_status == wifi_suspend_suspend) {
		pr_info("We can't do %s now: g_wifi_suspend_status = %d\n", __func__, g_wifi_suspend_status);
		mutex_unlock(&stp->cur_run_mutex);
		mutex_unlock(&open_mutex);
		return 0;
	}

	//check the resume state
	if (0 == stp->resume_complete_state) {
		pr_info("We can't do %s now: g_wifi_suspend_status = %d\n", __func__, g_wifi_suspend_status);
		mutex_unlock(&stp->cur_run_mutex);
		mutex_unlock(&open_mutex);
		return 0;
	}
#endif
#endif
#endif

#ifdef DEBUG_MODE
	printk("%s", __FUNCTION__);
	print_symbol(" -> %s\n", (unsigned long)__builtin_return_address(0));
#endif
		
#ifdef BSS_ACCESS_POINT_MODE
	//send_mac_status(MAC_AP_DOWN);
	send_mac_status(MAC_DISCONNECTED);
#endif

    /* Stop the network interface queue */
	if( (dev != NULL) && (!netif_queue_stopped(dev)) )
	{
		netif_stop_queue(dev);
	}
	
    /* Reset the MAC hardware and software, PHY etc */
	//chenq mod
#ifdef IBSS_BSS_STATION_MODE
	restart_mac_plus(&g_mac, 0);
#else
	ward_mac_reset(&g_mac);  //by zhao
	//chenq add for post event to os 2013-05-13
	send_mac_status(MAC_CLOSE);
#endif

#ifdef IBSS_BSS_STATION_MODE	
    if(g_coex_trffc_chk.coex_traffic_check_alarm_handle != NULL)
    {
        stop_alarm(g_coex_trffc_chk.coex_traffic_check_alarm_handle);
        delete_alarm(&g_coex_trffc_chk.coex_traffic_check_alarm_handle);
    }
#endif

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifndef WIFI_SLEEP_POLICY
    mutex_unlock(&stp->cur_run_mutex);
#endif
#endif

	TROUT_DBG4("trout close end\n");
	mutex_unlock(&open_mutex);
    	return 0;
}

#else
int mac_close(struct net_device *dev)
{
    struct trout_private *tp = netdev_priv(dev);

    /* Stop the network interface queue */
    netif_stop_queue(dev);

	stop_mac_daemons();
	
    /* Reset the MAC hardware and software, PHY etc */
    reset_mac(&g_mac, BFALSE);
    flush_work(&tp->event_work);
    flush_workqueue(tp->event_wq);
    destroy_workqueue(tp->event_wq);

    disable_machw_phy_and_pa();

#ifdef LOCALMEM_TX_DSCR
    /* Release occupied local memory back to Linux */
    kfree(g_local_mem_dma);
#endif /* LOCALMEM_TX_DSCR */

#ifdef MAC_ADDRESS_FROM_FLASH
    iounmap((void *)g_virt_flash_base);
#endif

	//chenq add 20120903
	//itm_mac_close_flag = 1;
    return 0;
}
#endif

/************ chenq add 2012-11-14 *************/
extern int wpa_supplicant_ioctl(struct net_device *dev, struct iw_point *p);
extern int itm_hostapd_ioctl(struct iw_point *p);
/************ chenq add 2012-11-14 end *********/


/*****************************************************************************/
/*                                                                           */
/*  Function Name : mac_ioctl                                                */
/*                                                                           */
/*  Description   : This function is used to configure mac parameters.       */
/*                                                                           */
/*  Inputs        : 1) Network device driver structure                       */
/*                  2) Interface request structure                           */
/*                  3) Command                                               */
/*                                                                           */
/*  Globals       : None                                                     */
/*  Processing    : None                                                     */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

int mac_ioctl(struct net_device *dev, struct ifreq *req, int cmd)
{
    switch(cmd)
    {
 #if 0   
/* add by ke.li */
#ifdef DV_SIM
    case 0x8B99:
    {
        set_action_request((UWORD8 *)req->ifr_data);
    }
    break;
#endif
    case SIOCSIWESSID:
    {
    	// modified by zw
    	char essid[MAX_SSID_LEN + 1];
    	if (copy_from_user(essid, req->ifr_data, sizeof(essid)))
    	{
    		return -EFAULT;
    	}
    	//mset_DesiredSSID(req->ifr_data);
        mset_DesiredSSID(essid);

        /* Restart the MAC */
        restart_mac(&g_mac, 0);
    }
    break;
    case SIOCDEVPRIVATE:    /* Get/set data */
    {
#ifdef DEBUG_MODE
        UWORD32           *temp = 0;
        WORD32            i     = 0;
        mem_access_info_t mem_access_info;

        if(copy_from_user(&mem_access_info, req->ifr_data,
                          sizeof(mem_access_info_t)))
            return -EFAULT;

        /* Read/Write to the MAC H/w PA registers */
        if(mem_access_info.read_write == MAC_PA_READ_FLAG)
        {
            temp = (UWORD32 *)(PA_BASE + mem_access_info.addr);

            for(i = 0; i < mem_access_info.count; i++)
            {
                mem_access_info.data[i] = convert_to_le(*(temp + i));
            }
        }
        else if(mem_access_info.read_write == MAC_PA_WRITE_FLAG)
        {
            temp = (UWORD32 *)(PA_BASE + mem_access_info.addr);
            *temp = convert_to_le(mem_access_info.data[0]);
        }
        /* Read/Write to the MAC H/w CE registers */
        else if(mem_access_info.read_write == MAC_CE_READ_FLAG)
        {
            temp = (UWORD32 *)(CE_BASE + mem_access_info.addr);

            for(i = 0; i < mem_access_info.count; i++)
            {
                mem_access_info.data[i] = convert_to_le(*(temp + i));
            }
        }
        else if(mem_access_info.read_write == MAC_CE_WRITE_FLAG)
        {
            temp = (UWORD32 *)(CE_BASE + mem_access_info.addr);
            *temp = convert_to_le(mem_access_info.data[0]);
        }
        else if(mem_access_info.read_write == PHY_READ_FLAG)
        {
            for(i = 0; i < mem_access_info.count; i++)
            {
                read_dot11_phy_reg(((UWORD8)mem_access_info.addr + i),
                                   &mem_access_info.data[i]);
            }
        }
        else if(mem_access_info.read_write == PHY_WRITE_FLAG)
        {
            write_dot11_phy_reg((UWORD8)mem_access_info.addr,
                                mem_access_info.data[0]);
        }
        else if(mem_access_info.read_write == MAC_DEBUG_FLAG)
        {
            if(mem_access_info.count == 0)
            {
                if(g_enable_debug_print > 0)
                    g_enable_debug_print--;
                PRINTK("Current Debug Level = %d\n",g_enable_debug_print);
            }
            else if(mem_access_info.count == 1)
            {
                if(g_enable_debug_print < MAX_DEBUG_PRINT_LEVEL)
                    g_enable_debug_print++;
                PRINTK("Current Debug Level = %d\n",g_enable_debug_print);
            }
            else if(mem_access_info.count == 2)
                print_test_stats(0);
            else if(mem_access_info.count == 5)
                print_debug_stats();
#ifdef MAC_HW_UNIT_TEST_MODE
            else if(mem_access_info.count == 3)
                print_frm_stats();
            else if(mem_access_info.count == 4)
                reset_frm_stats();
#endif /* MAC_HW_UNIT_TEST_MODE */
            else if(mem_access_info.count == 6)
                print_build_flags();
            else if(mem_access_info.count == 7)
                print_ba_debug_stats();
        }

        if (copy_to_user(req->ifr_data, &mem_access_info,
                         sizeof(mem_access_info_t)))
            return -EFAULT;

#endif /* DEBUG_MODE */
    }
    break;

    case SIOCSIWFREQ:
    {
        /* If the value is wrong, don't do anything. */
        if(req->ifr_flags < 0 && req->ifr_flags > 14)
            break;

        /* Set the channel number required by MAC */
        mset_CurrentChannel((UWORD8)req->ifr_flags -1);
        set_mac_chnl_num((UWORD8)req->ifr_flags);

        /* Restart the MAC */
        restart_mac(&g_mac, 0);
    }
    break;

    case SIOCSIWRTS:
    {
        mset_RTSThreshold(req->ifr_flags);
        set_machw_rts_thresh(req->ifr_flags);
    }
    break;

    case SIOCSIWRATE:
    {
        set_tx_rate(req->ifr_flags);
    }
    break;
#endif
	/************ chenq add 2012-11-14 *************/
	case SIOCIWFIRSTPRIV + 28:
	{
		struct iwreq *wrq = (struct iwreq *)req;
		return itm_hostapd_ioctl(&wrq->u.data);
		 
	}
	break;
	
	case SIOCIWFIRSTPRIV + 30:
	{
		struct iwreq *wrq = (struct iwreq *)req;

		return -1;
	}
	break;
	/************ chenq add 2012-11-14 end***********/
#ifdef CONFIG_CFG80211
	/*** leon liu added android priv cmd processing**/
	case SIOCDEVPRIVATE + 1:
	{
		return trout_cfg80211_android_priv_cmd(dev, req);
	}
	break;
#endif /*CONFIG_CFG80211*/
	/*bing li added for NPI support*/
#ifdef TROUT_WIFI_NPI
	case NPI_CTL_CODE://=SIOCDEVPRIVATE + 5
	{
		return  npi_dispatch(req);
	}
	break;
#endif

    default:
    {
        /* Do Nothing */
        return -EINVAL;	//add by chengwg.
    }
    break;

    }
    return 0;
}

// caisf add for wifi statistics 0212
struct net_device_stats *mac_net_get_stats(struct net_device *dev)
{
    struct trout_private *tp = netdev_priv(dev);
    struct net_device_stats *mac_net_stats_ptr = &(tp->mac_net_stats);

	return mac_net_stats_ptr;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : send_packet_to_stack_lnx                                 */
/*                                                                           */
/*  Description   : This function is used to send the packet to stack.       */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*  Processing    : None                                                     */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

int send_packet_to_stack_lnx(void *msdu_desc)
{
    UWORD32        frame_len = 0;
    UWORD32        temp      = 0;
    msdu_desc_t    *fdesc    = (msdu_desc_t *)msdu_desc;
    struct sk_buff *skb      = 0;
    
	TROUT_FUNC_ENTER;
	
#ifdef USE_PROCESSOR_DMA
    UWORD32 skb_algn  = 0;
    UWORD32 smem_algn = 0;
#endif /* USE_PROCESSOR_DMA */

    /* Compute the length of the frame (including host interface header) */
    frame_len = fdesc->data_len + fdesc->host_hdr_len;

    /* Need to send the packet up to the host, allocate a skb buffer */
    skb = dev_alloc_skb(frame_len + 32);
    if(skb == NULL)
    {
        TROUT_RX_DBG2("Low memory - packet droped\n");
        TROUT_FUNC_EXIT;
        return 0;
    }

#ifdef USE_PROCESSOR_DMA
    /* The Offset in the shared memory is fixed such that the Shared Memory  */
    /* Buffer alignment is same as the SKB alignment                         */
    skb_algn  = ((UWORD32)skb->data) & 0x1F;
    smem_algn = ((UWORD32)fdesc->buffer_addr + fdesc->host_hdr_offset) & 0x1F;
    temp      = (smem_algn >= skb_algn)? (smem_algn - skb_algn) :
                                         (smem_algn + 32 - skb_algn);
#else /* USE_PROCESSOR_DMA */
    /* Alignment of skb buffer is adjusted to match the MSDU buffer */
    skb_reserve(skb, (-(UWORD32)skb->data) & 0x3);

    temp = ((UWORD32)(fdesc->buffer_addr + fdesc->host_hdr_offset + 2) & 0x3);
#endif /* USE_PROCESSOR_DMA */

    skb_reserve(skb, temp);
    skb->dev = g_mac_dev;

#ifdef USE_PROCESSOR_DMA
    mem_dma(skb_put(skb, frame_len),
            fdesc->buffer_addr + fdesc->host_hdr_offset, frame_len,
            SHARED_TO_LOCAL);
#else /* USE_PROCESSOR_DMA */
    memcpy(skb_put(skb, frame_len),
           fdesc->buffer_addr + fdesc->host_hdr_offset, frame_len);
#endif /* USE_PROCESSOR_DMA */

    skb->protocol = eth_type_trans(skb, g_mac_dev);

	TROUT_RX_DBG5("netif_rx, send len=%d\n", skb->len);
#ifdef DEBUG_MODE    
    g_mac_stats.rx_data_frame_len += skb->len;
    g_mac_stats.rx_data_frame_count++;
#endif
	//caisf add 2013-02-15
    if(g_mac_net_stats)
    {
        g_mac_net_stats->rx_bytes += skb->len;
        g_mac_net_stats->rx_packets++;
    }

    /* Send the packet to the stack by giving it to the bridge */
    netif_rx(skb);

	TROUT_FUNC_EXIT;
    return 0;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : csl_mem_init                                             */
/*                                                                           */
/*  Description   : This function initializes the memory for the chip support*/
/*                  library for eCos operating system.                       */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function initializes the memory for the ECOS        */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void csl_mem_init(void)
{
    /* Do Nothing */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : csl_init                                                 */
/*                                                                           */
/*  Description   : This is a dummy function and does nothing.               */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*  Processing    : None                                                     */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void csl_init(void)
{
    init_int_vec_globals();
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : csl_reset                                                */
/*                                                                           */
/*  Description   : This function resets the chip support library for eCos   */
/*                  operating system.                                        */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*  Processing    : None                                                     */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void csl_reset(void)
{
    /* Do Nothing */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : create_interrupt                                         */
/*                                                                           */
/*  Description   : This function creates an interrupt using Linux operating */
/*                  system.                                                  */
/*                                                                           */
/*  Inputs        : 1) Interrupt vector                                      */
/*                  2) Interrupt priority                                    */
/*                  3) Interrupt data                                        */
/*                  4) Interrupt service routine                             */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : Memory required for handles and objects are got from     */
/*                  the memory pool. The required Linux functions are called */
/*                  with the input parameters.                               */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : Pointer to the interrupt handle                          */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
#ifdef DV_SIM
extern int deIrqInfoRegister(unsigned int irq,
                          irq_handler_t handler,
                          unsigned long flags,
                          const char *name,
                          void *dev);

extern void unregister_virt_irq(unsigned int irqno, void *dev_id);
#endif
INTERRUPT_HANDLE_T *create_interrupt(INTERRUPT_VECTOR_T vec,
                                     INTERRUPT_PRIORITY_T pri,
                                     ADDRWORD_T data, ISR_T *isr)
{
    int ret;

	TROUT_FUNC_ENTER;
    if(g_mac_dev == NULL)
    {
        TROUT_DBG1("Err: g_mac_dev is NULL!\n");
        TROUT_FUNC_EXIT;
        return NULL;
    }
    
#ifndef DV_SIM    
	ret = request_threaded_irq(g_mac_dev->irq,
	                           NULL,
	                           (void *)mac_isr_work, 
	                           IRQ_TYPE_LEVEL_HIGH | IRQF_ONESHOT | IRQF_NO_SUSPEND,
	                           g_mac_dev->name, g_mac_dev);

#else
	ret = deIrqInfoRegister(g_mac_dev->irq, (void *)isr->isr, 0, g_mac_dev->name,
                            g_mac_dev);
#endif
    /* Set the device irq to the vector value */
    if(ret != 0)
    {
        /* Fatal Error, Interrupt can't be registered */
        printk("Fatal Err: Interrupt can't be registered(%d)\n", ret);
        TROUT_FUNC_EXIT;
        return 0;
    }
    else
    {
        TROUT_DBG4("WiFi Interrupt %d has been registered!\n", g_mac_dev->irq);
    }

	TROUT_FUNC_EXIT;
    return (INTERRUPT_HANDLE_T *)1;  //Hugh: fix Ittiam bug: return non-zero flag, to indicate isr has been installed.
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : delete_interrupt                                         */
/*                                                                           */
/*  Description   : This function deletes an interrupt using Linux operating */
/*                  system.                                                  */
/*                                                                           */
/*  Inputs        : 1) Pointer to the interrupt handle                       */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The given interrupt is deleted and the associated memory */
/*                  is released.                                             */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void delete_interrupt(INTERRUPT_HANDLE_T *handle)
{
    /* Require interrupt vector here. Currently hardcoded */
#ifndef DV_SIM
	TROUT_FUNC_ENTER;
	
    if(g_mac_dev != NULL)
        free_irq(g_mac_dev->irq, g_mac_dev);
#else
    if(g_mac_dev != NULL)
        unregister_virt_irq(g_mac_dev->irq, g_mac_dev);

    TROUT_FUNC_EXIT;
#endif
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : disable_interrupt                                        */
/*                                                                           */
/*  Description   : This function disables an interrupt using Linux operating*/
/*                  system.                                                  */
/*                                                                           */
/*  Inputs        : 1) Interrupt vector                                      */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The given interrupt is disabled.                         */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void disable_interrupt(VECTOR_T int_vec)
{
    /* Do nothing */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : enable_interrupt                                         */
/*                                                                           */
/*  Description   : This function enables an interrupt using Linux operating */
/*                  system.                                                  */
/*                                                                           */
/*  Inputs        : 1) Interrupt vector                                      */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The given interrupt is enabled.                          */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void enable_interrupt(VECTOR_T int_vec)
{
    /* Do nothing */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : acknowledge_interrupt                                    */
/*                                                                           */
/*  Description   : This function acknowledges an interrupt using Linux      */
/*                  operating system.                                        */
/*                                                                           */
/*  Inputs        : 1) Interrupt vector                                      */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The given interrupt is acknowledged.                     */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void acknowledge_interrupt(VECTOR_T int_vec)
{
    /* Do Nothing, kernel takes care of acknowledging the interrupt */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : create_alarm                                             */
/*                                                                           */
/*  Description   : This function creates an alarm using eCos operating      */
/*                  system.                                                  */
/*                                                                           */
/*  Inputs        : 1) Alarm function                                        */
/*                                                                           */
/*  Globals       : g_rtc_handle                                             */
/*                                                                           */
/*  Processing    : An alarm is created and initialized.                     */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : Pointer to an alarm handle                               */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
ALARM_HANDLE_T* create_alarm(ALARM_FUNC_T *func, ADDRWORD_T data, ALARM_FUNC_WORK_T *work_func)
{
    trout_timer_struct *trout_timer;
    struct timer_list *temp;
   
    trout_timer = (trout_timer_struct *)kmalloc(sizeof(trout_timer_struct), GFP_KERNEL);
	if(trout_timer == NULL)
	{
		printk(KERN_ERR "alloc struct timer failed!\n");
		return NULL;
	}

	//print_symbol("-> %s\n", (unsigned long)__builtin_return_address(0));
	
    /* Users are adviced to call this function rather changing the fields    */
    /* themselves, it initializes the timer structure to required values     */
    trout_timer->data = data;
	trout_timer->wid = 0xFFFFFFFF;
	ALARM_STATE_IN(trout_timer, ALARM_CREATE);
    
    temp = &trout_timer->timer;
    atomic_set(&trout_timer->used, 0);
    init_timer(temp);
    temp->function = (void (*)(unsigned long))func;
    
    if(work_func != NULL)
    {
        INIT_WORK(&trout_timer->work, work_func);
		trout_timer->work_proceed_flag = BFALSE;
        temp->data = (UWORD32)trout_timer;
    }
    else
    {
        trout_timer->work.func = NULL;
		trout_timer->work_proceed_flag = BTRUE;
        temp->data = data;
    }    
    
    return (void *)trout_timer;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : delete_alarm                                             */
/*                                                                           */
/*  Description   : This function deletes an alarm using Linux operating     */
/*                  system.                                                  */
/*                                                                           */
/*  Inputs        : 1) Pointer to the alarm handle                           */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : An alarm is deleted and associated memory is freed.      */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
static void _delete_alarm(ALARM_HANDLE_T** handle,int self)
{
      trout_timer_struct * trout_timer = (trout_timer_struct *)*handle;
	
      if(trout_timer != NULL)
      {
	int  wait_cnt = 0;
	int timeout = 0;
        
	/*Internal bug #42, jiangtao.yi*/
	while(atomic_read(&trout_timer->used) != 0)
	{
            /*deal with the waiting process*/
	    if(wait_cnt > 200)
	    {
	      /*print out the log for every 10ms in waiting time.*/
	      printk("%d:Waiting 0x%x for 10ms.\n", timeout, trout_timer);
              wait_cnt = 0;

	      /*This should not happen because the waiting time should not be too long(longer than 1s). If it happens, there muse be something abnormal.
                 For this case, just go out in order to avoid dead loop(CPU hang). But,this may cause kernel panic.*/
	      if(timeout++ > 100)
	      {
	        timeout = 0;
		atomic_set(&trout_timer->used, 0);
		break;
	      }
           }
           else
           {
	      udelay(50);
	      wait_cnt++;
	   }
	}
       /*deal with the work associate the time before del_timer*/
       atomic_inc(&trout_timer->used);
	   ALARM_STATE_IN(trout_timer, ALARM_DEL);
       if((trout_timer->work.func != NULL)&&(trout_timer->work_proceed_flag == BFALSE))
       {
          TROUT_DBG5("DBG: state = 0x%x, wid = 0x%x, current = 0x%x, work.state = 0x%x\n", 
						  trout_timer->state, trout_timer->wid, current, work_busy(&trout_timer->work));
          if((trout_timer->wid == (unsigned int)current) && (work_busy(&trout_timer->work) | WORK_BUSY_RUNNING))
          {
               TROUT_DBG5("DBG: del:\%pS cancel itself! ret = 0x%x\n", trout_timer->work.func, work_busy(&trout_timer->work));
          }
          else
          {
              if(trout_timer->work.data.counter&0x01)
              {
                TROUT_DBG5("Delete_alarm:--------work pending %d------------\n", trout_timer->work.data.counter);
              } 

          	/*toggle the flag before processing the work.*/
          	trout_timer->work_proceed_flag = BTRUE;
          	TROUT_DBG5("DBG:del:before cancel_work_sync \%pS\n", trout_timer->work.func);
		if(!self){
          	cancel_work_sync(&trout_timer->work);
		}
		TROUT_DBG5("DBG:del:after cancel_work_sync \%pS, wid = 0x%x, current = 0x%x, work.state = 0x%x\n", 
			     trout_timer->work.func, trout_timer->wid, current, work_busy(&trout_timer->work));
          }
       }

        /* Delete the timer handle */
	if(!self){
        if(del_timer_sync((struct timer_list *)*handle)<0)
        {
            print_symbol("DBG:del:can't delte timer sync:%s\n", (unsigned long)__builtin_return_address(0));
            printk("DBG: del:used = %d, flag = %d\n", atomic_read(&trout_timer->used), trout_timer->work_proceed_flag);
        }
	}else{
		 if(del_timer((struct timer_list *)*handle)<0)
        	{
            		print_symbol("DBG:del:can't delte timer:%s\n", (unsigned long)__builtin_return_address(0));
            		printk("DBG: del:used = %d, flag = %d\n", atomic_read(&trout_timer->used), trout_timer->work_proceed_flag);
        	}
	}
        atomic_dec(&trout_timer->used);	
       /*
        //masked by Hugh for bug of alarm delete itself.
        trout_timer = *handle;
        if(trout_timer->work.func != NULL)
        {
            flush_work(&trout_timer->work);
        }
        */
        
        /* Free the memory of the handle */
        kfree(*handle);

       *handle = NULL;
    }
}

void delete_alarm(ALARM_HANDLE_T **handle)
{
    _delete_alarm(handle,0);
}

void delete_alarm_self(ALARM_HANDLE_T **handle)
{
    _delete_alarm(handle,1);
}
//End:add by wulei 2791 for bug 160423 on 2013-05-04
/*****************************************************************************/
/*                                                                           */
/*  Function Name : start_alarm                                              */
/*                                                                           */
/*  Description   : This function starts the 'specified' alarm for a         */
/*                  duration 'timeout_ms' specified by the user.             */
/*                  Completion function is called after the alarm expires.   */
/*                                                                           */
/*  Inputs        : 1) Pointer to the alarm handle                           */
/*                  2) Timeout in miliseconds                                */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The given alarm is started with the specified timeout    */
/*                  value in miliseconds by first converting it to the       */
/*                  equivalent schedular ticks.                              */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void start_alarm(ALARM_HANDLE_T *handle, UWORD32 timeout_ms)
{
#if 0
    /* Offset the jiffies by timeout_ms milliseconds */
    ((struct timer_list *)handle)->expires = jiffies + ((timeout_ms * HZ)/1024);

    /* Add timer to the list */
    add_timer((struct timer_list *)handle);
#else
	if((handle == NULL) ||  reset_mac_trylock() == 0)	//caisf fix bug.
	{
		return;
	}
	else
	{
	  trout_timer_struct *trout_timer = (trout_timer_struct *)handle;
	  atomic_inc(&trout_timer->used);
	  ALARM_STATE_IN(trout_timer, ALARM_ADD);
	  if(trout_timer->work.func != NULL)
            trout_timer->work_proceed_flag = BFALSE;
	  else
            trout_timer->work_proceed_flag = BTRUE;
          if (handle == &g_mgmt_alarm_handle){
		print_symbol("DBG:mgmt-call-%s\n", (unsigned long)__builtin_return_address(1));
		print_symbol("DBG:mgmt-call-%s\n", (unsigned long)__builtin_return_address(2));
	}
          mod_timer((struct timer_list *)handle, jiffies+((timeout_ms * HZ)/1000)); 
	  atomic_dec(&trout_timer->used);
	}
	 reset_mac_unlock();
#endif
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : stop_alarm                                               */
/*                                                                           */
/*  Description   : This function stops the 'specified' alarm.               */
/*                                                                           */
/*  Inputs        : 1) Pointer to the alarm handle                           */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The given alarm is stopped.                              */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

static void _stop_alarm(ALARM_HANDLE_T *handle,int self)
{
	trout_timer_struct *trout_timer = (trout_timer_struct *)handle;

    if(handle != NULL)
    {
        atomic_inc(&trout_timer->used);
		ALARM_STATE_IN(trout_timer, ALARM_STOP);
	    if((trout_timer->work.func != NULL) && (trout_timer->work_proceed_flag == BFALSE))
	    {
            TROUT_DBG5("DBG: state = 0x%x, wid = 0x%x, current = 0x%x, work.state = 0x%x\n", 
						trout_timer->state, trout_timer->wid, current, work_busy(&trout_timer->work));
            if((trout_timer->wid == (unsigned int)current) && (work_busy(&trout_timer->work) | WORK_BUSY_RUNNING))
            {
                TROUT_DBG5("DBG: stop:\%pS cancel itself! ret = 0x%x\n", trout_timer->work.func, work_busy(&trout_timer->work));
            }
            else
            {
	            /*toggle the flag before processing the work.*/
                if(trout_timer->work.data.counter&0x01)
                {
                     TROUT_DBG5("Delete_alarm:--------work pending %d------------\n", trout_timer->work.data.counter);
                } 
	            trout_timer->work_proceed_flag = BTRUE;
	            TROUT_DBG5("DBG:stop:before cancel_work_sync \%pS\n", trout_timer->work.func);
                cancel_work_sync(&trout_timer->work);
		   TROUT_DBG5("DBG:stop:after cancel_work_sync \%pS, wid = 0x%x, current = 0x%x, work.state = 0x%x\n", 
			     trout_timer->work.func, trout_timer->wid, current, work_busy(&trout_timer->work));
		    }
         }

        /* Delete the timer handle */
	if(!self){
        if(del_timer_sync((struct timer_list *)handle)<0)
        {
            print_symbol("DBG:stop:can't delte timer sync:%s\n", (unsigned long)__builtin_return_address(0));
            printk("DBG: stop:used = %d, flag = %d\n", atomic_read(&trout_timer->used), trout_timer->work_proceed_flag);
        }
	}else{
		  if(del_timer((struct timer_list *)handle)<0)
        	{
            		print_symbol("DBG:stop:can't delte timer:%s\n", (unsigned long)__builtin_return_address(0));
            		printk("DBG: stop:used = %d, flag = %d\n", atomic_read(&trout_timer->used), trout_timer->work_proceed_flag);
        	}
	}
       
	    atomic_dec(&trout_timer->used);
        /*
        //masked by Hugh for bug of alarm stop itself.
        trout_timer = handle;
        if(trout_timer->work.func != NULL)
        {
            flush_work(&trout_timer->work);
        }*/
    }
}

void stop_alarm(ALARM_HANDLE_T *handle)
{
    _stop_alarm(handle,0);
}

void stop_alarm_self(ALARM_HANDLE_T *handle)
{
    _stop_alarm(handle,1);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : print_banner_lnx                                         */
/*                                                                           */
/*  Description   : This function prints IP related Information on the       */
/*                  serial console                                           */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*  Processing    : None                                                     */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void print_banner_lnx(void)
{
    PRINTK("------------------------------------\n");
    PRINTK("IP version information\n");
    PRINTK("------------------------------------\n");
    PRINTK("MAC Hardware version : %s\n", get_hardwareProductVersion() + 1);
    read_phy_version();
    PRINTK("PHY Hardware version : %s\n", get_phyProductVersion() + 1);
    PRINTK("MAC Firmware version : " TROUT_WIFI_VERSION "\n");
    PRINTK("\n");
#ifndef TROUT_SDIO_INTERFACE
	print_spi_info();
#endif
	PRINTK("Mac auto rate: ");
#ifdef AUTORATE_FEATURE
	PRINTK("enable\n");
#else
	PRINTK("disable\n");
#endif
    PRINTK("Creation date: " MAC_CREATE_DATE "\n");
#ifdef TROUT_WIFI_NPI
    PRINTK("TROUT_WIFI_NPI Driver\n");
#endif
    PRINTK("------------------------------------\n");
}

#ifdef USE_PROCESSOR_DMA

/*****************************************************************************/
/*                                                                           */
/*  Function Name : mem_dma                                                  */
/*                                                                           */
/*  Description   : This function does a DMA transfer between the source and */
/*                  destination. For MWLAN, this function performs best when */
/*                  the source and destination buffers have the same         */
/*                  alignment over a 32 Byte window.                         */
/*                                                                           */
/*  Inputs        : dstv - Destination Address                               */
/*                  srcv - Source Address                                    */
/*                  len  - Length of the buffer                              */
/*  Globals       : None                                                     */
/*  Processing    : None                                                     */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 mem_dma(void *dstv, void *srcv, UWORD32 len, DMA_DIRECTION_T dir)
{
    void   *srcp = NULL;
    void   *dstp = NULL;
    UWORD8 retval = 0;
    UWORD8 salign = (UWORD32)srcv & 0x1F;
    UWORD8 dalign = (UWORD32)dstv & 0x1F;

    /* Align the buffers to 32 byte boundary */
    /* Check whether any optimization is possible. */
    if((salign != 0) && (salign == dalign))
    {
        UWORD8 tlen = MIN(32 - salign, len);

        memcpy(dstv, srcv, tlen);

        if(tlen == len)
            return retval;

        len -= tlen;
        srcv = (void *)((UWORD32)srcv + tlen);
        dstv = (void *)((UWORD32)dstv + tlen);
    }

    /* Compute the Physical addresses of the buffers. */
    switch(dir)
    {
    case SHARED_TO_LOCAL:
        srcp = (void *)virt_to_phy_addr((UWORD32)srcv);
        dstp = (void *)__pa((UWORD32)dstv);
        break;
    case LOCAL_TO_SHARED:
        srcp = (void *)__pa((UWORD32)srcv);
        dstp = (void *)virt_to_phy_addr((UWORD32)dstv);
        break;
    case LOCAL_TO_LOCAL:
        srcp = (void *)__pa((UWORD32)srcv);
        dstp = (void *)__pa((UWORD32)dstv);
        break;
    default: /* SHARED_TO_SHARED */
        srcp = (void *)virt_to_phy_addr((UWORD32)srcv);
        dstp = (void *)virt_to_phy_addr((UWORD32)dstv);
        break;
    }

    /* Actual DMA done here */
    retval = do_proc_dma(dstp, srcp, len);

    return retval;
}

#endif /* USE_PROCESSOR_DMA */


//Hugh: add sysfs for debug.
static ssize_t trout_show(struct device *dev,
                              struct device_attribute *attr,
			                  char *buf)
{
    struct trout_private *tp = dev_get_drvdata(dev);    

    if(tp == NULL)
    {
        printk("trout_show: no such device.\n");
        return 0;
    }

	return snprintf(buf, PAGE_SIZE, "rx pointer reg[0x%x]=0x%x\n", rMAC_RX_BUFF_ADDR, host_read_trout_reg(rMAC_RX_BUFF_ADDR));
}

extern void tx_complete_isr(void);
#if 0
static ssize_t trout_store(struct device *dev,
                                            struct device_attribute *attr,
                                            const char *buf, size_t count)
{
    UWORD32 addr, temp32;
    char *ptr, *bufcp;

    bufcp = kzalloc(count+1, GFP_KERNEL);
    mem_using_count_inc();
    
    memcpy(bufcp, buf, count);
    bufcp[count] = 0;
    printk("trout_store[%u]: %s\n", count, bufcp);

    if(memcmp(bufcp, "regrd ", 6) == 0)
    {
        ptr = bufcp + 6;
        addr = (UWORD32)simple_strtoul(ptr, NULL, 0);
        printk("regrd[0x%x]=0x%x.\n", addr, host_read_trout_reg(addr));
    }
    else if(memcmp(bufcp, "regwt ", 6) == 0)
    {
        ptr = bufcp + 6;
        addr = (UWORD32)simple_strtoul(ptr, &ptr, 0);
        ptr++;
        temp32 = (UWORD32)simple_strtoul(ptr, &ptr, 0);
       
        printk("regwt[0x%x]=0x%x, ret=%d.\n", addr, temp32, host_write_trout_reg(temp32, addr));
    }
    else if(memcmp(bufcp, "txdbg0", 6) == 0)
    {
    	switch_flag = 0;
        printk("switch_flag = 0.\n");
    }
    else if(memcmp(bufcp, "reset_init_phyrf", 16) == 0)
    {
        printk("reset PHY, init PHY and rf...\n");
        reset_phy();
        initialize_phy();
        initialize_rf();
        printk("done.\n");
    }
    else if(memcmp(bufcp, "reset_init_phy", 14) == 0)
    {
        printk("reset and init PHY...\n");
        reset_phy();
        initialize_phy();
        printk("done.\n");
    }
    else if(memcmp(bufcp, "init_phyrf", 10) == 0)
    {
        printk("init PHY and RF...\n");
        initialize_phy();
        initialize_rf();
        printk("done.\n");
    }
    else if(memcmp(bufcp, "init_phy", 8) == 0)
    {
        printk("init PHY...\n");
        initialize_phy();
        printk("done.\n");
    }
    else if(memcmp(bufcp, "init_rf", 7) == 0)
    {
        printk("init RF...\n");
        initialize_rf();
        printk("done.\n");
    }
    else if(memcmp(bufcp, "restart_mac", 11) == 0)
    {
        printk("restart mac...\n");
        restart_mac(&g_mac, 0);
        printk("done.\n");
    }
    else if(memcmp(bufcp, "reinit_rxq", 10) == 0)
    {
        printk("reinit rxq queue...\n");
        reinit_mac_rxq();
        printk("done.\n");
    }
    else if(memcmp(bufcp, "irq_en", 6) == 0)
    {
        printk("enable irq...\n");
        if(g_mac_dev != NULL)
        {
            enable_irq(g_mac_dev->irq);
            printk("done.\n");
        }
        else
        {
            printk("fail: no such device!\n");
        }
    }
    else if(memcmp(bufcp, "irq_de", 6) == 0)
    {
        printk("disable irq...\n");
        if(g_mac_dev != NULL)
        {
            disable_irq_nosync(g_mac_dev->irq);
            printk("done.\n");
        }
        else
        {
            printk("fail: no such device!\n");
        }
    }
    else if(memcmp(bufcp, "reinit_mem", 10) == 0)
    {
        printk("reinit mem and queue...\n");
        initialize_macsw(&g_mac);
        printk("done.\n");
    }
#ifdef TROUT_TRACE_DBG        
    else if(memcmp(bufcp, "rx_mem_show", 11) == 0)
    {
        printk("rx pointer[0x%x]: 0x%x\n", rMAC_RX_BUFF_ADDR, host_read_trout_reg(rMAC_RX_BUFF_ADDR));
        printk("rx frame  [0x%x]: 0x%x\n", rMAC_RX_FRAME_POINTER, host_read_trout_reg(rMAC_RX_FRAME_POINTER));
        
        printk("rx sharedmem show...\n");
        rx_share_ram_show();
        printk("done.\n");
    }
    else if(memcmp(bufcp, "trace_dbg_mask_set ", sizeof("trace_dbg_mask_set ")-1) == 0)
    {
		ptr = bufcp + sizeof("trace_dbg_mask_set ")-1;
        addr = (UWORD32)simple_strtoul(ptr, NULL, 0);
        trout_dbg_mask_set(addr);
        printk("after done, trout_dbg_mask = 0x%08x\n", trout_dbg_mask_get());
    }
    else if(memcmp(bufcp, "trace_dbg_mask_clear ", sizeof("trace_dbg_mask_clear ")-1) == 0)
    {
		ptr = bufcp + sizeof("trace_dbg_mask_clear ")-1;
        addr = (UWORD32)simple_strtoul(ptr, NULL, 0);
		trout_dbg_mask_clear(addr);
        printk("after done, trout_dbg_mask = 0x%08x\n", trout_dbg_mask_get());
    }
    else if(memcmp(bufcp, "trace_dbg_mask_get", sizeof("trace_dbg_mask_get")-1) == 0)
    {
		printk("trout_dbg_mask = 0x%08x\n", trout_dbg_mask_get());
    }
    else if(memcmp(bufcp, "print_count_info", sizeof("print_count_info")-1) == 0)
    {
		print_count_info();
    }
    else if(memcmp(bufcp, "clean_count_info", sizeof("clean_count_info")-1) == 0)
    {
		clean_count_info();
    }
    else if(memcmp(bufcp, "clean_tx_int", sizeof("clean_tx_int")-1) == 0)
    {
		printk("clear tx int status!\n");
    	printk("before clean,int status: 0x%08X\n", convert_to_le(host_read_trout_reg((UWORD32)rCOMM_INT_STAT)) >> 2);
		get_machw_tx_frame_pointer();
		reset_machw_tx_comp_int();
		printk("after clean,int status: 0x%08X\n", convert_to_le(host_read_trout_reg((UWORD32)rCOMM_INT_STAT)) >> 2);
    }
    else if(memcmp(bufcp, "clean_all_int", sizeof("clean_all_int")-1) == 0)
    {
		UWORD32 int_stat;
		int_stat = convert_to_le(host_read_trout_reg((UWORD32)rCOMM_INT_STAT)) >> 2;
		printk("before clean, int_stat = 0x%x\n", int_stat);

		//read for clear tx, rx int.
		host_read_trout_reg((UWORD32)rMAC_TX_FRAME_POINTER);
		host_read_trout_reg((UWORD32)rMAC_HIP_RX_FRAME_POINTER);
		host_read_trout_reg((UWORD32)rMAC_RX_FRAME_POINTER);
		
		host_write_trout_reg((int_stat << 2), (UWORD32)rCOMM_INT_CLEAR);

		int_stat = convert_to_le(host_read_trout_reg((UWORD32)rCOMM_INT_STAT)) >> 2;
		printk("after clean, int_stat = 0x%x\n", int_stat);
    }
    else if(memcmp(bufcp, "tx_send_pkt", sizeof("tx_send_pkt")-1) == 0)
    {
		printk("transact pkt, then tx send pkt!\n");
		tx_complete_isr();
    }
#endif    
    else if(memcmp(bufcp, "reinit_irq", 10) == 0)
    {
        printk("delete irq.\n");
        delete_mac_interrupts();

        printk("create irq.\n");
        create_mac_interrupts();
        
        printk("done.\n");
    }
    else if(memcmp(bufcp, "slot_info", 9) == 0)
    {
        extern UWORD32 slot_pkt_nr[];
        extern UWORD32 slot_mount_nr[];
        
        printk("slot_info:\n");
        printk("0:Tx_pkts = %u\n", slot_pkt_nr[0]);        
        printk("0:mount_nr = %u\n", slot_mount_nr[0]);        
        printk("1:Tx_pkts = %u\n", slot_pkt_nr[1]);        
        printk("1:mount_nr = %u\n", slot_mount_nr[1]);        

        printk("done.\n");
    }

    
    else
    {
        printk("bad cmd!\n");
    }

    kfree(bufcp);
    mem_using_count_dec();
    
    return count;
}
#else
#define trout_show NULL
#define trout_store NULL
#endif

static struct device_attribute trout_sys_dbg = __ATTR(trout, S_IRUGO|S_IWUGO, trout_show, trout_store);

#ifdef TROUT_B2B_TEST_MODE

int b2b_rx_pkt_add(b2b_rx_pkt_s *rx_pkt)
{
    struct trout_private *tp = netdev_priv(g_mac_dev);
    struct b2b_test_qm *qm = &tp->b2b_qm;

	mutex_lock(&qm->qm_lock);
	
	list_add_tail(&rx_pkt->pkt_node, &qm->rx_pkt_list);
    qm->rx_list_pkt_cnt++;
    
    //wakeup upper level rx process.
    wake_up_interruptible(&qm->rx_wait_queue);
    
	mutex_unlock(&qm->qm_lock);

    return 0;
}

int b2b_list_reinit(void)
{
    struct trout_private *tp = netdev_priv(g_mac_dev);
    struct b2b_test_qm *qm = &tp->b2b_qm;
    b2b_rx_pkt_s *b2b_rx = NULL;

    list_for_each_entry(b2b_rx, &qm->rx_pkt_list, pkt_node)
    {
        if(b2b_rx != NULL)
            kfree(b2b_rx);

        b2b_rx = NULL;
    }

    
    memset(qm, 0, sizeof(qm));
    mutex_init(&qm->qm_lock);
    init_waitqueue_head(&qm->rx_wait_queue);    
    
	INIT_LIST_HEAD(&qm->rx_pkt_list);
	INIT_LIST_HEAD(&qm->tx_pkt_list);
    
    return 0;
}

EXPORT_SYMBOL(b2b_list_reinit);

int trout_b2b_pkt_to_host(__user char *buf, int buf_len, int timeout)
{
    struct trout_private *tp = netdev_priv(g_mac_dev);
    struct b2b_test_qm *qm = &tp->b2b_qm;
    b2b_rx_pkt_s *b2b_rx = NULL;
    int len = 0;
    long rc = 0;

//    printk("wait event.\n");
    if(timeout != -1)
    {  
        rc = wait_event_interruptible_timeout(qm->rx_wait_queue, qm->rx_list_pkt_cnt, HZ*timeout);
        if (rc == 0)
            return -ETIMEDOUT;
        else if (rc < 0)
            return rc;
    }
    else        
        wait_event_interruptible(qm->rx_wait_queue, qm->rx_list_pkt_cnt);

	mutex_lock(&qm->qm_lock);
	
    b2b_rx = list_first_entry(&qm->rx_pkt_list, b2b_rx_pkt_s, pkt_node);
    if(b2b_rx->pkt == NULL || b2b_rx->len < 0)
    {
        printk("bad rx pkt.\n");
        len = 0;
    }
    else
    {
        len = b2b_rx->len < buf_len ? b2b_rx->len : buf_len;
        if(copy_to_user(buf, b2b_rx->pkt, len)) 
        {
            printk("rx: copy_to_user fail.\n");
            len = -EFAULT;
        }
    }
    
    list_del(&b2b_rx->pkt_node);

    if(qm->rx_list_pkt_cnt > 0)
        qm->rx_list_pkt_cnt--;
    
    qm->rx_cnt++;

    kfree(b2b_rx);
    
	mutex_unlock(&qm->qm_lock);

    return len;
}

EXPORT_SYMBOL(trout_b2b_pkt_to_host);

static void b2b_init(struct net_device *dev)
{        
    struct trout_private *tp = netdev_priv(dev);
    struct b2b_test_qm *qm = &tp->b2b_qm;
    
    memset(qm, 0, sizeof(qm));
    mutex_init(&qm->qm_lock);
    init_waitqueue_head(&qm->rx_wait_queue);    
    
	INIT_LIST_HEAD(&qm->rx_pkt_list);
	INIT_LIST_HEAD(&qm->tx_pkt_list);
}

#endif

#ifdef TROUT_TRACE_DBG
void sprd_timer2_init(void)
{
/*leon liu stripped timer2 settings*/
#if 0
    /* enable timer register access */
	__raw_bits_or((1 << 2), GREG_GEN0);
	
	/* stop timer2 */
	__raw_writel(0, TIMER2_CONTROL);
	__raw_writel((1<<3), TIMER2_CLEAR);

    /* set load value */
	__raw_writel(TIMER2_LOAD_VALUE, TIMER2_LOAD);

	/* enable timer2 */
    __raw_bits_or((1 << 6), TIMER2_CONTROL);
    __raw_bits_or((1 << 7), TIMER2_CONTROL);
#endif
}


void sprd_timer2_destroy(void)
{
/*leon liu stripped timer2 settings*/
#if 0
	__raw_writel(0, TIMER2_CONTROL);
	__raw_writel((1<<3), TIMER2_CLEAR);
#endif
}
#endif	/* TROUT_TRACE_DBG */

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#if 0
static void trout_wifi_early_suspend(struct early_suspend *e_s)
{
	printk("##:trout_wifi_early_suspend start...##\n");

	/*  disable power  */
	//itm_set_power_type((UWORD8)( Maximum_PSPOLL_Power_Save_Mode & 0xFF));//change powersave mode to Minimum_FAST_Power_Save_Mode
	itm_set_power_type((UWORD8)( Minimum_FAST_Power_Save_Mode & 0xFF));
}

extern unsigned int host2arm_irq_cnt;
static void trout_wifi_late_resume(struct early_suspend *e_s)
{
	printk("##:trout_wifi_late_resume start...##\n");
	
	/*  enable power  */
        if(g_wifi_suspend_status == wifi_suspend_early_suspend) // early sleep
	{
		printk("g_wifi_suspend_status = %d\n",g_wifi_suspend_status);
		itm_set_power_type((UWORD8)(Active_Mode & 0xFF)) ;
		//leon liu added on 2013-4-5, start powersave_timer again
		pstimer_start_late_resume(&pstimer);
	}
	if(g_wifi_suspend_status == wifi_suspend_suspend) // deep sleep
	{
		/*notify_cp_with_handshake(PS_MSG_WIFI_RESUME_MAGIC, 3);*/
		/*restart_mac_plus(&g_mac, 0);*/
		sta_wakeup();
	}
}
struct early_suspend trout_wifi_early_suspend_handler = {
	.level = 149,
	.suspend = trout_wifi_early_suspend,
	.resume = trout_wifi_late_resume,
};
#endif
#endif

#ifdef TROUT_SDIO_INTERFACE
/*leon liu added for compiling on Android 4.x*/
#if 0 
struct sdio_trout_data *sdio_trout_dev_get(void)
{
    struct trout_private *stp;

    if(g_mac_dev == NULL)
        return NULL;
    
    stp = netdev_priv(g_mac_dev);
    
    return &(stp->sdio_data);
}

static int sdio_trout_probe(struct sdio_func *func,
			   const struct sdio_device_id *id)
{
    int err_ret;
    int rc;
    struct net_device *netdev;
    struct trout_private *stp;

    //unsigned long gpio_cfg = MFP_CFG_X(GPIO136, AF0, DS1, F_PULL_NONE, S_PULL_NONE, IO_IE);

#ifdef TROUT_TRACE_DBG
    sprd_timer2_init();

#ifdef TRACE_LOG_TO_FILE
    //chenq add 0912
    if(trout_trace_init(0,0,0) != 0)
    {
        printk("unable to inti trace \n");
        TROUT_FUNC_EXIT;
        return -1;
    }	
#endif	/* TRACE_LOG_TO_FILE */
    //trace_func_init();
#endif

    TROUT_FUNC_ENTER;
    sdio_claim_host(func);
    err_ret = sdio_enable_func(func);
    sdio_set_block_size(func,512);	
    sdio_release_host(func);

    netdev = alloc_etherdev(sizeof(*stp));
    if(!netdev) 
    {
        TROUT_DBG1("unable to alloc new ethernet\n");
        TROUT_FUNC_EXIT;
        return -ENOMEM;
    }

    //chenq add 0723
    g_itm_config_buf = (unsigned char *)kmalloc(0x7FF, GFP_KERNEL);
    if(!g_itm_config_buf)
    {
        TROUT_DBG1("unable to alloc new g_itm_config_buf!\n");
        free_netdev(netdev);
        TROUT_FUNC_EXIT;
        return -ENOMEM;
    }

    SET_NETDEV_DEV(netdev, &(func->dev));
    stp = netdev_priv(netdev);
    stp->dev = netdev;

    netdev->netdev_ops = &trout_netdev_ops;

    //chenq add
    netdev->wireless_handlers = &itm_iw_handler_def;
#ifdef IBSS_BSS_STATION_MODE
    strncpy(netdev->name, "wlan%d", sizeof(netdev->name));
#else
    strncpy(netdev->name, "wlap%d", sizeof(netdev->name));
#endif

    netdev->name[sizeof(netdev->name) - 1] = '\0';
    netdev->watchdog_timeo = msecs_to_jiffies(500);	//timeout, add by chengwg!

    stp->sdio_data.func = func;
    sdio_trout_init(&(stp->sdio_data));
    dev_set_drvdata(&func->dev, stp);

    if(sprd_3rdparty_gpio_wifi_irq>0)
    {
        gpio_free(sprd_3rdparty_gpio_wifi_irq);

        //sprd_mfp_config(&gpio_cfg, 1);
        rc =gpio_request(sprd_3rdparty_gpio_wifi_irq, "trout int");
        if(rc){
            TROUT_DBG1("cannot alloc gpio for trout irq\n");
            goto err_out0;
        }

        gpio_direction_input(sprd_3rdparty_gpio_wifi_irq);
        rc = sprd_alloc_gpio_irq(sprd_3rdparty_gpio_wifi_irq);
        if(rc < 0){
            TROUT_DBG1("cannot alloc gpio for trout irq\n");
            gpio_free(sprd_3rdparty_gpio_wifi_irq);
            goto err_out0;
        }
    }
    else
    {
        printk("trout:sprd_3rdparty_gpio_wifi_irq[0x%x]\n",sprd_3rdparty_gpio_wifi_irq);
        goto err_out0;
    }

    netdev->irq = rc;       //get irq number.
    g_mac_dev = netdev;
    //caisf add 2013-02-15
    g_mac_net_stats = &stp->mac_net_stats;

#ifdef TROUT_B2B_TEST_MODE
    b2b_init(netdev);
#endif

    //caisf add 2013-02-15
    mac_net_get_stats(g_mac_dev);

    rc = mac_init_fn(netdev);
    if(rc < 0){
        TROUT_DBG1("mac_init_fn() fail!\n");
        goto err_out1;
    }

    rc = register_netdev(netdev);
    if(rc == 0){
        TROUT_DBG4("%s: register netdev success!\n", __func__);
        rc  = device_create_file(&func->dev, &trout_sys_dbg);
        if (rc != 0)
        {
            TROUT_DBG3("warning: trout system debug create failed.\n");
        }

        TROUT_DBG4("%s: return ok.\n\r\n", __func__);

        itm_mac_close1_open2 = 2;
        mac_open(netdev);

        TROUT_FUNC_EXIT;
        return 0;
    }

err_out1:
    sprd_free_gpio_irq(netdev->irq);
    if(sprd_3rdparty_gpio_wifi_irq>0)
    {
        gpio_free(sprd_3rdparty_gpio_wifi_irq);
    }

err_out0:
    free_netdev(netdev);
    g_mac_dev = NULL;
    g_mac_net_stats = NULL;
    kfree(g_itm_config_buf);
    g_itm_config_buf = NULL;

    TROUT_FUNC_EXIT;
    return rc;
}

static void __devexit sdio_trout_remove(struct sdio_func *func)
{
	struct net_device *netdev = NULL;
	struct trout_private *stp = dev_get_drvdata(&(func->dev));

	TROUT_FUNC_ENTER;
	if((stp == NULL) || ((netdev = stp->dev) == NULL))
	{
		TROUT_DBG1("%s: error! tp(%p) or netdev(%p) is NULL!\n", __func__, stp, netdev);
		return;
	}

	//chenq add 2012-10-18
	itm_mac_close1_open2 = 1;
	mac_close(netdev);

	//chenq add
	/* Clean up netdev. */
	unregister_netdev(netdev);
	free_mem_regions();
	sprd_free_gpio_irq(netdev->irq);
        if(sprd_3rdparty_gpio_wifi_irq>0)
        {
            gpio_free(sprd_3rdparty_gpio_wifi_irq);
        }
	free_netdev(netdev);	//add by chengwg.
	g_mac_dev = NULL;
	//caisf add 2013-02-15
    g_mac_net_stats = NULL;

    device_remove_file(&func->dev, &trout_sys_dbg);

	//chenq mask
	/* Clean up netdev. */
	//unregister_netdev(netdev);
	
	sdio_claim_host(func);
	sdio_disable_func(func);
	sdio_release_host(func);

	//chenq add 0723
	if(g_itm_config_buf != NULL)
	{
		kfree(g_itm_config_buf);
		g_itm_config_buf = NULL;
	}
#ifdef TROUT_TRACE_DBG
	sprd_timer2_destroy();
#ifdef TRACE_LOG_TO_FILE	
	//chenq add 0912
	trout_trace_exit();
#endif	/*TRACE_LOG_TO_FILE */
#endif	/* TROUT_TRACE_DBG */

	TROUT_FUNC_EXIT;
}


static int sdio_trout_suspend(struct device *dev)
{
	TROUT_DBG4("sdio_trout_suspend\n");
	return 0;
}

static int sdio_trout_resume(struct device *dev)
{
	TROUT_DBG4("sdio_trout_resume\n");
	return 0;
}


#define TROUT_VENDOR_ID 0x00
#define TROUT_DEVICE_ID  0x2260

static const struct sdio_device_id sdio_trout_ids[] = {
	{SDIO_DEVICE(TROUT_VENDOR_ID,TROUT_DEVICE_ID)},
	{},
};


static const struct dev_pm_ops sdio_pm = {
	.suspend = sdio_trout_suspend,
	.resume = sdio_trout_resume,
};


static struct sdio_driver sdio_trout_driver ={
	.probe	= sdio_trout_probe,
	.remove	= sdio_trout_remove,
	.name	= "sdio_trout_wifi",
	.id_table = sdio_trout_ids,
	.drv = {
		.name	= "sdio_trout_wifi",
		.pm 	= &sdio_pm,
	},
};
#endif /*if 0 leon liu added for compiling on android 4.x*/

#else

//Add trout_chip_spi_reset to csl_linux.c
static void trout_chip_spi_reset(void)
{
    __gpio_set_value(44, 0);
    mdelay(1);

    if(sprd_3rdparty_gpio_wifi_reset>0)
    {
        __gpio_set_value(sprd_3rdparty_gpio_wifi_reset, 0);
        mdelay(100);
        __gpio_set_value(sprd_3rdparty_gpio_wifi_reset, 1);
        //printk("trout_gpio_reset!\n");
        mdelay(1);
    }
    else
    {
        printk("trout:sprd_3rdparty_gpio_wifi_reset[0x%x]\n",sprd_3rdparty_gpio_wifi_reset);
    }

    __gpio_set_value(44, 1);
}

struct spidev_data *trout_spi_dev_get(void)
{
//    struct spi_device *spi;
    struct trout_private *tp;

    if(g_mac_dev == NULL)
        return NULL;
    
    tp = netdev_priv(g_mac_dev);
    
    return &(tp->spi_data);
}

#ifndef DV_SIM
static int spi_trout_probe(struct spi_device *spi)
{
    int rc;
    struct net_device *netdev;
    struct trout_private *tp;    
    //unsigned long gpio_cfg = MFP_CFG_X(GPIO136, AF0, DS1, F_PULL_NONE, S_PULL_NONE, IO_IE);

#ifdef TROUT_TRACE_DBG	
	sprd_timer2_init();

#ifdef TRACE_LOG_TO_FILE
	//chenq add 0912
	if( trout_trace_init(0,0,0) != 0)
	{
		printk("unable to inti trace \n");
		TRACE_FUNC_EXIT;
		return -1;
	}
#endif	/* TRACE_LOG_TO_FILE */	
#endif	/* DEBUG_MODE */
	
    TROUT_FUNC_ENTER;
    netdev = alloc_etherdev(sizeof(*tp));
    if (!netdev) {
         TROUT_DBG1("unable to alloc new ethernet!\n");
         TROUT_FUNC_EXIT;
        return -ENOMEM;
    }

	//chenq add 0723
	g_itm_config_buf = (unsigned char *)kmalloc(0x7FF, GFP_KERNEL);
	if(!g_itm_config_buf)
	{
		TROUT_DBG1("unable to alloc new g_itm_config_buf!\n");
		free_netdev(netdev);
		TROUT_FUNC_EXIT;
        return -ENOMEM;
	}
	
    
	SET_NETDEV_DEV(netdev, &spi->dev);
    tp = netdev_priv(netdev);
	tp->dev = netdev;
#if 1 //LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,29)
    netdev->netdev_ops = &trout_netdev_ops;

    //chenq add
    netdev->wireless_handlers = &itm_iw_handler_def;
#else
    netdev->open             = mac_open;
    netdev->stop             = mac_close;
    netdev->hard_start_xmit  = mac_xmit;
    netdev->do_ioctl         = mac_ioctl;

    /* called by /proc/net/dev */
	//dev->get_stats = uf_net_get_stats;

    netdev->set_multicast_list = mac_multicast_list;
#endif

	#ifdef IBSS_BSS_STATION_MODE
	strncpy(netdev->name, "wlan%d", sizeof(netdev->name));
	#else
	strncpy(netdev->name, "wlap%d", sizeof(netdev->name));
	#endif
    //strncpy(netdev->name, "wlan%d", sizeof(netdev->name));
    netdev->name[sizeof(netdev->name) - 1] = '\0';

    //chenq move to next line 2012-12-20
    //set spi speed and bitwidth.
    //tp->spi_data.spi = spi;
    //spitrout_init(&(tp->spi_data));
	//dev_set_drvdata(&spi->dev, tp);

    if(sprd_3rdparty_gpio_wifi_irq>0)
    {
        //Hugh: configure trout irq pin.
        gpio_free(sprd_3rdparty_gpio_wifi_irq);
        //sprd_mfp_config(&gpio_cfg, 1);
        rc = gpio_request(sprd_3rdparty_gpio_wifi_irq, "trout int");
        if(rc)
        {
            TROUT_DBG1("cannot alloc gpio for trout irq\n");
            goto err_out0;
        }

        gpio_direction_input(sprd_3rdparty_gpio_wifi_irq);
        rc = sprd_alloc_gpio_irq(sprd_3rdparty_gpio_wifi_irq);
        if(rc < 0)
        {
            TROUT_DBG1("cannot alloc gpio for trout irq\n");
            gpio_free(sprd_3rdparty_gpio_wifi_irq);
            goto err_out0;
        }
    }
    else
    {
        printk("trout:sprd_3rdparty_gpio_wifi_irq[0x%x]\n",sprd_3rdparty_gpio_wifi_irq);
    }

     netdev->irq = rc;       //get irq number.
     g_mac_dev = netdev;		//call mac_init_fn first.
	 //caisf add 2013-02-15
     g_mac_net_stats = &tp->mac_net_stats;
	 
     trout_chip_spi_reset();

    //set spi speed and bitwidth.
    tp->spi_data.spi = spi;
    dev_set_drvdata(&spi->dev, tp);
    spitrout_init(&(tp->spi_data));
    
#ifdef TROUT_B2B_TEST_MODE
    b2b_init(netdev);
#endif
#ifdef TROUT_WIFI_NPI
    //npi_tx_data_init();
//	trout_rf_test_init(netdev);
#endif

    rc = mac_init_fn(netdev);
    if(rc < 0)
    {
        TROUT_DBG1("mac_init_fn() fail!\n");
        //unregister_netdev(netdev);
        goto err_out1;
    }
    
	rc = register_netdev(netdev);
	if(rc == 0)
	{
		rc  = device_create_file(&spi->dev, &trout_sys_dbg);
	    if (rc != 0)
        {
            TROUT_DBG3("warning: trout system debug create failed.\n");
        }
		itm_mac_close1_open2 = 2;
		mac_open(netdev);
        
        TROUT_FUNC_EXIT;
        return 0;
    }

err_out1:
    sprd_free_gpio_irq(netdev->irq);
    if(sprd_3rdparty_gpio_wifi_irq>0)
    {
        gpio_free(sprd_3rdparty_gpio_wifi_irq);
    }

err_out0:
	free_netdev(netdev);
    g_mac_dev = NULL;
	//caisf add 2013-02-15
    g_mac_net_stats = NULL;

	//chenq add 0723
	kfree(g_itm_config_buf);
	g_itm_config_buf = NULL;

	TROUT_FUNC_EXIT;
    return rc;
}


static int __devexit spi_trout_remove(struct spi_device *spi)
{
	struct trout_private *tp = dev_get_drvdata(&spi->dev);
	struct net_device *netdev = NULL;

	TROUT_FUNC_ENTER;
    if((tp == NULL) || ((netdev = tp->dev) == NULL))
    {
        TROUT_DBG1("%s: error! tp(%p) or netdev(%p) is NULL!\n", __FUNCTION__, tp, netdev);
        TROUT_FUNC_EXIT;
        return -EINVAL;
    }
#ifdef TROUT_WIFI_NPI
    //npi_tx_data_release();
//	trout_rf_test_release(netdev);
#endif
	//chenq add 2012-10-18
	itm_mac_close1_open2 = 1;
	mac_close(netdev);
    device_remove_file(&spi->dev, &trout_sys_dbg);

	/* Clean up netdev. */
	unregister_netdev(netdev);

    free_mem_regions();
    //free gpio resource.
    sprd_free_gpio_irq(netdev->irq);
    if(sprd_3rdparty_gpio_wifi_irq>0)
    {
        gpio_free(sprd_3rdparty_gpio_wifi_irq);
    }

    TROUT_DBG4("trout: spitrout_exit!\n");
    spitrout_exit(&tp->spi_data);
    g_mac_dev = NULL;
	//caisf add 2013-02-15
    g_mac_net_stats = NULL;

	#if 0
	/* Clean up netdev. */
	unregister_netdev(netdev);
	#endif
	
	//chenq add 0723
	kfree(g_itm_config_buf);
    g_itm_config_buf = NULL;

	TROUT_FUNC_EXIT;

#ifdef TRACE_LOG_TO_FILE	
	//chenq add 0912
	trout_trace_exit();
	
	sprd_timer2_destroy();
#endif	/* TRACE_LOG_TO_FILE */
	return 0;
}

#else
static int spi_trout_probe(void)
{
	int rc;
    struct net_device *netdev;
    struct trout_private *tp;    
	struct spi_device *spi = kmalloc(sizeof(*spi), GFP_KERNEL);

#ifdef TROUT_TRACE_DBG
	//chenq add 0912
	if( trout_trace_init(0,0,0) != 0)
	{
		printk("unable to inti trace \n");
		free(spi);
		return -1;
	}
#endif
    netdev = alloc_etherdev(sizeof(*tp));    
    if (!netdev) 
    {
        printk("unable to alloc new ethernet\n");
        return -ENOMEM;
    }

	//chenq add 0723
	g_itm_config_buf = (unsigned char *)kmalloc(0x7FF, GFP_KERNEL);
	if(!g_itm_config_buf)
	{
		printk("unable to alloc new g_itm_config_buf \n");
		free_netdev(netdev);
        return -ENOMEM;
	}
	//////////////////////////
    
//	SET_NETDEV_DEV(netdev, &spi->dev);
    tp = netdev_priv(netdev);
	tp->dev = netdev;
    netdev->netdev_ops = &trout_netdev_ops;

    //chenq add
    netdev->wireless_handlers = &itm_iw_handler_def;	
	
    strncpy(netdev->name, "iwlan%d", sizeof(netdev->name));
    netdev->name[sizeof(netdev->name) - 1] = '\0';

    trout_chip_spi_reset();
    
    tp->spi_data.spi = spi;
	dev_set_drvdata(&spi->dev, tp);
    netdev->irq = 12;       //get irq number.

    //init mac
    g_mac_dev = netdev;
	//caisf add 2013-02-15
    g_mac_net_stats = &tp->mac_net_stats;

	rc = register_netdev(netdev);
	if(rc == 0)
	{
		printk("register netdev OK!\n");
	    rc = mac_init_fn(netdev);
	    if(rc < 0)
	    {
	        printk("mac_init_fn() fail!\n");
			goto err_out0;
	    }
	    
        return 0;
    }
    	

err_out0:
	printk(KERN_ALERT "net device register failed!\n");
	kfree(spi);
    unregister_netdev(netdev);
    g_mac_dev = NULL;
	//caisf add 2013-02-15
    g_mac_net_stats = NULL;

    return rc;
}

static int __devexit spi_trout_remove(struct spi_device *spi)
{
	struct trout_private *tp = dev_get_drvdata(&spi->dev);
	struct net_device *netdev = tp->dev;

    if((tp == NULL) || ((netdev = tp->dev) == NULL))
    {
        printk("%s: error! tp OR netdev is NULL!\n", __FUNCTION__);
        return -EINVAL;
    }
    
    g_mac_dev = NULL;
	//caisf add 2013-02-15
    g_mac_net_stats = NULL;

	/* Clean up netdev. */
	unregister_netdev(netdev);

	//chenq add 0723
	kfree(g_itm_config_buf);
	g_itm_config_buf = NULL;
#ifdef TROUT_TRACE_DBG    
  //chenq add 0912
	trout_trace_exit();
#endif	  
	return 0;
}
#endif

static struct spi_driver sprd_trout_driver = {
    .driver = {
        .name   = "spitrout",
        .bus    = &spi_bus_type,
        .owner  = THIS_MODULE,
    },

    .probe      = spi_trout_probe,
    .remove     = __devexit_p(spi_trout_remove),
};

#endif
/*****************************************************************************/
/*                                                                           */
/*  Function Name : init_mac_driver                                          */
/*                                                                           */
/*  Description   : This is the function called when the kernel is booting up*/
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*  Processing    : None                                                     */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
static int g_use_count = 0;	//module init debug.

/*xuan yang, 2013.5.30, add resume assoc flg*/
static UWORD8 need_assoc_flg = 0;
static UWORD8 resume_assoc_flg = 0;
static UWORD8 assoc_ssid[33];

UWORD8 set_resume_assoc_flg(UWORD8 flg)
{
	resume_assoc_flg = flg;
	printk("%s(%d) resume_assoc=%d\n", __FUNCTION__, __LINE__, resume_assoc_flg);
	return resume_assoc_flg;
}

UWORD8 get_resume_assoc_flg(void)
{
	printk("%s(%d) resume_assoc=%d\n", __FUNCTION__, __LINE__, resume_assoc_flg);
	return resume_assoc_flg;
}


extern int send_null_frame_to_AP_trick(UWORD8 psm, BOOL_T is_qos, UWORD8 priority);
extern int send_ps_poll_to_AP_trick(void);
extern unsigned int txhw_idle(void);
extern void show_tx_slots(void);
void dump_allregs(unsigned long ix, unsigned long iy);
UWORD32 txvs = 0;

#if (defined(CONFIG_PM) && defined(TROUT_WIFI_POWER_SLEEP_ENABLE))
extern BOOL_T is_all_machw_q_null(void);

extern BOOL_T  host_awake_by_arm7;
extern UWORD32 ps_last_int_mask;

extern BOOL_T is_all_machw_q_null(void);
static int trout_wifi_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct trout_private *tp = netdev_priv(ndev);
	UWORD32 *tmp, t;
	int cnt = 0, idx = 0, tnr = 0, v;
	UWORD32	ss = 0;
	unsigned char *pp = NULL;

//	pr_info("======== %s ========\n", __func__);
	pr_info("SPD\n" );

	// avoid suspend when NPI testing
#ifdef TROUT_WIFI_NPI
	pr_info("(%s)WIFI is testing in NPI mode, avoid suspend\n", __func__);
	return 1;
#endif
    //xuan.yang, 2013-10-16, if the trout module is unloading, the wifi can not suspend
    if(check_trout_module_state(TROUT_MODULE_UNLOADING)) {
       printk("%s the trout module is unloading, return fail\n", __FUNCTION__);
       return 1;
    }

#ifndef WIFI_SLEEP_POLICY
	mutex_lock(&tp->cur_run_mutex);
	printk("%s get cur run mutex ------------ yx \n", __FUNCTION__);
#endif
	// avoid suspend when NPI testing
	if( reset_mac_trylock() == 0 ) {
		pr_info("Wi-Fi is under reseting, please try %s again!\n", __func__);
#ifndef WIFI_SLEEP_POLICY
		mutex_unlock(&tp->cur_run_mutex);
#endif
		return 1;
	}
#ifdef WIFI_SLEEP_POLICY 
        if(wake_lock_active(&handshake_frame_lock)
                || wake_lock_active(&scan_ap_lock)) // caisf/zhao add, 1014   
        {  
             pr_info("%s-%d: wake_lock locked, skip suspend.\n", __func__, __LINE__);  
		reset_mac_unlock();
             return 1;  
        }  

	if (!mutex_trylock(&suspend_mutex)){
		reset_mac_unlock();
		return 1;
	}
	else {
#ifdef WAKE_LOW_POWER_POLICY
	if(!mutex_trylock(&tp->ps_mutex))	//add by chwg, 2013.12.7
	{		
		mutex_unlock(&suspend_mutex);
		reset_mac_unlock();
		//mutex_unlock(&tp->cur_run_mutex);
		return 1;
	}
	stop_alarm(g_flow_detect_timer);
	exit_low_power_mode(BFALSE);
#endif /* WAKE_LOW_POWER_POLICY */
		if((get_mac_state() == ENABLED) || (g_keep_connection == BTRUE)) { /* keep Wi-Fi on if connection was established */
		/*if(g_wifi_suspend_status == wifi_suspend_nosuspend) { [>recover from power save<]*/
			/*g_wifi_suspend_status = wifi_suspend_early_suspend; [>keep out all ioctl ASAP<]*/
			/*send_null_frame_to_AP(STA_DOZE, BTRUE, 0);*/
			//printk("[%s] netif_stop_queue\n" ,__FUNCTION__);
			if(g_wifi_bt_coex)	//add by chwg.
			{
				printk("%s: In WiFi&BT coexist mode, not suspend!\n", __func__);
				/*leon liu added, release ps_mutex*/
				mutex_unlock(&tp->ps_mutex);
				mutex_unlock(&suspend_mutex);
				reset_mac_unlock();
				return 1;
			}
			
			netif_stop_queue(g_mac_dev);
			//pr_info("======== cancel works\n");
			//cancel_work_sync(&tp->event_work); /*cancel the works before suspend*/
			cnt = 0;
			// here we prevent all mgmet frames
		#ifdef MAC_WMM
			if(get_wmm_enabled() == BTRUE){
				force_suspend_softtxq_above(get_txq_num(0));
			}else{
				force_suspend_softtxq_above(NORMAL_PRI_Q);
			}
		#else
			force_suspend_softtxq_above(NORMAL_PRI_Q);
		#endif
			while(!is_all_machw_q_null()){
				pr_info("Using is_all_machw_q_null()\n");
				msleep(100);
				cnt++;
				if(cnt >= 40){
					printk("wait for other frame send out timeout!\n");
					show_tx_slots();
					netif_wake_queue(g_mac_dev);
#ifdef WAKE_LOW_POWER_POLICY
				mutex_unlock(&tp->ps_mutex);
#endif
					mutex_unlock(&suspend_mutex);
					reset_mac_unlock();
					return 1;
				}
			}
			cnt = 0;
			// here we make sure all txq above BK_Q is disable, and other Q must be enabled
		#ifdef MAC_WMM
			if(get_wmm_enabled() == BTRUE){
				force_suspend_softtxq_above(AC_BK_Q);
			}else{
				force_suspend_softtxq_above(NORMAL_PRI_Q);
			}
		#else
			force_suspend_softtxq_above(NORMAL_PRI_Q);
		#endif
retry:
			v = send_null_frame_to_AP_trick(STA_DOZE, BTRUE, 0);
			if(!v && null_frame_dscr != NULL) {
				tmp = null_frame_dscr;
				tnr = 0;
				pr_info("======== waiting for null frame\n");
wait:
				t = wait_for_completion_timeout(&null_frame_completion, msecs_to_jiffies(20));
				if(null_frame_dscr == tmp){
					if(t == 0)
						tnr++;
					if(tnr < 50)
						goto wait;
	//				printk("@@@: no tx_complete is for at least 1s\n");
                    printk("no tx ISR\n");
				}
				if((UWORD32)null_frame_dscr == 0x1){
					cnt++;
					//printk("@@@:SUSPEND SEND NULL try %d times\n", cnt);
                     printk("S-S-N %d\n", cnt);
					if(cnt < 5)
						goto retry;
				}
			//	pr_info("======== waiting for null frame done\n");
			}else{
				ss = 1;
				//printk("@@@:SUSPEND no memory for NULL\n");
                printk("SPD no mem for NULL\n");
			}
			if(ss || tnr >= 50 || cnt >= 5 ){
			/* to prevent suspend retry too much times and still fail thus cause */
			/* rcv pkt which doing in process_all_events couldn't do lead to     */
			/* link loss --chwg 2014.01.3 */
			g_link_loss_count = 0;
				netif_wake_queue(g_mac_dev);
#ifdef WAKE_LOW_POWER_POLICY
			mutex_unlock(&tp->ps_mutex);
#endif
				mutex_unlock(&suspend_mutex);
				reset_mac_unlock();
				return 1;
			}else{
				UWORD32 zero_ip = 0;
			    disable_all_txq();
                 pp = get_local_ipadd(); 
                 if(pp == NULL){
                     pp = (unsigned char *)&zero_ip;
					 printk("puyan %s pp is null",__FUNCTION__);
                 }
				 
				/* tell ARM7 the IP address */
				host_write_trout_ram((void *)(BEACON_MEM_END - 4), (void *)pp, 4);
			prepare_null_frame_for_cp(0, BTRUE, 0);
			prepare_ps_poll_for_cp();
			sta_doze_trick(SUSPEND_DOZE,1);
				printk("=====%s: suspend done!======\n", __func__);
			}
		} else {
			if(g_wifi_suspend_status == wifi_suspend_nosuspend) { /*turn off Wi-Fi if there was no connection */
				pr_info("======== no connection, turn off Wi-Fi\n", __func__);
				//printk("[%s] netif_stop_queue\n" ,__FUNCTION__);
				netif_stop_queue(ndev);	//modify by chengwg, 2013.7.9
				/*g_wifi_suspend_status = wifi_suspend_suspend; [>keep out all ioctl ASAP<] */
				reset_mac_unlock();
				restart_mac_plus(&g_mac, 0);
				sta_sleep();
				pr_info("======== %s done ========\n", __func__);

#ifdef WAKE_LOW_POWER_POLICY
			mutex_unlock(&tp->ps_mutex);
#endif		
				mutex_unlock(&suspend_mutex);
				return 0;
			} else if(g_wifi_suspend_status == wifi_suspend_suspend) { /*done by STOP*/
				pr_info("======== done by STOP\n", __func__);
			}
		}
        }	
	//printk("SPD done!\n");
	printk("==========%s: exit=========\n", __func__);
#ifdef WAKE_LOW_POWER_POLICY
	mutex_unlock(&tp->ps_mutex);
#endif
	mutex_unlock(&suspend_mutex);
	reset_mac_unlock();
	return 0;
#else
	netif_stop_queue(ndev);	//modify by chengwg, 2013.7.9
	g_wifi_suspend_status = wifi_suspend_suspend;
	
	/*xuan yang, 20.5.30, save desired ssid, set assoc flg, save wid info*/    
	//save desired ssid
	WORD8 *ssid_temp = mget_DesiredSSID();
	WORD8 ssid_len = strlen(ssid_temp);
	strcpy(assoc_ssid, ssid_temp);
	assoc_ssid[ssid_len] = '\0';

	//set assoc flg
	if ((DISABLED != get_mac_state()) && (0 < ssid_len)) {
		need_assoc_flg = 1;
	} else {
		need_assoc_flg = 0;
	}
	
	//save wid info
    if( 1 == need_assoc_flg) {
		//xuan yang, 2013-8-23, add wid mutex to sync g_reset_mac_in_progress
		tp = netdev_priv(g_mac_dev);
		mutex_lock(&tp->rst_wid_mutex);

		g_reset_mac_in_progress    = BTRUE;
		save_wids();
		g_reset_mac_in_progress    = BFALSE;

		mutex_unlock(&tp->rst_wid_mutex);
	}

	printk("%s(%d) need_assoc=%d assoc_ssid=%s mac_state=%d\n", 
				__FUNCTION__, __LINE__, need_assoc_flg, assoc_ssid, get_mac_state());
	
#if 0
	if(dev && !netif_queue_stopped(dev))
		netif_stop_queue(dev);
	wait_for_tx_finsh();
	#if 0
        if(get_ps_state() == STA_DOZE)
        {
           pr_info("-------------------God!!!----------------------");
           sta_awake(BTRUE);
        }
	#endif
	restart_mac_plus(&g_mac, 0);
#endif
	reset_mac_unlock();
	restart_mac_plus(&g_mac, 0);
	//netif_stop_queue(ndev);
	sta_sleep();
	mutex_unlock(&tp->cur_run_mutex);
	printk("%s release cur run mutex ------------ yx \n", __FUNCTION__);
	return 0;
#endif
}
static int trout_wifi_resume(struct device *dev)
{
	pr_info("========%s========\n", __func__);
	struct platform_device *pdev = to_platform_device(dev);
	struct net_device *ndev = platform_get_drvdata(pdev);
	//pr_info("======== %s ========\n", __func__);
	UWORD32 *tmp;
    

	int cnt = 0, tnr = 0, v;
	struct trout_private *stp;
	

	#ifdef TROUT_WIFI_NPI
    pr_info("(%s)WIFI is testing in NPI mode, avoid resume\n", __func__);
    return 0;
    #endif
	
#ifdef WIFI_SLEEP_POLICY
	unsigned char *pp = NULL; 
        stp = netdev_priv(ndev);
#ifdef WAKE_LOW_POWER_POLICY
    mutex_lock(&stp->ps_mutex);
#endif
    pp = get_local_ipadd();
	if(pp!=NULL){
        g_wakeup_flag_for_60s = 1;
    }
	/*mutex_trylock(&suspend_mutex);*/
	/*pr_info("======== %s: g_wifi_suspend_status %d\n", __func__, g_wifi_suspend_status);*/
        if((get_mac_state() == ENABLED) || (g_keep_connection == BTRUE)) { /* keep Wi-Fi on if connection was established */
	/*if(g_wifi_suspend_status == wifi_suspend_early_suspend) { [>recover from power save<]*/
		sta_awake_trick();
		/* other wakeup source, we need sync RX read/write pointer */
		//host_write_trout_reg(ps_last_int_mask , (UWORD32)rCOMM_INT_MASK);
		//update_trout_int_mask(ps_last_int_mask);
		host_awake_by_arm7 = BTRUE;

		enable_all_txq();

		if(waitqueue_active(&wifi_resume_completion.wait)) {
			//pr_info("======== kick ass\n", __func__);
              pr_info("KA\n");
			complete(&wifi_resume_completion);
		}

		mac_event_schedule();
		/*rx_complete_isr(HIGH_PRI_RXQ);*/

#if 0
retry:
		/*send_null_frame_to_AP(STA_ACTIVE, BTRUE, 0);*/
	    //v = send_null_frame_to_AP_trick(STA_ACTIVE, BTRUE, 0);
        v = send_ps_poll_to_AP_trick();

		if(!v && null_frame_dscr != NULL) {
			tmp = null_frame_dscr;
			tnr = 0;
wait:
			msleep(20);
			if(null_frame_dscr == tmp){
				tnr++;
				if(tnr < 20)
					goto wait;
			//	printk("@@@: RESUME -at least 400ms no TX ISR for NULL frame\n");
                  printk("R-N-I\n");
			}
			if((UWORD32)null_frame_dscr == 0x1){
				cnt++;
				//printk("@@@:RESUME-SEND NULL try %d times\n", cnt);
                printk("R-S-N %d\n", cnt);
				if(cnt < 3)
					goto retry;
			}
		//	printk("OTHER satus!\n");
		}else{
			//printk("@@@:RESUME no memory for NULL or other reason\n");
              printk("R no mem for NULL\n");
		}
#endif
		netif_wake_queue(g_mac_dev);
	} else {
	/*} else if(g_wifi_suspend_status == wifi_suspend_suspend) { [>recover from sleep<]*/
		sta_wakeup();
		/*del_timer_sync(&buffer_frame_timer);*/
		netif_wake_queue(ndev);
	}
	enable_all_txq();
	netif_wake_queue(g_mac_dev);
	if (host_read_trout_reg((UWORD32)rSYSREG_POWER_CTRL) & 0x800000)
		pr_info("======== %s Trout down!!!\n", __func__);
	pr_info("RESUME done!\n");
#ifdef WAKE_LOW_POWER_POLICY
	printk("after resume, restart flow detect timer again!\n");
	clear_history_flow_record();
	restart_flow_detect_timer(&g_flow_detect_timer, FLOW_DETECT_TIME, 0);
	mutex_unlock(&stp->ps_mutex);
#endif
	/*mutex_unlock(&suspend_mutex);*/
	printk("==========%s: exit=========\n", __func__);
	
	return 0;
#else
	struct trout_private *stp;

	//xuan.yang, 2013-09-17, get run metux, and set resume state
	stp = netdev_priv(ndev);
    mutex_lock(&stp->cur_run_mutex);
	/*xuan yang, 20.5.30, set send state flg*/
	printk("%s(%d) need_assoc=%d assoc_ssid=%s\n",
				__FUNCTION__, __LINE__, need_assoc_flg, assoc_ssid);
	//set send state flg
	if(1 == need_assoc_flg) {
		set_resume_assoc_flg(1);
	}

	sta_wakeup();
	netif_wake_queue(ndev);
	
	/*xuan yang, 20.5.30, restore wid, set scan mode and assoc ap*/  
	//restore wid
	if(1 == need_assoc_flg)	{
		//xuan yang, 2013-8-23, add wid mutex to sync g_reset_mac_in_progress
		stp = netdev_priv(g_mac_dev);
		mutex_lock(&stp->rst_wid_mutex);

		g_reset_mac_in_progress    = BTRUE;
		if( reset_mac_trylock() ) {	
			/*xuan yang, 2013.6.11, do nothing in the reset mac process*/
			restore_wids();
			resolve_wid_conflicts(&g_mac);
			reset_mac_unlock();
		}
		g_reset_mac_in_progress    = BFALSE;

		mutex_unlock(&stp->rst_wid_mutex);
	}
	
	//set scan mode and assoc ap
	if( 1 == need_assoc_flg) {
		g_scan_source = DEFAULT_SCAN;
		need_assoc_flg = 0;
	} else {
		g_scan_source = USER_SCAN;
	}
	set_site_survey(SITE_SURVEY_OFF);
	start_mac_and_phy(&g_mac);

	//xuan.yang, 2013-09-17, set resume state, and unlock run metux
	stp->resume_complete_state = 1;
	mutex_unlock(&stp->cur_run_mutex);
	printk("%s release cur run mutex ------------ yx \n", __FUNCTION__);

	return 0;
#endif
}
#else
#define	trout_wifi_suspend	NULL
#define	trout_wifi_resume	NULL
#endif

static const struct dev_pm_ops trout_wifi_pm = {
	.suspend = trout_wifi_suspend,
	.resume = trout_wifi_resume,
};



#ifdef WAKE_LOW_POWER_POLICY
ALARM_HANDLE_T *g_flow_detect_timer = NULL;
LOW_POWER_FLOW_CTRL_T g_low_power_flow_ctrl;
int enter_low_power_mode(void)	//chwg debug.
{
	struct trout_private *tp = netdev_priv(g_mac_dev);
	UWORD32 *tmp, t;
	int cnt = 0, idx = 0, tnr = 0, v;
	UWORD32	ss = 0;

	if(wake_lock_active(&deauth_err_lock))  
	{
		printk("start low power mode before unlock 'deauth_err_lock'!\n");
		return 1;
	}
	
	if(1 == itm_get_dhcp_status())
	{
		printk("start low power mode before dhcp do not run!\n");
		return 1;
	}

	if(get_local_ipadd() == NULL)	//start after sta get the ip addr, debug using!
	{
		printk("start low power mode before get ip, exit!\n");
		return 1;
	}
	
	pr_info("======== %s start ========\n", __func__);
	mutex_lock(&low_power_mutex);

    //xuan.yang, 2013-10-16, if the trout module is unloading, the wifi can not suspend
    if(check_trout_module_state(TROUT_MODULE_UNLOADING)) 
    {
       printk("%s the trout module is unloading, return fail\n", __func__);
	   mutex_unlock(&low_power_mutex);
       return 1;
    }

	// avoid suspend when NPI testing
	if(reset_mac_trylock() == 0) 
	{
		pr_info("Wi-Fi is under reseting, please try %s again!\n", __func__);
		mutex_unlock(&low_power_mutex);
		return 1;
	}

#if 1
	if(wake_lock_active(&handshake_frame_lock)
					|| wake_lock_active(&scan_ap_lock)) // caisf/zhao add, 1014   
	{  
		pr_info("%s-%d: wake_lock locked, skip suspend.\n", __func__, __LINE__);  
		reset_mac_unlock();
		mutex_unlock(&low_power_mutex);
		return 1;  
	}  
#else
	if(wake_lock_active(&scan_ap_lock)) // caisf/zhao add, 1014   
	{  
		pr_info("%s-%d: wake_lock scan_ap_lock locked, skip suspend.\n", __func__, __LINE__);  
		reset_mac_unlock();
		mutex_unlock(&low_power_mutex);
		return 1;  
	}
#endif

	if(!mutex_trylock(&tp->ps_mutex))
	{
		printk("%s: get ps_mutex lock fail!\n", __func__);
		reset_mac_unlock();
		mutex_unlock(&low_power_mutex);
		return 1;
	}

	if(g_wifi_power_mode == WIFI_LOW_POWER_MODE)
	{
		printk("%s: current is already in low power mode!\n", __func__);
		mutex_unlock(&tp->ps_mutex);
		reset_mac_unlock();
		mutex_unlock(&low_power_mutex);
		return 0;
	}

	//printk("going to low power mode, stop low power detect timer first!\n");
	//stop_alarm(g_mac_low_power_timer);
	//stop_alarm(g_flow_detect_timer);
	
	if((get_mac_state() == ENABLED) || (g_keep_connection == BTRUE)) 
	{ /* keep Wi-Fi on if connection was established */
		netif_stop_queue(g_mac_dev);
		cnt = 0;

		/* This make sure in active mode before enter low power mode */
		//send_null_frame_to_AP_trick(STA_ACTIVE, BTRUE, 0);
		
	#ifdef MAC_WMM
		if(get_wmm_enabled() == BTRUE)
		{
			force_suspend_softtxq_above(get_txq_num(0));
		}
		else
		{
			force_suspend_softtxq_above(NORMAL_PRI_Q);
		}
	#else
		force_suspend_softtxq_above(NORMAL_PRI_Q);
	#endif
		while(!is_all_machw_q_null())
		{
			printk("Using is_all_machw_q_null()\n");
			msleep(100);
			cnt++;
			if(cnt >= 40)
			{
				printk("wait for other frame send out timeout!\n");
				show_tx_slots();
				netif_wake_queue(g_mac_dev);
				mutex_unlock(&tp->ps_mutex);
				reset_mac_unlock();
				mutex_unlock(&low_power_mutex);
				return 1;
			}
		}
		cnt = 0;
		// here we make sure all txq above BK_Q is disable, and other Q must be enabled
	#ifdef MAC_WMM
		if(get_wmm_enabled() == BTRUE)
		{
			force_suspend_softtxq_above(AC_BK_Q);
		}
		else
		{
			force_suspend_softtxq_above(NORMAL_PRI_Q);
		}
	#else
		force_suspend_softtxq_above(NORMAL_PRI_Q);
	#endif
retry:
		v = send_null_frame_to_AP_trick(STA_DOZE, BTRUE, 0);
		if(!v && null_frame_dscr != NULL) 
		{
			tmp = null_frame_dscr;
			tnr = 0;
			pr_info("======== waiting for null frame\n");
wait:
			t = wait_for_completion_timeout(&null_frame_completion, msecs_to_jiffies(20));
			if(null_frame_dscr == tmp)
			{
				if(t == 0)
					tnr++;
				if(tnr < 50)
					goto wait;
				printk("@@@: no tx_complete is for at least 1s\n");
			}
			if((UWORD32)null_frame_dscr == 0x1)
			{
				cnt++;
				printk("@@@:SUSPEND SEND NULL try %d times\n", cnt);
				if(cnt < 5)
					goto retry;
			}
			pr_info("======== waiting for null frame done\n");
		}
		else
		{			
			ss = 1;
			printk("%s:SUSPEND no memory for NULL\n", __func__);
		}
		
		if(ss || tnr >= 50 || cnt >= 5)
		{
			netif_wake_queue(g_mac_dev);
			mutex_unlock(&tp->ps_mutex);
			reset_mac_unlock();
			mutex_unlock(&low_power_mutex);
			return 1;
		}
		else
		{
			disable_all_txq();
			prepare_null_frame_for_cp(0, BTRUE, 0);
			prepare_ps_poll_for_cp();
			sta_doze_trick(LOW_POWER_DOZE,1);
			netif_wake_queue(g_mac_dev);
//			restart_exit_low_power_timer(&g_exit_low_power_timer, SLEEP_TIMEOUT, 0);
			mutex_unlock(&tp->ps_mutex);
			reset_mac_unlock();
			mutex_unlock(&low_power_mutex);
			printk("=====%s: succeed!======\n", __func__);
			return 0;
		}
	}
	
	printk("======== %s fail ========\n", __func__);
	mutex_unlock(&tp->ps_mutex);
	reset_mac_unlock();
	mutex_unlock(&low_power_mutex);
	
	return 1;
}

extern unsigned int  Get_Power_Mode(void);	//define in trout_sdio module.
//////////// create tx/rx traffic timer ////////////
static void flow_detect_work(struct work_struct *work)
{
	int ret = 0;
	UWORD32 pkt_num = 0;
	UWORD32 last_pkt_num[3];
	static UWORD32 detect_cnt = 0;
	UWORD32 tc = 0;
	 if(reset_mac_trylock() == 0){
		return;
	}

	ALARM_WORK_ENTRY(work);
	/* In WiFi & BT hardware coexist mode, we will not going to low power mode! */
	//if(Get_Power_Mode() == 5)
	if(g_wifi_bt_coex)
	{
		printk("current is in WiFi+BT software coexist mode, skip!\n");
		clear_history_flow_record();
		restart_flow_detect_timer(&g_flow_detect_timer, FLOW_DETECT_TIME, 0);
		ALARM_WORK_EXIT(work);
		reset_mac_unlock();
		return;
	}

	last_pkt_num[0] = g_low_power_flow_ctrl.total_pkt_num_record[0];
	last_pkt_num[1] = g_low_power_flow_ctrl.total_pkt_num_record[1];
	last_pkt_num[2] = g_low_power_flow_ctrl.total_pkt_num_record[2];
	pkt_num = g_low_power_flow_ctrl.tx_pkt_num + g_low_power_flow_ctrl.rx_pkt_num;

	printk("flow_detect: detect_cnt=%d, tx_pkt_num=%u, rx_pkt_num=%u\n", detect_cnt+1, 
					g_low_power_flow_ctrl.tx_pkt_num, g_low_power_flow_ctrl.rx_pkt_num);

	printk("pkt_num=%u, last[0]=%u, last[1]=%u, last[2]=%u\n", 
					pkt_num, last_pkt_num[0], last_pkt_num[1], last_pkt_num[2]);

	tc = pkt_num + last_pkt_num[0] + last_pkt_num[1] + last_pkt_num[2];
	if((++detect_cnt >= 3) && (tc < LOW_POWER_TOTAL_PKT_THRESHOLD)) 
	{
		printk("flow detect: pkt level %u less than threshold %d, start low power mode!\n",
						tc, LOW_POWER_TOTAL_PKT_THRESHOLD);
		ret = enter_low_power_mode();
		if(ret == 0)
		{
			detect_cnt = 0;
			printk("enter low power mode succeed!\n");
			ALARM_WORK_EXIT(work);
			reset_mac_unlock();		
			return;
		}
	}

	g_low_power_flow_ctrl.total_pkt_num_record[2] = g_low_power_flow_ctrl.total_pkt_num_record[1];
	g_low_power_flow_ctrl.total_pkt_num_record[1] = g_low_power_flow_ctrl.total_pkt_num_record[0];
	g_low_power_flow_ctrl.total_pkt_num_record[0] = pkt_num;
	restart_flow_detect_timer(&g_flow_detect_timer, FLOW_DETECT_TIME, 0);
	ALARM_WORK_EXIT(work);
	reset_mac_unlock();
}


void flow_detect_fn(ADDRWORD_T data)
{
    alarm_fn_work_sched(data);
}

void restart_flow_detect_timer(ALARM_HANDLE_T **hdl, UWORD32 time, UWORD32 data)
{
    if(*hdl == NULL)
    {
        *hdl = create_alarm(flow_detect_fn, data, flow_detect_work);
        if(*hdl == NULL)
        {
			printk("create start low power timer fail!\n");
			return;
        }
    }

    stop_alarm(*hdl);
    g_low_power_flow_ctrl.tx_pkt_num = 0;
	g_low_power_flow_ctrl.rx_pkt_num = 0;
    start_alarm(*hdl, time);
}


int exit_low_power_mode(BOOL_T restart)		//chwg debug.
{
	UWORD32 *tmp;
	int cnt = 0, tnr = 0, v;

	print_symbol("[exit_low_power_mode] -> %s\n", (unsigned long)__builtin_return_address(0));
	mutex_lock(&low_power_mutex);

	if(g_wifi_power_mode != WIFI_LOW_POWER_MODE)
	{
		printk("%s: not in lower power mode!\n", __func__);
		mutex_unlock(&low_power_mutex);
		return 0;
	}
	
//    if((get_mac_state() == ENABLED) || (g_keep_connection == BTRUE)) 
//    { /* keep Wi-Fi on if connection was established */
		sta_awake_trick();
		//host_awake_by_arm7 = BTRUE;
		g_wifi_power_mode = WIFI_NORMAL_POWER_MODE;
		enable_all_txq();

		if(wsem_is_locked())	//just used for test current is under reseting!
		{
			printk("Wi-Fi is under reseting, return!\n", __func__);
			mutex_unlock(&low_power_mutex);
			return 0;
		}
	
		mac_event_schedule();
//	}

//	netif_wake_queue(g_mac_dev);
	if (host_read_trout_reg((UWORD32)rSYSREG_POWER_CTRL) & 0x800000)
		pr_info("======== %s Trout down!!!\n", __func__);

	pr_info("======== %s done ========\n", __func__);

	if(restart)
	{
		clear_history_flow_record();
		//restart_low_power_timer(&g_mac_low_power_timer, WAKE_TIMEOUT, 0);
		restart_flow_detect_timer(&g_flow_detect_timer, FLOW_DETECT_TIME, 0);
	}
	mutex_unlock(&low_power_mutex);

	return 0;
}


#endif
/*leon liu added debug wake lock for AP mode*/
#ifdef BSS_ACCESS_POINT_MODE
struct wake_lock ap_lock;
#endif

static int trout_wifi_remove(struct platform_device *pdev)
{
	struct net_device *netdev = g_mac_dev;
	struct trout_private *tp = netdev_priv(netdev);

#ifdef 	IBSS_BSS_STATION_MODE
	//chenq add do ap list merge logic 2013-08-28
	UWORD32 *tmp_addr = NULL;
#endif

	TROUT_FUNC_ENTER;

	TROUT_DBG4("mac driver exit...\n");

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
      set_trout_module_state(TROUT_MODULE_UNLOADING);
#endif

	//leon liu added, stop powersave timer
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
	pstimer_stop(&pstimer);
	//Ensure timer is stopped
	mdelay(100);
	//leon liu added for powersave timer deintialization 2013-4-1
	pstimer_destroy(&pstimer);
#endif
	host_notify_arm7_coex_ready(BFALSE);
	host_notify_arm7_wifi_on(BFALSE);

// clear tx int before driver remove in coexistence situation. wzl
#ifdef 	IBSS_BSS_STATION_MODE
	if(BTRUE == g_wifi_bt_coex){
		UWORD32 arm2host = 0;
		arm2host = host_read_trout_reg((UWORD32)rCOMM_ARM2HOST_INFO3);
		if(arm2host & BIT0)	//tx int.
		{
			TROUT_DBG1("[%s]: clear tx int before wifi driver remove\n", __FUNCTION__);
			arm2host &= (~BIT0);		
			host_write_trout_reg(arm2host, (UWORD32)rCOMM_ARM2HOST_INFO3);
		}
	}
#endif

	if(netdev == NULL)
	{
		TROUT_DBG1("%s: error! netdev(%p) is NULL!\n", __FUNCTION__, netdev);
		return -EINVAL;
	}

#ifdef BSS_ACCESS_POINT_MODE
	host_notify_arm7_connect_status(BFALSE);
#endif	/* BSS_ACCESS_POINT_MODE */

	/* Reset the MAC hardware and software, PHY etc */
	reset_mac(&g_mac, BFALSE);	//modify by chengwg 2013.03.13!
	//host_notify_arm7_wifi_reset(BFALSE); // after reset_mac, reset signal before wifi off.
	flush_work(&tp->event_work);
	if(tp->event_wq != NULL)
    {
		flush_workqueue(tp->event_wq);
	    destroy_workqueue(tp->event_wq);
	
		tp->event_wq = NULL;
	}
    //disable_machw_phy_and_pa();

    stop_mac_daemons();
    
#ifdef LOCALMEM_TX_DSCR
    /* Release occupied local memory back to Linux */
	if(g_local_mem_dma != NULL)
	{
    	kfree(g_local_mem_dma);
		g_local_mem_dma = NULL;
    }
#endif /* LOCALMEM_TX_DSCR */

//Hugh fix bug when undef 'MAC_ADDRESS_FROM_FLASH'
#ifdef MAC_ADDRESS_FROM_FLASH
    iounmap((void *)g_virt_flash_base);
#endif

	//stop_mem_adjust_task();	//add by chengwg.
#ifdef TROUT_TRACE_DBG	
	//trace_thread_exit();
	//trace_func_exit();
#endif
	/****************************/

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
	//zhou add
	//unregister_early_suspend(&trout_wifi_early_suspend_handler);
#endif

	//chenq add
	/* Clean up netdev. */
	unregister_netdev(netdev);

/*leon liu added cleanup code for cfg80211*/
#ifdef CONFIG_CFG80211
	if (tp->wdev != NULL)
	{
		trout_wdev_free(tp->wdev);
	}
#endif
	host_rx_queue_free();	//mv to here by chengwg, 2013-03-13.
#ifdef TX_PKT_USE_DMA		
	tx_dma_buf_free();
#endif	
	free_mem_regions();
	/*sprd_free_gpio_irq(netdev->irq);*/
        if(sprd_3rdparty_gpio_wifi_irq>0)
        {
            gpio_free(sprd_3rdparty_gpio_wifi_irq);
        }
	
	platform_set_drvdata(pdev, NULL);
	free_netdev(netdev);	//add by chengwg.
	g_mac_dev = NULL;
	
	//caisf add 2013-02-15
    if(g_mac_net_stats)
	    memset(g_mac_net_stats, 0, sizeof(struct net_device_stats));
    g_mac_net_stats = NULL;

	//chenq add 0723
	if(g_itm_config_buf != NULL)
	{
		kfree(g_itm_config_buf);
		g_itm_config_buf = NULL;
	}

#ifdef 	IBSS_BSS_STATION_MODE
	//chenq add do ap list merge logic 2013-08-28
	if( g_user_getscan_aplist != NULL )
	{
		while(g_user_getscan_aplist != NULL)
		{
			tmp_addr = (UWORD32 *)g_user_getscan_aplist->bss_next;
			kfree(g_user_getscan_aplist->bss_curr);
			kfree(g_user_getscan_aplist);
			g_user_getscan_aplist = tmp_addr;
		}
	}
	g_user_getscan_aplist = NULL;
	g_link_list_bss_count = 0;
#endif

#ifdef TROUT_TRACE_DBG
	sprd_timer2_destroy();
#ifdef TRACE_LOG_TO_FILE	
	//chenq add 0912
	trout_trace_exit();
#endif	/*TRACE_LOG_TO_FILE */
#endif	/* TROUT_TRACE_DBG */

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
	/* when remove dirver we should change to DOZE by zhao */
	/*zhou huiquan add for powersave*/
	#ifdef IBSS_BSS_STATION_MODE 
	//g_trout_state = TROUT_DOZE;	/* useless  zhangzhao*/
	trout_awake_fn = NULL;
	#endif
#endif

	TROUT_DBG4("=======exit_mac_driver: use=%d=======\n", --g_use_count);
	
	TROUT_FUNC_EXIT;

#ifdef TROUT2_WIFI_IC
	/* Close wifi RF !!!*/
	//Set_Trout_RF_Stop(WIFI_MODULE);
	/* Must last step power off Trout2 IC !!!*/
	Set_Trout_PowerOff(WIFI_MODULE);
#endif

#ifdef TROUT_WIFI_NPI
    //npi_tx_data_release();

    wake_unlock(&wifi_npi_power_lock);
    wake_lock_destroy(&wifi_npi_power_lock);
#endif

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifdef WIFI_SLEEP_POLICY
	wake_lock_destroy(&buffer_frame_lock);  /*by keguang 2013.09.04*/
	wake_lock_destroy(&scan_lock);
	wake_lock_destroy(&scan_ap_lock); //by caisf 2013.09.29
	wake_lock_destroy(&deauth_err_lock); //by caisf 2013.09.29
	del_hs_wake_timer();                 //by caisf
        wake_lock_destroy(&handshake_frame_lock); //by caisf 2013.09.29 
#endif
#endif
	wake_lock_destroy(&reset_mac_lock);
/*leon liu added debug wake lock in ap mode*/
#ifdef BSS_ACCESS_POINT_MODE
	wake_unlock(&ap_lock);
	wake_lock_destroy(&ap_lock);
#endif
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
    set_trout_module_state(TROUT_MODULE_UNLOADED);
#endif
	return 0;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,8))
#ifdef CONFIG_ARCH_SC8825
#define REG_PIN_TROUT_WIFI_IRQ	REG_PIN_KEYOUT6
#endif
#endif

#ifdef IBSS_BSS_STATION_MODE
extern UWORD32 g_pwr_tpc_switch; //add for TPC switch mode
#endif

static int trout_wifi_probe(struct platform_device *pdev)
{
	int rc;
	struct net_device *netdev;
	struct trout_private *stp;
	mutex_lock(&open_mutex);
#if 0
	#ifdef USE_TROUT_PHONE
	//unsigned long gpio_cfg = MFP_CFG_X(GPIO142, AF0, DS1, F_PULL_NONE, S_PULL_NONE, IO_IE);
	#else
	//unsigned long gpio_cfg = MFP_CFG_X(GPIO136, AF0, DS1, F_PULL_NONE, S_PULL_NONE, IO_IE);
	#endif
#endif

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
    set_trout_module_state(TROUT_MODULE_LOADING);
#endif

#ifdef IBSS_BSS_STATION_MODE
	g_pwr_tpc_switch = 1; //unset_max_power, default enable TPC mode
#endif

	/*leon liu modifed gpio configuation*/
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,8))
	unsigned long gpio_cfg = (BITS_PIN_DS(1) | BITS_PIN_AF(3) | BIT_PIN_SLP_IE);
	
	SPRD_GPIO_REG_WRITEL(gpio_cfg, REG_PIN_TROUT_WIFI_IRQ);
	printk("%s: configuring trout wifi's irq pin as gpio input\n", __func__);
#endif
	sprd_3rdparty_gpio_wifi_irq = SPRD_TROUT_INT_GPIO;

#ifdef TROUT2_WIFI_IC
	/* Must first step power on Trout2 IC !!!*/
	if(!Set_Trout_PowerOn(WIFI_MODULE))
	{
		printk("%s: wifi module set trout power on failed!\n", __FUNCTION__);
		mutex_unlock(&open_mutex);
		return -1;
	}
    /* Down bt code for power sleep function */
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
	/* Move download bt code to power on function */
//    Set_Trout_Download_BT_Code();
#endif
	/* Open wifi RF !!!*/
//	Set_Trout_RF_Start(WIFI_MODULE);
#endif

	printk("chenq debug init_mac_driver in\n");
#ifdef TROUT_TRACE_DBG
	sprd_timer2_init();

#ifdef TRACE_LOG_TO_FILE
	//chenq add 0912
	if(trout_trace_init(0,0,0) != 0)
	{
		printk("unable to inti trace \n");
#ifdef TROUT2_WIFI_IC
		/* Close wifi RF !!!*/
		//Set_Trout_RF_Stop(WIFI_MODULE);
		/* Must last step power off Trout2 IC !!!*/
		Set_Trout_PowerOff(WIFI_MODULE);
#endif
		TROUT_FUNC_EXIT;
		mutex_unlock(&open_mutex);
		return -1;
	}	
#endif	/* TRACE_LOG_TO_FILE */
	//trace_func_init();
#endif

	TROUT_FUNC_ENTER;

	netdev = alloc_etherdev(sizeof(*stp));
	if(!netdev) 
	{
		TROUT_DBG1("unable to alloc new ethernet\n");
#ifdef TROUT2_WIFI_IC
		/* Close wifi RF !!!*/
		//Set_Trout_RF_Stop(WIFI_MODULE);
		/* Must last step power off Trout2 IC !!!*/
		Set_Trout_PowerOff(WIFI_MODULE);
#endif
		TROUT_FUNC_EXIT;
		mutex_unlock(&open_mutex);
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, netdev);
	pr_info("%s: pdev->dev: %p\n", __func__, pdev->dev);
	pr_info("%s: ndev: %p\n", __func__, netdev);

	//chenq add 0723
	g_itm_config_buf = (unsigned char *)kmalloc(0x7FF, GFP_KERNEL);
	if(!g_itm_config_buf)
	{
		TROUT_DBG1("unable to alloc new g_itm_config_buf!\n");
		free_netdev(netdev);
#ifdef TROUT2_WIFI_IC
		/* Close wifi RF !!!*/
		//Set_Trout_RF_Stop(WIFI_MODULE);
		/* Must last step power off Trout2 IC !!!*/
		Set_Trout_PowerOff(WIFI_MODULE);
#endif
		TROUT_FUNC_EXIT;
		mutex_unlock(&open_mutex);
        return -ENOMEM;
	}

	//SET_NETDEV_DEV(netdev, &(func->dev));
	stp = netdev_priv(netdev);
	stp->dev = netdev;
	/* init mutexs by zhao */
	init_rwsem(&stp->rst_semaphore);
	//mutex_init(&stp->rst_mutex);
	mutex_init(&stp->iphy_mutex);
	mutex_init(&stp->txsr_mutex);
	mutex_init(&stp->sm_mutex);
#ifdef WAKE_LOW_POWER_POLICY
	mutex_init(&stp->ps_mutex);
#endif

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifndef WIFI_SLEEP_POLICY
	//xuan.yang, 2013-09-17, init cur run mutex lock, and resume state.
	mutex_init(&stp->cur_run_mutex);
	stp->resume_complete_state = 1;
#endif
#endif

	//xuan yang, 2013-8-13, init wid mutex
	mutex_init(&stp->rst_wid_mutex);
	
	netdev->netdev_ops = &trout_netdev_ops;
	
	//chenq add
	/*leon liu added judgement for WEXT*/
#ifdef CONFIG_TROUT_WEXT
	netdev->wireless_handlers = &itm_iw_handler_def;
#endif
	#ifdef IBSS_BSS_STATION_MODE
	strncpy(netdev->name, "wlan%d", sizeof(netdev->name));
	#else
	/*
	 * leon liu modifed interface name in AP mode since 4.0 uses wlan0
	 * strncpy(netdev->name, "wlap%d", sizeof(netdev->name));
	 */
	strncpy(netdev->name, "wlan%d", sizeof(netdev->name));

	#endif
	
	netdev->name[sizeof(netdev->name) - 1] = '\0';
	netdev->watchdog_timeo = msecs_to_jiffies(500);	//timeout, add by chengwg!
	
	stp->sdio_data.func = NULL;//func;
	/*leon liu added special case for cfg80211*/
#ifdef CONFIG_CFG80211
	stp->sdio_data.func = trout_get_sdio_func();

	if (stp->sdio_data.func == NULL)
	{
			printk("No sdio func found for trout\n");
			free_netdev(netdev);
			mutex_unlock(&open_mutex);
			return -ENODEV;
	}

	rc = trout_wdev_alloc(stp, &stp->sdio_data.func->dev);	

	if (rc)
	{
			printk("Cannot alloc wireless device for trout\n");
			free_netdev(netdev);
			TROUT_FUNC_EXIT;
			mutex_unlock(&open_mutex);
			return rc;
	}

	{
		extern struct wireless_dev *g_wdev;

		//stp->wdev = g_wdev;
	printk("Linux wireless device(wdev %p stp %p g_wdev is %p netdev->ieee80211_ptr is %p, &stp->wdev is %p) registered succesfully\n", stp->wdev, stp, g_wdev, netdev->ieee80211_ptr, &stp->wdev);
	}
#endif

        if(sprd_3rdparty_gpio_wifi_irq>0)
        {
            gpio_free(sprd_3rdparty_gpio_wifi_irq);

            //sprd_mfp_config(&gpio_cfg, 1);
            rc =gpio_request(sprd_3rdparty_gpio_wifi_irq, "trout int");
            if(rc){
                TROUT_DBG1("cannot alloc gpio for trout irq\n");
                goto err_out0;
            }

            gpio_direction_input(sprd_3rdparty_gpio_wifi_irq);
			/*leon liu modified sprd_alloc_gpio_irq to __gpio_to_irq*/
			//rc = sprd_alloc_gpio_irq(SPRD_TROUT_INT_GPIO);
			rc = __gpio_to_irq(SPRD_TROUT_INT_GPIO);
            if(rc < 0){
                TROUT_DBG1("cannot alloc gpio for trout irq\n");
                gpio_free(sprd_3rdparty_gpio_wifi_irq);
                goto err_out0;
            }
        }
        else
        {
            printk("trout:sprd_3rdparty_gpio_wifi_irq[0x%x]\n",sprd_3rdparty_gpio_wifi_irq);
            goto err_out0;
        }

	netdev->irq = rc;       //get irq number.
	g_mac_dev = netdev;
	//caisf add 2013-02-15
    g_mac_net_stats = &stp->mac_net_stats;
    if(g_mac_net_stats)
    	memset(g_mac_net_stats, 0, sizeof(struct net_device_stats));

#ifdef TROUT_B2B_TEST_MODE
	b2b_init(netdev);
#endif
#ifdef TROUT_WIFI_NPI
    //npi_tx_data_init();
	//trout_rf_test_init(netdev);
#endif

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
	//leon liu added for powersave timer initialization 2013-4-1
	if (pstimer_init(&pstimer, DEFAULT_PS_TIMEOUT_MS, DEFAULT_PS_PKTS_THRESHOLD) < 0)
	{
		TROUT_DBG1("pstimer_init() failed!\n");
	}
#endif

	memset(gsb, 0, sizeof(*gsb));
	get_wifi_buf(&gsb->ps, &gsb->size);
	printk("RS_BUF start:%X, size:%X\n", gsb->ps, gsb->size);
	if(gsb->ps)
		alloc_sb_buf();
	
	//chenq add for trout wifi cfg 2013-01-10
	init_from_cfg();

	rc = mac_init_fn(netdev);
	if(rc < 0){
		TROUT_DBG1("mac_init_fn() fail!\n");
		goto err_out1;
	}


	   //libing modify for fix switch bugger.
	    /* Call this function, it will set up all the required flags and bind    */
	    /* the driver with higher layer                                          */
	    ether_setup(netdev);

	TROUT_DBG4("register netdev .........\n");
	rc = register_netdev(netdev);
	if(rc == 0){
		TROUT_DBG4("%s: register netdev success!\n", __func__);
        
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
	//register_early_suspend(&trout_wifi_early_suspend_handler);
#endif
        TROUT_DBG4("%s: return ok.\n\r\n", __func__);

		/******************************/
		//if(stp->event_wq == NULL)
	    //{
//		    stp->event_wq = create_workqueue("event_wq");
            //Hugh: only schedul on a core.
		    stp->event_wq = create_singlethread_workqueue("event_wq");
    	    INIT_WORK(&stp->event_work, mac_event_work);
        //}
		

		#ifndef MAC_HW_UNIT_TEST_MODE
#ifdef MAC_P2P //wxb modify
		config_op_params(&g_mac);
#endif /* ifdef MAC_P2P */
		#else /* MAC_HW_UNIT_TEST_MODE */
	    	enable_operation(&g_mac);

	    /* Once all initializations are complete the PA control register is      */
	    /* programmed to enable PHY and MAC H/w.                                 */
	    enable_machw_phy_and_pa();

	    /* Start MAC controller task */
	    start_mac_controller();
		#endif /* MAC_HW_UNIT_TEST_MODE */

		#ifdef TROUT2_WIFI_FPGA
			#ifdef TROUT2_WIFI_FPGA_RF55
				host_write_trout_reg(0x800000, (((UWORD32)0x62)<<2));//dumy add for liusy test 0705  :: 07/11 tx inv enable
				host_write_trout_reg(0x20, (((UWORD32)0x63)<<2));//dumy add for liusy test 0705  :: 07/11 rx inv enable
				#ifdef TROUT_SDIO_INTERFACE
	    		//host_write_trout_reg(0x0A, (((UWORD32)0xc8)<<2));//liusy test 0904  :: rx 0 phase / tx async use 180 phase
				//chenq mod 2013-01-25 for BT context
	 				host_write_trout_reg(0x4000000A, (((UWORD32)0xc8)<<2));//liusy test 0904  :: rx 0 phase / tx async use 180 phase
					host_write_trout_reg(0x02,(((UWORD32)0x4015)<<2)); //delete arm7 reset ?
				#endif
			#endif /*TROUT2_WIFI_FPGA_RF55*/

			#ifdef TROUT2_WIFI_FPGA_RF2829
				host_write_trout_reg(0x00000010, (((UWORD32)0xc8)<<2));
			#endif /*TROUT2_WIFI_FPGA_RF2829*/

		#endif
		
		#ifdef TROUT2_WIFI_IC
			host_write_trout_reg(0x800000, (((UWORD32)0x62)<<2));//dumy add for liusy test 0705  :: 07/11 tx inv enable
			host_write_trout_reg(0x20, (((UWORD32)0x63)<<2));//dumy add for liusy test 0705  :: 07/11 rx inv enable
			#ifdef TROUT_SDIO_INTERFACE
	    		//host_write_trout_reg(0x0A, (((UWORD32)0xc8)<<2));//liusy test 0904  :: rx 0 phase / tx async use 180 phase
				//chenq mod 2013-01-25 for BT context
	 			host_write_trout_reg(0x4000000A, (((UWORD32)0xc8)<<2));//liusy test 0904  :: rx 0 phase / tx async use 180 phase
				host_write_trout_reg(0x02,(((UWORD32)0x4015)<<2)); //delete arm7 reset ?
			#endif
		#endif

		#ifdef TROUT_WIFI_FPGA
			//for FPGA-bit:trout_sdio_u2_ext40_tg1_newrf_1023.bit.
			#ifndef TROUT_SDIO_INTERFACE
    			host_write_trout_reg(0x02, (((UWORD32)0xc8)<<2));//liusy test 0904  :: rx 0 phase / tx async use 180 phase
			#else
   				host_write_trout_reg(0x02, (((UWORD32)0xc8)<<2));//liusy test 0904  :: rx / tx async use 180 phase
			#endif /* TROUT_SDIO_INTERFACE */
		#endif

		#ifdef TROUT_WIFI_EVB
			//for FPGA-bit:trout_sdio_u2_ext40_tg1_newrf_1023.bit.
			#ifndef TROUT_SDIO_INTERFACE
    			host_write_trout_reg(0x02, (((UWORD32)0xc8)<<2));//liusy test 0904  :: rx 0 phase / tx async use 180 phase
			#else
    			host_write_trout_reg(0x02, (((UWORD32)0xc8)<<2));//liusy test 0904  :: rx / tx async use 180 phase
			#endif /* TROUT_SDIO_INTERFACE */
		#endif

		//start_mem_adjust_task();
		#ifdef TROUT_TRACE_DBG
    		//start_trace_info_show_thread();
    		//trace_func_init();
    
    		//gpio_test_init();
		#endif
		/******************************/
#ifdef IBSS_BSS_STATION_MODE
		/* in station mode, before not connected to ap, initial the status to disconnectd */
		host_notify_arm7_connect_status(BFALSE);	
#else
		/* in ap mode, set the status to connected directly */
		host_notify_arm7_connect_status(BTRUE);
#endif
		host_notify_arm7_wifi_on(BTRUE);

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
	/*zhou huiquan add for protect r/w reg*/
	#ifdef IBSS_BSS_STATION_MODE
	g_trout_state = TROUT_AWAKE;
	//leon liu modified, use sta_wakup_combo instead of sta_awake 2013-4-4
	trout_awake_fn = sta_wakeup_combo;
	#endif
#endif
		wake_lock_init(&reset_mac_lock, WAKE_LOCK_SUSPEND, "itm_reset_mac");	//moved by chengwg 2013.08.05
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifdef WIFI_SLEEP_POLICY
		wake_lock_init(&scan_ap_lock, WAKE_LOCK_SUSPEND, "scan_ap_lock");	//by caisf 2013.09.29
		wake_lock_init(&deauth_err_lock, WAKE_LOCK_SUSPEND, "deauth_error_lock");	//by caisf 2013.09.29
                wake_lock_init(&handshake_frame_lock, WAKE_LOCK_SUSPEND, "handshake_frame_lock");    //by caisf 2013.10.04 
		wake_lock_init(&buffer_frame_lock, WAKE_LOCK_SUSPEND, "buffer_frame");  /*by keguang 2013.09.04*/
		wake_lock_init(&scan_lock, WAKE_LOCK_SUSPEND, "scan_in_process");	//moved by zhao
#endif
#endif
#ifdef WAKE_LOW_POWER_POLICY
		wake_low_power_fn = exit_low_power_mode;	//chwg add, 2013.12.6.
#endif
/*leon liu added debug ap wake lock*/
#ifdef BSS_ACCESS_POINT_MODE
		wake_lock_init(&ap_lock, WAKE_LOCK_SUSPEND, "itm_ap_lock");
		wake_lock(&ap_lock);
#endif
		#ifdef TROUT_WIFI_NPI
		//trout_rf_test_init(netdev);
		wake_lock_init(&wifi_npi_power_lock, WAKE_LOCK_SUSPEND, "wifi_npi_suspend");
		wake_lock(&wifi_npi_power_lock);
		#endif
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
       set_trout_module_state(TROUT_MODULE_LOADED);
#endif
		mutex_unlock(&open_mutex);
		TROUT_DBG4("=======init_mac_driver: use=%d=======\n", ++g_use_count);
		
		TROUT_FUNC_EXIT;
		return 0;
	}

err_out1:
	/*leon liu striiped sprd_free_gpio_irq*/
	//sprd_free_gpio_irq(netdev->irq);
        if(sprd_3rdparty_gpio_wifi_irq>0)
        {
            gpio_free(sprd_3rdparty_gpio_wifi_irq);
        }

err_out0:
/*leon liu added cleanup code for cfg80211*/
#ifdef CONFIG_CFG80211
	if (stp->wdev != NULL)
	{
		trout_wdev_free(stp->wdev);
	}
#endif
	free_netdev(netdev);
	g_mac_dev = NULL;
	//caisf add 2013-02-15
    g_mac_net_stats = NULL;
	
	kfree(g_itm_config_buf);
	g_itm_config_buf = NULL;

#ifdef TROUT2_WIFI_IC
	/* Close wifi RF !!!*/
	//Set_Trout_RF_Stop(WIFI_MODULE);
	/* Must last step power off Trout2 IC !!!*/
	Set_Trout_PowerOff(WIFI_MODULE);
#endif
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
    set_trout_module_state(TROUT_MODULE_FAIL);
#endif
	mutex_unlock(&open_mutex);
	TROUT_FUNC_EXIT;
	
	TROUT_FUNC_EXIT;
    return rc;
}

static struct platform_driver trout_wifi_driver = {
	.probe		= trout_wifi_probe,
	.remove		= trout_wifi_remove,
	.driver		= {
		.name	= "trout_wifi",
		.owner	= THIS_MODULE,
		.pm 	= &trout_wifi_pm,
	},
};

#if 0
static int __init init_mac_driver(void)
{
#ifdef USE_INTERNAL_SDIO_SPI_IF 
    int ret;

#ifdef TROUT_SDIO_INTERFACE
	ret = sdio_register_driver(&sdio_trout_driver);
	TROUT_DBG4("trout init: ret=%d\n", ret);
	if (ret < 0)
		return ret;
#else
	ret = spi_register_driver(&sprd_trout_driver);
	TROUT_DBG4("trout init: ret=%d\n", ret);
	if (ret < 0)
		return ret;
#endif

    return 0;

#else /* USE_INTERNAL_SDIO_SPI_IF */
	int rc;
	struct net_device *netdev;
	struct trout_private *stp;

	#ifdef USE_TROUT_PHONE
	unsigned long gpio_cfg = MFP_CFG_X(GPIO142, AF0, DS1, F_PULL_NONE, S_PULL_NONE, IO_IE);
	#else
	unsigned long gpio_cfg = MFP_CFG_X(GPIO136, AF0, DS1, F_PULL_NONE, S_PULL_NONE, IO_IE);
	#endif

	printk("chenq debug init_mac_driver in\n");
#ifdef TROUT_TRACE_DBG
	sprd_timer2_init();

#ifdef TRACE_LOG_TO_FILE
	//chenq add 0912
	if(trout_trace_init(0,0,0) != 0)
	{
		printk("unable to inti trace \n");
		TROUT_FUNC_EXIT;
		return -1;
	}	
#endif	/* TRACE_LOG_TO_FILE */
	//trace_func_init();
#endif

	TROUT_FUNC_ENTER;

	netdev = alloc_etherdev(sizeof(*stp));
	if(!netdev) 
	{
		TROUT_DBG1("unable to alloc new ethernet\n");
		TROUT_FUNC_EXIT;
		return -ENOMEM;
	}

	//chenq add 0723
	g_itm_config_buf = (unsigned char *)kmalloc(0x7FF, GFP_KERNEL);
	if(!g_itm_config_buf)
	{
		TROUT_DBG1("unable to alloc new g_itm_config_buf!\n");
		free_netdev(netdev);
		TROUT_FUNC_EXIT;
        return -ENOMEM;
	}

	//SET_NETDEV_DEV(netdev, &(func->dev));
	stp = netdev_priv(netdev);
	stp->dev = netdev;
	
	netdev->netdev_ops = &trout_netdev_ops;
	
	//chenq add
	netdev->wireless_handlers = &itm_iw_handler_def;
	#ifdef IBSS_BSS_STATION_MODE
	strncpy(netdev->name, "wlan%d", sizeof(netdev->name));
	#else
	strncpy(netdev->name, "wlap%d", sizeof(netdev->name));
	#endif
	
	netdev->name[sizeof(netdev->name) - 1] = '\0';
	netdev->watchdog_timeo = msecs_to_jiffies(500);	//timeout, add by chengwg!
	
	stp->sdio_data.func = NULL;//func;

    gpio_free(SPRD_TROUT_INT_GPIO);

	sprd_mfp_config(&gpio_cfg, 1);
	rc =gpio_request(SPRD_TROUT_INT_GPIO, "trout int");
	if(rc){
		TROUT_DBG1("cannot alloc gpio for trout irq\n");
		goto err_out0;
	}
	
	gpio_direction_input(SPRD_TROUT_INT_GPIO);
	rc = sprd_alloc_gpio_irq(SPRD_TROUT_INT_GPIO);
	if(rc < 0){
		TROUT_DBG1("cannot alloc gpio for trout irq\n");
		gpio_free(SPRD_TROUT_INT_GPIO);
		goto err_out0;
	}

	netdev->irq = rc;       //get irq number.
	g_mac_dev = netdev;
	//caisf add 2013-02-15
    g_mac_net_stats = &stp->mac_net_stats;
    if(g_mac_net_stats)
    	memset(g_mac_net_stats, 0, sizeof(struct net_device_stats));

#ifdef TROUT_B2B_TEST_MODE
	b2b_init(netdev);
#endif

	//chenq add for trout wifi cfg 2013-01-10
	init_from_cfg();

	rc = mac_init_fn(netdev);
	if(rc < 0){
		TROUT_DBG1("mac_init_fn() fail!\n");
		goto err_out1;
	}

	rc = register_netdev(netdev);
	if(rc == 0){
		TROUT_DBG4("%s: register netdev success!\n", __func__);

        TROUT_DBG4("%s: return ok.\n\r\n", __func__);
        
		itm_mac_close1_open2 = 2;
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
		/*register_early_suspend(&trout_wifi_early_suspend_handler);*/
#endif
		mac_open(netdev);

		printk("=======init_mac_driver: use=%d=======\n", ++g_use_count);
		
		TROUT_FUNC_EXIT;
		return 0;
	}
	
err_out1:
	sprd_free_gpio_irq(netdev->irq);
	gpio_free(SPRD_TROUT_INT_GPIO);

err_out0:
	free_netdev(netdev);
	g_mac_dev = NULL;
	//caisf add 2013-02-15
    g_mac_net_stats = NULL;
	
	kfree(g_itm_config_buf);
	g_itm_config_buf = NULL;
	
	TROUT_FUNC_EXIT;
    return rc;

#endif
}
#else
static int __init init_mac_driver(void)
{
	int rc;
	driver_exit = 0;
	meter_init();
#ifdef USE_INTERNAL_SDIO_SPI_IF 
    int ret;

#ifdef TROUT_SDIO_INTERFACE
	ret = sdio_register_driver(&sdio_trout_driver);
	TROUT_DBG4("trout init: ret=%d\n", ret);
	if (ret < 0)
		return ret;
#else
	ret = spi_register_driver(&sprd_trout_driver);
	TROUT_DBG4("trout init: ret=%d\n", ret);
	if (ret < 0)
		return ret;
#endif
    trout_rw_init();
    return 0;
#else /* USE_INTERNAL_SDIO_SPI_IF */
	printk("[version]\tTrout Wi-Fi module (%s %s)\n", __DATE__, __TIME__);
        pdev = platform_device_register_simple("trout_wifi", 0, NULL, 0); 
	if (IS_ERR(pdev)) {
		return PTR_ERR(pdev); 
	}
	rc = platform_driver_register(&trout_wifi_driver);
	
	if(rc >= 0 ){
		trout_rw_init();
		trout_runmode_init();
	}
	return rc;
#endif
}

#endif


#ifndef DV_SIM
module_init(init_mac_driver);
#else
module_init(spi_trout_probe);
#endif

/*****************************************************************************/
/*                                                                           */
/*  Function Name : exit_mac_driver                                          */
/*                                                                           */
/*  Description   : This is the function which is called when halting.       */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*  Processing    : None                                                     */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
#ifndef DV_SIM

#if 0
static void __exit exit_mac_driver(void)
{
#ifdef USE_INTERNAL_SDIO_SPI_IF	
#ifdef TROUT_SDIO_INTERFACE
	sdio_unregister_driver(&sdio_trout_driver);
#else
	spi_unregister_driver(&sprd_trout_driver);
#endif
#else /* USE_INTERNAL_SDIO_SPI_IF */

	struct net_device *netdev = g_mac_dev;//NULL
	//struct trout_private *stp = dev_get_drvdata(&(func->dev));

	TROUT_FUNC_ENTER;

	TROUT_DBG4("mac driver exit...\n");

	if(netdev == NULL)
	{
		TROUT_DBG1("%s: error! netdev(%p) is NULL!\n", __FUNCTION__, netdev);
		return;
	}

	//chenq add 2012-10-18
	itm_mac_close1_open2 = 1;
	mac_close(netdev);
	//zhou add
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
	unregister_early_suspend(&trout_wifi_early_suspend_handler);
#endif

	//chenq add
	/* Clean up netdev. */
	unregister_netdev(netdev);
#ifdef CONFIG_CFG80211
	if (stp->wdev != NULL)
	{
		trout_wdev_free(stp->wdev);
	}
#endif
	free_mem_regions();
	/*sprd_free_gpio_irq(netdev->irq);*/
	gpio_free(SPRD_TROUT_INT_GPIO);
	free_netdev(netdev);	//add by chengwg.
	g_mac_dev = NULL;
	//caisf add 2013-02-15
    if(g_mac_net_stats)
	    memset(g_mac_net_stats, 0, sizeof(struct net_device_stats));
    g_mac_net_stats = NULL;

	//chenq add 0723
	if(g_itm_config_buf != NULL)
	{
		kfree(g_itm_config_buf);
		g_itm_config_buf = NULL;
	}
#ifdef TROUT_TRACE_DBG
	sprd_timer2_destroy();
#ifdef TRACE_LOG_TO_FILE	
	//chenq add 0912
	trout_trace_exit();
#endif	/*TRACE_LOG_TO_FILE */
#endif	/* TROUT_TRACE_DBG */

	printk("=======exit_mac_driver: use=%d=======\n", --g_use_count);
	
	TROUT_FUNC_EXIT;

#endif	
}
#else

static void __exit exit_mac_driver(void)
{
	printk("[exit_mac_driver]\t>>>\n");
	mutex_lock(&open_mutex);

#ifdef USE_INTERNAL_SDIO_SPI_IF	
#ifdef TROUT_SDIO_INTERFACE
	sdio_unregister_driver(&sdio_trout_driver);
#else
	spi_unregister_driver(&sprd_trout_driver);
#endif
#else /* USE_INTERNAL_SDIO_SPI_IF */
	platform_driver_unregister(&trout_wifi_driver);
	platform_device_unregister(pdev);
	pdev = NULL;
#endif	
	driver_exit = 1;
	trout_rw_exit();
	trout_runmode_exit();
	meter_exit();
	mutex_unlock(&open_mutex);
	printk("[exit_mac_driver]\t<<<\n");
}

#endif

#else
static void __exit exit_mac_driver(void)
{
	unregister_netdev(g_mac_dev);
}
#endif

module_exit(exit_mac_driver);

#ifdef MAC_P2P
/*****************************************************************************/
/* NOTE ON module switching in linux                                         */
/* -----------------------------------                                       */
/* Ittiam MAC SW runs as a driver(KO) in linux. To switch modules we need to */
/* remove (rmmod) and insert (insmod) another module. This can be done by an */
/* user space application only.                                              */
/* Currently this is being done bye using call_usermodehelper()              */
/* This method seems to be very standard method used by drivers to implement */
/* user space function call. However this function cannot be called from     */
/* atomic (uninterruptable) code. Since our MAC SW runs as a [tasklet]       */
/* (SW interrupt) in Linux, it is atomic and we cannot call this function    */
/* directly from any of the MAC control code.                                */
/* To get around this problem, we need to schedule a [work], from where the  */
/* user space application is called.                                         */
/* This is the major difference between a tasklet and a work :-              */
/* [Tasklets run in software interrupt context with the result that all      */
/* tasklet code must be atomic. Instead, workqueue functions run in the      */
/* context of a special kernel process; as a result, they have more          */
/* flexibility. In particular, workqueue functions can sleep.]               */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/
/*                                                                           */
/*  Function Name : switch_module                                            */
/*                                                                           */
/*  Description   : This is the call back function for the WORK to switch    */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*  Processing    : This function spawns an user space function to do the    */
/*                  the module switching for P2P in linux                    */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
//static void switch_module(void *junk) //caisf mod
static void switch_module(struct work_struct *work)
{
    char *argv[3] = {NULL, NULL, NULL};
    char *envp[3] = {NULL, NULL, NULL};

    printk("spawning user space application switch module\n");

    /*************************************************************************/
    /* Note on call_usermodehelper()                                         */
    /* static inline int call_usermodehelper(char *path, char **argv,        */
    /*                                       char **envp, int wait)          */
    /* argv[0] - the user space application to do the actual rmmod/insmod    */
    /* envp[0/1] - minimal command environment (Not much known)              */
    /*************************************************************************/
#if 1 //caisf mod
	// 20120709 caisf add, merged ittiam mac v1.2 code
    //argv[0] = "/test.out";
    //argv[0] = "/bin/switch_module.out";
    argv[0] = "/system/lib/modules/switch_module.out";

#ifdef BSS_ACCESS_POINT_MODE
    argv[1] = "sta";
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef IBSS_BSS_STATION_MODE
    argv[1] = "ap";
#endif /* IBSS_BSS_STATION_MODE */

#else
    argv[0] = "sh";

    argv[1] = "/system/bin/switch_module.out";
#endif

    envp[0] = "HOME=/root/";
    envp[1] = "PATH=/sbin:/bin:/usr/sbin:/usr/bin";

    printk("%s %s %s %s %s\n", argv[0], argv[1], envp[0], envp[1], envp[2]);

#if 1 //caisf mod 
    call_usermodehelper(argv[0], argv, envp, 0);
#else
    {
	struct subprocess_info *info;

	// GFP_ATOMIC/GFP_KERNEL
	info = call_usermodehelper_setup(argv[0], argv, envp, GFP_KERNEL);
    if(info == NULL)
    {
        printk("switch_module failed! call_usermodehelper_setup error!\n");
        return;
    }
    
    call_usermodehelper_exec(info,UMH_NO_WAIT);
    }
#endif

    printk("spawning complete\n");
}

static struct work_struct mod_switch;
/*****************************************************************************/
/*                                                                           */
/*  Function Name : schedule_mod_switch                                      */
/*                                                                           */
/*  Description   : This function is to be called from the MAC SW to schedule*/
/*                  a module switching                                       */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : mod_switch (static)                                      */
/*  Processing    : This function initializes and schedules the work         */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void schedule_mod_switch(void)
{
    printk("\nscheduling work\n");

    /*************************************************************************/
    /* INIT_WORK(struct work_struct *work,                                   */
    /*           void (*function)(void *), void *data);                      */
    /* work - work structure                                                 */
    /* function - kernel mode function to be scheduled                       */
    /* data - Not known, being set to NULL                                   */
    /*************************************************************************/

    INIT_WORK(&mod_switch, switch_module);

    if(schedule_work_on(0, &mod_switch))
    {
        printk("work scheduled\n");
    }
    else
    {
        printk("work not scheduled\n");
    }
}
#endif /* MAC_P2P */

MODULE_DESCRIPTION("TROUT WIFI driver");
MODULE_AUTHOR("Spreadtrum Inc.");
MODULE_LICENSE("GPL");
MODULE_VERSION(TROUT_WIFI_VERSION);


#endif /* OS_LINUX_CSL_TYPE */
