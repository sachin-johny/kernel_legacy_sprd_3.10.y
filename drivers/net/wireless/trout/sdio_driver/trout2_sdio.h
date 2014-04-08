#ifndef _TROUT2_SDIO_H
#define _TROUT2_SDIO_H
#define TROUT_SDIO_VERSION	"0.27"
#define TROUT_SDIO_BLOCK_SIZE	512	/* rx fifo max size in bytes */

//#define POW_CTRL_VALUE		0x700 /*for Trout1*/ 
#define POW_CTRL_VALUE		0x500 /*for Trout2*/ 

#define TROUT_SHARE_MEM_BASE_ADDR 0x10000	/* word address */
#define TROUT_BT_RAM_BASE_ADDR    0x5000	/* word address */

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE 
/*zhou huiquan add for protect read/write register*/
#define TROUT_AWAKE                      1
#define TROUT_DOZE                 	 0
/*leon liu added TROUT_SLEEP state*/
#define TROUT_SLEEP			 2	
#define LOCK_TURE                        1
#define LOCK_FALSE                       0
#define SYS_ADDR_MAX                     (0x1FF<<2)
#define SYS_ADDR_MIN                     0
#define FM_ADDR_MIN                      (0x3000<<2)
#define FM_ADDR_MAX                      (0x3FFF<<2)
#define UWORD32                          unsigned int

typedef void (*sdio_trout_awake)(bool flag);

extern sdio_trout_awake trout_awake_fn;
extern unsigned char g_trout_state;
void root_wifimac_wakeup(void);
unsigned int host_write_trout_ram(void *trout_addr, void *host_addr,unsigned int length);
#endif

extern struct sdio_func *trout_sdio_func;
extern struct sdio_func *cur_sdio_func;/* Current need  */
extern struct mutex rw_reg_mutex;
#define TROUT_MODULE_FAIL                0
#define TROUT_MODULE_LOADING             1
#define TROUT_MODULE_LOADED              2
#define TROUT_MODULE_UNLOADING           3
#define TROUT_MODULE_UNLOADED            4
#define TROUT_MODULE_IDLE                5
#endif
