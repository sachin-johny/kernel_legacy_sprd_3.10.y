/*
 * Driver for Pixcir I2C touchscreen controllers.
 *
 * Copyright (C) 2010-2011 Pixcir, Inc.
 *
 * pixcir_i2c_ts.c V3.0	from v3.0 support TangoC solution and remove the previous soltutions
 *
 * pixcir_i2c_ts.c V3.1	Add bootloader function	7
 *			Add RESET_TP		9
 * 			Add ENABLE_IRQ		10
 *			Add DISABLE_IRQ		11
 * 			Add BOOTLOADER_STU	16
 *			Add ATTB_VALUE		17
 *			Add Write/Read Interface for APP software
 *
 * pixcir_i2c_ts.c V3.2.0A	for INT_MODE 0x0A
 *				arrange to pixcir 10 slot
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */ 

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/timer.h>
#include <linux/gpio.h>

#include <linux/sysfs.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <mach/board.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <mach/gpio.h>

#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/string.h>
#include <asm/unistd.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/earlysuspend.h>
#include <linux/input.h>
#include <mach/regulator.h>
#include <linux/regulator/consumer.h>
#include <asm/atomic.h>
#include <linux/i2c/msg21xx_i2c_ts.h>
/*********************************Bee-0928-TOP****************************************/
//#define PIXCIR_DEBUG
//#define PIXCIR_CQ_CALL		0
#ifdef PIXCIR_DEBUG
#define PIXCIR_DBG(format, ...)	\
		printk(KERN_INFO "PIXCIR_TS " format "\n", ## __VA_ARGS__)
#define	TPD_DEBUG(format, ...)	\
		printk(KERN_INFO "PIXCIR_TS " format "\n", ## __VA_ARGS__)
#else
#define PIXCIR_DBG(format, ...)
#define TPD_DEBUG(format, ...)
#endif


#define SLAVE_ADDR		0x48
#define	BOOTLOADER_ADDR		0x5d

#define MS_TS_MSG21XX_X_MAX 320
#define MS_TS_MSG21XX_Y_MAX 480

#ifndef I2C_MAJOR
#define I2C_MAJOR 		125
#endif

#define I2C_MINORS 		256

#define	CALIBRATION_FLAG	1
#define	BOOTLOADER		7
#define RESET_TP		9

#define	ENABLE_IRQ		10
#define	DISABLE_IRQ		11
#define	BOOTLOADER_STU		16
#define ATTB_VALUE		17

#define	MAX_FINGER_NUM		5
#define X_OFFSET		30
#define Y_OFFSET		40
#define TPD_OK 0
 
#define TPD_REG_BASE 0x00
#define TPD_SOFT_RESET_MODE 0x01
#define TPD_OP_MODE 0x00
#define TPD_LOW_PWR_MODE 0x04
#define TPD_SYSINFO_MODE 0x10
#define GET_HSTMODE(reg)  ((reg & 0x70) >> 4)  // in op mode or not 
#define GET_BOOTLOADERMODE(reg) ((reg & 0x10) >> 4)  // in bl mode 

struct regulator *regu;
struct regulator *regu1;

//#define	TP_FIRMWARE_UPDATE

#ifdef TP_FIRMWARE_UPDATE
#define U8 unsigned char
#define S8 signed char
#define U16 unsigned short
#define S16 signed short
#define U32 unsigned int
#define S32 signed int
//#define TOUCH_ADDR_MSG20XX   0x4C
//#define FW_ADDR_WRITE_MSG21XX      0xC4
//#define FW_ADDR_READ_MSG21XX        0xC5
#define TOUCH_ADDR_MSG21XX   0x26
#define FW_ADDR_MSG21XX      0x62		//0xC4
#define FW_UPDATE_ADDR_MSG21XX   0x49		//0x92
//#define FW_UPDATE_ADDR_WRITE_MSG21XX   0x92
//#define FW_UPDATE_ADDR_READ_MSG21XX   0x93
static  char *fw_version;
static U8 temp[94][1024];
static int FwDataCnt;
struct class *firmware_class;
struct device *firmware_cmd_dev;
static int update_switch = 0;

#define pr_tp(format, args...)  do {} while (0)
#define pr_ch(format, args...)  do {} while (0)
#undef pr_k(format, args...)
#define pr_k(format, args...)  do {} while (0)
#endif	//TP_FIRMWARE_UPDAT


//tenggaoyun add
#define NEW_FUNC 1
#if NEW_FUNC
#include <linux/miscdevice.h>
#include <linux/wakelock.h>
#include <linux/types.h>

#define AL3006_PLS_DEVICE 		        "rohm_proximity"
#define AL3006_PLS_INPUT_DEV  	        "rohm_proximity"
#define AL3006_PLS_ADDRESS		        0x1C
#define AL3006_PLS_RETRY_COUNT	        3
#define LTR_IOCTL_MAGIC 				0x1C
#define LTR_IOCTL_GET_PFLAG  			_IOR(LTR_IOCTL_MAGIC, 1, int)
#define LTR_IOCTL_GET_LFLAG  			_IOR(LTR_IOCTL_MAGIC, 2, int)
#define LTR_IOCTL_SET_PFLAG  			_IOW(LTR_IOCTL_MAGIC, 3, int)
#define LTR_IOCTL_SET_LFLAG  			_IOW(LTR_IOCTL_MAGIC, 4, int)
#define LTR_IOCTL_GET_DATA  			_IOW(LTR_IOCTL_MAGIC, 5, unsigned char)
#define LTR_IOCTL_SET_LFLAG1                    _IOW(LTR_IOCTL_MAGIC, 15, int)


//REG
#define AL3006_PLS_REG_CONFIG		0x00
#define AL3006_PLS_REG_TIME_CTRL	0x01
#define AL3006_PLS_REG_DLS_CTRL		0x02
#define AL3006_PLS_REG_INT_STATUS	0x03
#define AL3006_PLS_REG_DPS_CTRL		0x04
#define AL3006_PLS_REG_DATA			0x05
#define AL3006_PLS_REG_DLS_WIN		0x08

//CMD
#define AL3006_PLS_BOTH_ACTIVE		0x02
#define AL3006_PLS_ALPS_ACTIVE		0x00
#define AL3006_PLS_PXY_ACTIVE		0x01
#define AL3006_PLS_BOTH_DEACTIVE	0x0B
#define	AL3006_PLS_INT_MASK		0x03
#define AL3006_PLS_DPS_INT			0x02
#define AL3006_PLS_DLS_INT			0x01
typedef enum _SENSOR_TYPE{
	AL3006_PLS_ALPS=0,
	AL3006_PLS_PXY,
	AL3006_PLS_BOTH,
}SENSOR_TYPE;


typedef  struct _al3006_pls_t{
	struct input_dev *input;
	struct i2c_client *client;
	struct work_struct	work;
	struct workqueue_struct *ltr_work_queue;
	struct early_suspend ltr_early_suspend;
}al3006_pls_t;


static u8 tpd_proximity_flag = 0;
static u8 tpd_proximity_detect = 1;  //0-->close ; 1--> far away       00
static u8 tpd_als_opened = 0;   
struct input_dev *input_dev;
struct timer_list ps_timer;
static struct wake_lock pls_delayed_work_wake_lock;
static int debug_level=1;
static int pre_value=0xff;
#endif

#define REG_RT_PRIO(x) ((x) | 0x10000000)
#define RTPM_PRIO_TPD                       REG_RT_PRIO(4)
static unsigned char  bl_cmd[] = {
	 0x00, 0xFF, 0xA5,
	 0x00, 0x01, 0x02,
	 0x03, 0x04, 0x05,
	 0x06, 0x07};
	 //exit bl mode 
	 struct tpd_operation_data_t{
		 unsigned char  hst_mode;
		 unsigned char  tt_mode;
		 unsigned char  tt_stat;
		 
		 unsigned char  x1_M,x1_L;
		 unsigned char  y1_M,y1_L;
		 unsigned char  z1;
		 unsigned char  evnt_id;
	 
		 unsigned char  x2_M,x2_L;
		 unsigned char  y2_M,y2_L;
		 unsigned char  r_0d;
		 unsigned char  gest_cnt;
		 unsigned char  gest_id;
	 };
	 struct tpd_bootloader_data_t{
		 unsigned char  bl_file;
		 unsigned char  bl_status;
		 unsigned char  bl_error;
		 unsigned char  blver_hi,blver_lo;
		 unsigned char  bld_blver_hi,bld_blver_lo;
	 
		 unsigned char  ttspver_hi,ttspver_lo;
		 unsigned char  appid_hi,appid_lo;
		 unsigned char  appver_hi,appver_lo;
	 
		 unsigned char  cid_0;
		 unsigned char  cid_1;
		 unsigned char  cid_2;
		 
	 };
	 struct tpd_sysinfo_data_t{
				  unsigned char    hst_mode;
				  unsigned char   mfg_cmd;
				  unsigned char   mfg_stat;
				  unsigned char  cid[3];
				  unsigned char  tt_undef1;
 
				  unsigned char  uid[8];
				  unsigned char   bl_verh;
				  unsigned char   bl_verl;
 
				  unsigned char  tts_verh;
				  unsigned char  tts_verl;

				  unsigned char  app_idh;
				   unsigned char  app_idl;
				   unsigned char  app_verh;
				   unsigned char  app_verl;

				   unsigned char  tt_undef2[6];
				   unsigned char   act_intrvl;
				   unsigned char   tch_tmout;
				   unsigned char   lp_intrvl;
	 
				  
				 
					  
	 };
struct touch_info {
    int x1, y1;
    int x2, y2;
	int x3, y3;
    int p1, p2,p3;
    int count;
};
struct TouchScreenInfo_t{
    unsigned char nTouchKeyMode;
    unsigned char nTouchKeyCode;
    //unsigned char nFingerNum;
} ;


static unsigned char status_reg = 0;
int global_irq;

struct timer_list	tp_timer;		/* "no irq" timer */


static struct i2c_driver pixcir_i2c_ts_driver;
static struct class *i2c_dev_class;
static LIST_HEAD( i2c_dev_list);
static DEFINE_SPINLOCK( i2c_dev_list_lock);

#define TOUCH_VIRTUAL_KEYS
static struct i2c_client *this_client;
static int pixcir_irq;
static int suspend_flag;
static struct early_suspend	pixcir_early_suspend;

#ifdef PIXCIR_CQ_CALL
static struct task_struct *thread = NULL;
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static int tpd_flag=0;
static int tpd_suspend_flag=0;
#endif


static ssize_t pixcir_set_calibrate(struct device* cd, struct device_attribute *attr,
		       const char* buf, size_t len);
static ssize_t pixcir_show_suspend(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t pixcir_store_suspend(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static void pixcir_reset(void);
static void pixcir_ts_suspend(struct early_suspend *handler);
static void pixcir_ts_resume(struct early_suspend *handler);
static void pixcir_ts_pwron(void);
static void pixcir_ts_pwroff(void);
 static struct tpd_operation_data_t g_operation_data;
 static struct tpd_bootloader_data_t g_bootloader_data;
 static struct tpd_sysinfo_data_t g_sysinfo_data;

//extern unsigned char poweroff_ctp_fm_flag;
static void sy_init();


static int pixcir_i2c_rxdata(char *rxdata, int length)
{
        int ret;
        struct i2c_msg msgs[] = {
                /*
                {
                        .addr   = this_client->addr,
                        .flags  = 0,
                        .len    = 1,
                        .buf    = rxdata,
                },
                */
                {
                        .addr   = this_client->addr,
                        .flags  = I2C_M_RD,
                        .len    = length,
                        .buf    = rxdata,
                },
        };

        ret = i2c_transfer(this_client->adapter, msgs,2);
        if (ret < 0)
                pr_err("%s i2c read error: %d\n", __func__, ret);
        
        return ret;
}


static int pixcir_i2c_txdata(char *txdata, int length)
{
		int ret;
		struct i2c_msg msg[] = {
			{
				.addr	= this_client->addr,
				.flags	= 0,
				.len		= length,
				.buf		= txdata,
			},
		};

		ret = i2c_transfer(this_client->adapter, msg, 1);
		if (ret < 0)
			pr_err("%s i2c write error: %d\n", __func__, ret);

		return ret;
}

static int pixcir_i2c_write_data(unsigned char addr, unsigned char data)
{
	unsigned char buf[2];
	buf[0]=addr;
	buf[1]=data;
	return pixcir_i2c_txdata(buf, 2); 
}



static bool msg2133_i2c_read(char *pbt_buf, int dw_lenth)
{
    int ret;
    //    pr_ch("The msg_i2c_client->addr=0x%x\n",i2c_client->addr);
    ret = i2c_master_recv(this_client, pbt_buf, dw_lenth);

    if(ret <= 0)
    {
        //pr_tp("msg_i2c_read_interface error\n");
        return false;
    }

    return true;
}

static bool msg2133_i2c_write(char *pbt_buf, int dw_lenth)
{
    int ret;
    //    pr_ch("The msg_i2c_client->addr=0x%x\n",i2c_client->addr);
    ret = i2c_master_send(this_client, pbt_buf, dw_lenth);

    if(ret <= 0)
    {
        //pr_tp("msg_i2c_read_interface error\n");
        return false;
    }

    return true;
}


#ifdef TP_FIRMWARE_UPDATE
static void i2c_read_msg2133(U8 *pbt_buf, int dw_lenth)
{
    this_client->addr = FW_ADDR_MSG21XX;
	i2c_master_recv(this_client, pbt_buf, dw_lenth);	//0xC5_8bit
	this_client->addr = TOUCH_ADDR_MSG21XX;
}

static void i2c_write_msg2133(U8 *pbt_buf, int dw_lenth)
{

	this_client->addr = FW_ADDR_MSG21XX;
	i2c_master_send(this_client, pbt_buf, dw_lenth);		//0xC4_8bit
	this_client->addr = TOUCH_ADDR_MSG21XX;
}

static void i2c_read_update_msg2133(U8 *pbt_buf, int dw_lenth)
{	

	this_client->addr = FW_UPDATE_ADDR_MSG21XX;
	i2c_master_recv(this_client, pbt_buf, dw_lenth);	//0x93_8bit
	this_client->addr = TOUCH_ADDR_MSG21XX;
}

static void i2c_write_update_msg2133(U8 *pbt_buf, int dw_lenth)
{	
    this_client->addr = FW_UPDATE_ADDR_MSG21XX;
	i2c_master_send(this_client, pbt_buf, dw_lenth);	//0x92_8bit
	this_client->addr = TOUCH_ADDR_MSG21XX;
}



void dbbusDWIICEnterSerialDebugMode(void)
{
    U8 data[5];
    // Enter the Serial Debug Mode
    data[0] = 0x53;
    data[1] = 0x45;
    data[2] = 0x52;
    data[3] = 0x44;
    data[4] = 0x42;
    i2c_write_msg2133(data, 5);
}

void dbbusDWIICStopMCU(void)
{
    U8 data[1];
    // Stop the MCU
    data[0] = 0x37;
    i2c_write_msg2133(data, 1);
}

void dbbusDWIICIICUseBus(void)
{
    U8 data[1];
    // IIC Use Bus
    data[0] = 0x35;
    i2c_write_msg2133(data, 1);
}

void dbbusDWIICIICReshape(void)
{
    U8 data[1];
    // IIC Re-shape
    data[0] = 0x71;
    i2c_write_msg2133(data, 1);
}

void dbbusDWIICIICNotUseBus(void)
{
    U8 data[1];
    // IIC Not Use Bus
    data[0] = 0x34;
    i2c_write_msg2133(data, 1);
}

void dbbusDWIICNotStopMCU(void)
{
    U8 data[1];
    // Not Stop the MCU
    data[0] = 0x36;
    i2c_write_msg2133(data, 1);
}

void dbbusDWIICExitSerialDebugMode(void)
{
    U8 data[1];
    // Exit the Serial Debug Mode
    data[0] = 0x45;
    i2c_write_msg2133(data, 1);
    // Delay some interval to guard the next transaction
}

void drvISP_EntryIspMode(void)
{
    U8 bWriteData[5] =
    {
        0x4D, 0x53, 0x54, 0x41, 0x52
    };
    i2c_write_update_msg2133(bWriteData, 5);
    msleep(10);           // delay about 10ms
}

void drvISP_WriteEnable(void)
{
    U8 bWriteData[2] =
    {
        0x10, 0x06
    };
    U8 bWriteData1 = 0x12;
    i2c_write_update_msg2133(bWriteData, 2);
    i2c_write_update_msg2133(&bWriteData1, 1);
}

U8 drvISP_Read(U8 n, U8 *pDataToRead)    //First it needs send 0x11 to notify we want to get flash data back.
{
    U8 Read_cmd = 0x11;
    U8 i = 0;
    unsigned char dbbus_rx_data[16] = {0};
    i2c_write_update_msg2133(&Read_cmd, 1);
    //if (n == 1)
    {
        i2c_read_update_msg2133(&dbbus_rx_data[0], n + 1);

        for(i = 0; i < n; i++)
        {
            *(pDataToRead + i) = dbbus_rx_data[i + 1];
        }
    }
    //else
    {
        //     i2c_read_update_msg2133(pDataToRead, n);
    }
    return 0;
}

U8 drvISP_ReadStatus(void)
{
    U8 bReadData = 0;
    U8 bWriteData[2] =
    {
        0x10, 0x05
    };
    U8 bWriteData1 = 0x12;
    msleep(1);           // delay about 100us
    i2c_write_update_msg2133(bWriteData, 2);
    msleep(1);           // delay about 100us
    drvISP_Read(1, &bReadData);
    msleep(10);           // delay about 10ms
    i2c_write_update_msg2133(&bWriteData1, 1);
    return bReadData;
}



void drvISP_BlockErase(U32 addr)
{
    U8 bWriteData[5] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
    U8 bWriteData1 = 0x12;
    U32 timeOutCount=0;
	
    drvISP_WriteEnable();
    //Enable write status register
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x50;
    i2c_write_update_msg2133(bWriteData, 2);
    i2c_write_update_msg2133(&bWriteData1, 1);
    //Write Status
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x01;
    bWriteData[2] = 0x00;
    i2c_write_update_msg2133(bWriteData, 3);
    i2c_write_update_msg2133(&bWriteData1, 1);
    //Write disable
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x04;
    i2c_write_update_msg2133(bWriteData, 2);
    i2c_write_update_msg2133(&bWriteData1, 1);

    timeOutCount=0;
    msleep(1);           // delay about 100us
    while((drvISP_ReadStatus() & 0x01) == 0x01)
    {
        timeOutCount++;
	 if ( timeOutCount > 10000 ) 
            break; /* around 1 sec timeout */
    }

    //pr_ch("The drvISP_ReadStatus3=%d\n", drvISP_ReadStatus());
    drvISP_WriteEnable();
    //pr_ch("The drvISP_ReadStatus4=%d\n", drvISP_ReadStatus());
    bWriteData[0] = 0x10;
    bWriteData[1] = 0xC7;        //Block Erase
    //bWriteData[2] = ((addr >> 16) & 0xFF) ;
    //bWriteData[3] = ((addr >> 8) & 0xFF) ;
    // bWriteData[4] = (addr & 0xFF) ;
    i2c_write_update_msg2133(bWriteData, 2);
    //i2c_write_update_msg2133( &bWriteData, 5);
    i2c_write_update_msg2133(&bWriteData1, 1);

    timeOutCount=0;
    msleep(1);           // delay about 100us
    while((drvISP_ReadStatus() & 0x01) == 0x01)

    {
        timeOutCount++;
	 if ( timeOutCount > 10000 ) 
            break; /* around 1 sec timeout */
    }
}

void drvISP_Program(U16 k, U8 *pDataToWrite)
{
    U16 i = 0;
    U16 j = 0;
    //U16 n = 0;
    U8 TX_data[133];
    U8 bWriteData1 = 0x12;
    U32 addr = k * 1024;
#if 1

    for(j = 0; j < 8; j++)    //128*8 cycle
    {
        TX_data[0] = 0x10;
        TX_data[1] = 0x02;// Page Program CMD
        TX_data[2] = (addr + 128 * j) >> 16;
        TX_data[3] = (addr + 128 * j) >> 8;
        TX_data[4] = (addr + 128 * j);

        for(i = 0; i < 128; i++)
        {
            TX_data[5 + i] = pDataToWrite[j * 128 + i];
        }

        while((drvISP_ReadStatus() & 0x01) == 0x01)
        {
            ;    //wait until not in write operation
        }

        drvISP_WriteEnable();
        i2c_write_update_msg2133( TX_data, 133);   //write 133 byte per cycle
        i2c_write_update_msg2133(&bWriteData1, 1);
    }

#else

    for(j = 0; j < 512; j++)    //128*8 cycle
    {
        TX_data[0] = 0x10;
        TX_data[1] = 0x02;// Page Program CMD
        TX_data[2] = (addr + 2 * j) >> 16;
        TX_data[3] = (addr + 2 * j) >> 8;
        TX_data[4] = (addr + 2 * j);

        for(i = 0; i < 2; i++)
        {
            TX_data[5 + i] = pDataToWrite[j * 2 + i];
        }

        while((drvISP_ReadStatus() & 0x01) == 0x01)
        {
            ;    //wait until not in write operation
        }

        drvISP_WriteEnable();
        i2c_write_update_msg2133(TX_data, 7);    //write 7 byte per cycle
        i2c_write_update_msg2133(&bWriteData1, 1);
    }

#endif
}

void drvISP_ExitIspMode(void)
{
    U8 bWriteData = 0x24;
    i2c_write_update_msg2133(&bWriteData, 1);
}

static ssize_t firmware_update_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
	printk("tyd-tp: firmware_update_show\n");
    return sprintf(buf, "%s\n", fw_version);
}

static ssize_t firmware_update_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    U8 i;
    U8 dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};
    update_switch = 1;
    //drvISP_EntryIspMode();
    //drvISP_BlockErase(0x00000);
    //M by cheehwa _HalTscrHWReset();

    //关闭中断
    disable_irq_nosync(global_irq);
	
    pixcir_reset();
    msleep(500);			//delay_500ms	
    //msctpc_LoopDelay ( 100 );        // delay about 100ms*****
    // Enable slave's ISP ECO mode
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    //pr_ch("dbbusDWIICIICReshape\n");
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x08;
    dbbus_tx_data[2] = 0x0c;
    dbbus_tx_data[3] = 0x08;
    // Disable the Watchdog
    i2c_write_msg2133(dbbus_tx_data, 4);
    //Get_Chip_Version();
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    dbbus_tx_data[3] = 0x00;
    i2c_write_msg2133(dbbus_tx_data, 4);
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    i2c_write_msg2133(dbbus_tx_data, 4);
    //pr_ch("update\n");
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    i2c_write_msg2133(dbbus_tx_data, 4);
    //Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    i2c_write_msg2133(dbbus_tx_data, 4);
    //Enable SPI Pad
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x02;
    i2c_write_msg2133(dbbus_tx_data, 3);
    i2c_read_msg2133(&dbbus_rx_data[0], 2);
    //pr_tp("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);
    dbbus_tx_data[3] = (dbbus_rx_data[0] | 0x20);  //Set Bit 5
    i2c_write_msg2133(dbbus_tx_data, 4);
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x25;
    i2c_write_msg2133(dbbus_tx_data, 3);
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    i2c_read_msg2133(&dbbus_rx_data[0], 2);
    //pr_tp("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);
    dbbus_tx_data[3] = dbbus_rx_data[0] & 0xFC;  //Clear Bit 1,0
    i2c_write_msg2133(dbbus_tx_data, 4);
    /*
    //------------
    // ISP Speed Change to 400K
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    i2c_write_msg2133( dbbus_tx_data, 3);
    i2c_read_msg2133( &dbbus_rx_data[3], 1);
    //pr_tp("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);
    dbbus_tx_data[3] = dbbus_tx_data[3]&0xf7;  //Clear Bit3
    i2c_write_msg2133( dbbus_tx_data, 4);
    */
    //WP overwrite
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x0E;
    dbbus_tx_data[3] = 0x02;
    i2c_write_msg2133(dbbus_tx_data, 4);
    //set pin high
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x10;
    dbbus_tx_data[3] = 0x08;
    i2c_write_msg2133(dbbus_tx_data, 4);
    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();
    ///////////////////////////////////////
    // Start to load firmware
    ///////////////////////////////////////
    drvISP_EntryIspMode();
    pr_ch("entryisp\n");
    drvISP_BlockErase(0x00000);
    //msleep(1000);
    pr_tp("FwVersion=2");

    for(i = 0; i < 94; i++)    // total  94 KB : 1 byte per R/W
    {
        //msleep(1);//delay_100us
        pr_ch("drvISP_Program\n");
        drvISP_Program(i, temp[i]);    // program to slave's flash
        //pr_ch("drvISP_Verify\n");
        //drvISP_Verify ( i, temp[i] ); //verify data
    }

    //pr_tp("update OK\n");
    drvISP_ExitIspMode();
    FwDataCnt = 0;
    pixcir_reset();	
    msleep(500);		//delay_500ms	
    pr_tp("update OK\n");
    update_switch = 0;
    //打开中断
    enable_irq(global_irq);
    return size;
}

///////////////////////////////////////////////////////////////////////////
static DEVICE_ATTR(update, 0777, firmware_update_show, firmware_update_store);

/*test=================*/
static ssize_t firmware_clear_show(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{
    U16 k = 0, i = 0, j = 0;
    U8 bWriteData[5] =
    {
        0x10, 0x03, 0, 0, 0
    };
    U8 RX_data[256];
    U8 bWriteData1 = 0x12;
    U32 addr = 0;
    pr_ch("\n");
	printk("tyd-tp: firmware_clear_show\n");
    for(k = 0; k < 94; i++)    // total  94 KB : 1 byte per R/W
    {
        addr = k * 1024;

        for(j = 0; j < 8; j++)    //128*8 cycle
        {
            bWriteData[2] = (U8)((addr + j * 128) >> 16);
            bWriteData[3] = (U8)((addr + j * 128) >> 8);
            bWriteData[4] = (U8)(addr + j * 128);

            while((drvISP_ReadStatus() & 0x01) == 0x01)
            {
                ;    //wait until not in write operation
            }

            i2c_write_update_msg2133(bWriteData, 5);     //write read flash addr
            drvISP_Read(128, RX_data);
            i2c_write_update_msg2133(&bWriteData1, 1);    //cmd end

            for(i = 0; i < 128; i++)    //log out if verify error
            {
                if(RX_data[i] != 0xFF)
                {
                    pr_tp("k=%d,j=%d,i=%d===============erase not clean================", k, j, i);
                }
            }
        }
    }

    pr_tp("read finish\n");
    return sprintf(buf, "%s\n", fw_version);
}

static ssize_t firmware_clear_store(struct device *dev,
                                    struct device_attribute *attr, const char *buf, size_t size)
{
    U8 dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};
    //msctpc_LoopDelay ( 100 );        // delay about 100ms*****
    // Enable slave's ISP ECO mode
    /*
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();*/
    pr_ch("\n");
	printk("tyd-tp: firmware_clear_store\n");
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x08;
    dbbus_tx_data[2] = 0x0c;
    dbbus_tx_data[3] = 0x08;
    // Disable the Watchdog
    i2c_write_msg2133(dbbus_tx_data, 4);
    //Get_Chip_Version();
    //FwVersion  = 2;
    //if (FwVersion  == 2)
    {
        dbbus_tx_data[0] = 0x10;
        dbbus_tx_data[1] = 0x11;
        dbbus_tx_data[2] = 0xE2;
        dbbus_tx_data[3] = 0x00;
        i2c_write_msg2133(dbbus_tx_data, 4);
    }
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    i2c_write_msg2133(dbbus_tx_data, 4);
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    i2c_write_msg2133(dbbus_tx_data, 4);
    //Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    i2c_write_msg2133(dbbus_tx_data, 4);
    //Enable SPI Pad
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x02;
    i2c_write_msg2133(dbbus_tx_data, 3);
    i2c_read_msg2133(&dbbus_rx_data[0], 2);
    pr_tp("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);
    dbbus_tx_data[3] = (dbbus_rx_data[0] | 0x20);  //Set Bit 5
    i2c_write_msg2133(dbbus_tx_data, 4);
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x25;
    i2c_write_msg2133(dbbus_tx_data, 3);
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    i2c_read_msg2133(&dbbus_rx_data[0], 2);
    pr_tp("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);
    dbbus_tx_data[3] = dbbus_rx_data[0] & 0xFC;  //Clear Bit 1,0
    i2c_write_msg2133(dbbus_tx_data, 4);
    //WP overwrite
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x0E;
    dbbus_tx_data[3] = 0x02;
    i2c_write_msg2133(dbbus_tx_data, 4);
    //set pin high
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x10;
    dbbus_tx_data[3] = 0x08;
    i2c_write_msg2133(dbbus_tx_data, 4);
    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();
    ///////////////////////////////////////
    // Start to load firmware
    ///////////////////////////////////////
    drvISP_EntryIspMode();
    pr_tp("chip erase+\n");
    drvISP_BlockErase(0x00000);
    pr_tp("chip erase-\n");
    drvISP_ExitIspMode();
    return size;
}

static DEVICE_ATTR(clear, 0777, firmware_clear_show, firmware_clear_store);

/*test=================*/
/*Add by Tracy.Lin for update touch panel firmware and get fw version*/

static ssize_t firmware_version_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
	printk("tyd-tp: firmware_version_show\n");
    pr_ch("*** firmware_version_show fw_version = %s***\n", fw_version);
    return sprintf(buf, "%s\n", fw_version);
}

static ssize_t firmware_version_store(struct device *dev,
                                      struct device_attribute *attr, const char *buf, size_t size)
{
    unsigned char dbbus_tx_data[3];
    unsigned char dbbus_rx_data[4] ;
    unsigned short major = 0, minor = 0;
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    fw_version = kzalloc(sizeof(char), GFP_KERNEL);
    pr_ch("\n");
	printk("tyd-tp: firmware_version_store\n");
    //Get_Chip_Version();
    dbbus_tx_data[0] = 0x53;
    dbbus_tx_data[1] = 0x00;
    dbbus_tx_data[2] = 0x74;
    //i2c_write(TOUCH_ADDR_MSG20XX, &dbbus_tx_data[0], 3);
    //i2c_read(TOUCH_ADDR_MSG20XX, &dbbus_rx_data[0], 4);
    msg2133_i2c_write(&dbbus_tx_data[0], 3);
    msg2133_i2c_read(&dbbus_rx_data[0], 4);
    major = (dbbus_rx_data[1] << 8) + dbbus_rx_data[0];
    minor = (dbbus_rx_data[3] << 8) + dbbus_rx_data[2];
    pr_tp("***major = %d ***\n", major);
    pr_tp("***minor = %d ***\n", minor);
    sprintf(fw_version, "%03d%03d", major, minor);
    pr_tp("***fw_version = %s ***\n", fw_version);
    return size;
}
static DEVICE_ATTR(version, 0777, firmware_version_show, firmware_version_store);

static ssize_t firmware_data_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
	printk("tyd-tp: firmware_data_show\n");
    return FwDataCnt;
}

static ssize_t firmware_data_store(struct device *dev,
                                   struct device_attribute *attr, const char *buf, size_t size)
{
    int i;
    pr_ch("***FwDataCnt = %d ***\n", FwDataCnt);
	printk("tyd-tp: firmware_data_store\n");
    for(i = 0; i < 1024; i++)
    {
        memcpy(temp[FwDataCnt], buf, 1024);
    }

    FwDataCnt++;
    return size;
}
static DEVICE_ATTR(data, 0777, firmware_data_show, firmware_data_store);
//////////////////////////////////////////////////////////////////////////
#endif	//TP_FIRMWARE_UPDATE




static DEVICE_ATTR(calibrate, S_IRUGO | S_IWUSR, NULL, pixcir_set_calibrate);
static DEVICE_ATTR(suspend, S_IRUGO | S_IWUSR, pixcir_show_suspend, pixcir_store_suspend);

static ssize_t pixcir_set_calibrate(struct device* cd, struct device_attribute *attr,
		       const char* buf, size_t len)
{
	unsigned long on_off = simple_strtoul(buf, NULL, 10);
	
	if(on_off==1)
	{
		printk("%s: PIXCIR calibrate\n",__func__);
		pixcir_i2c_write_data(0x3a , 0x03);
		msleep(5*1000);
	}
	
	return len;
}



static ssize_t pixcir_show_suspend(struct device* cd,
				     struct device_attribute *attr, char* buf)
{
	ssize_t ret = 0;

	if(suspend_flag==1)
		sprintf(buf, "Pixcir Suspend\n");
	else
		sprintf(buf, "Pixcir Resume\n");
	
	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t pixcir_store_suspend(struct device* cd, struct device_attribute *attr,
		       const char* buf, size_t len)
{
	unsigned long on_off = simple_strtoul(buf, NULL, 10);
	suspend_flag = on_off;
	
	if(on_off==1)
	{
		printk("Pixcir Entry Suspend\n");
		pixcir_ts_suspend(NULL);
	}
	else
	{
		printk("Pixcir Entry Resume\n");
		pixcir_ts_resume(NULL);

	}
	
	return len;
}


static int pixcir_create_sysfs(struct i2c_client *client)
{
	int err;
	struct device *dev = &(client->dev);

	PIXCIR_DBG("%s\n", __func__);
	
	err = device_create_file(dev, &dev_attr_calibrate);
	err = device_create_file(dev, &dev_attr_suspend);

	return err;
}

static void pixcir_ts_suspend(struct early_suspend *handler)
{
       	printk("==%s==\n", __func__);

#if NEW_FUNC
	if (tpd_proximity_flag == 1)
	{
		return;
	}
#endif
	disable_irq_nosync(global_irq);
       	msleep(3);
	gpio_set_value(GPIO_TOUCH_RESET, 0);
	msleep(10);
}

#ifdef PIXCIR_CQ_CALL
static int touch_event_handler(void *unused)
{
    struct sched_param param = { .sched_priority = RTPM_PRIO_TPD }; 
    unsigned char buf;
    unsigned char tempdata[3];
    int i, error;
    
    sched_setscheduler(current, SCHED_RR, &param); 

    do
    {
	PIXCIR_DBG("%s: chz TASK_INTERRUPTIBLE \n",__func__);
        set_current_state(TASK_INTERRUPTIBLE);
	wait_event_interruptible(waiter, tpd_flag != 0);
        tpd_flag = 0;
        set_current_state(TASK_RUNNING); 
	PIXCIR_DBG("%s: chz TASK_RUNNING \n",__func__);
        while (!tpd_suspend_flag )
        {
		for(i=0;i<3;i++)
		{
			buf =0x1C;
			error=pixcir_i2c_rxdata(&buf, 1);
			tempdata[i]=buf;
			
			PIXCIR_DBG("%s: buf[%d]=0x%x\n",__func__,i, buf);
			msleep(10);
		}
		PIXCIR_DBG("%s: chz TASK_RUNNING \n",__func__);
		if((tempdata[0]==tempdata[1])&&(tempdata[1]==tempdata[2]))
		{
	       	    pixcir_reset();
		}  
                PIXCIR_DBG("%s: chz msleep(5000) \n",__func__);
		msleep(5000);
        }
		
    } while ( !kthread_should_stop() ); 
}
#endif

static void pixcir_ts_resume(struct early_suspend *handler)
{	
        unsigned char rdbuf[27];
       int num = 10;
       //disable_irq_nosync(global_irq);
	printk("==%s==start==\n", __func__);
#if NEW_FUNC
	if (tpd_proximity_flag == 1)
	{
		return;
	}
#endif
	pixcir_ts_pwron();
	enable_irq(global_irq);
	pixcir_reset();
#ifdef PIXCIR_CQ_CALL
        tpd_suspend_flag=0;
        tpd_flag=1;
        wake_up_interruptible(&waiter);
#endif
	/*init_timer(&tp_timer);
	tp_timer.function = &tp_timer_handle;
	tp_timer.expires = jiffies +5*HZ;
	add_timer(&tp_timer);*/
	printk("==%s==end==\n", __func__);

}


#ifdef TOUCH_VIRTUAL_KEYS
#if 1
#define SC8810_KEY_HOME	102
#define SC8810_KEY_MENU	30
#define SC8810_KEY_BACK	17
#define SC8810_KEY_SEARCH  217
#else
#define SC8810_KEY_MENU	17
#define SC8810_KEY_HOME	30
#define SC8810_KEY_BACK	102
#define SC8810_KEY_SEARCH  217
#endif

static char keymap[]={SC8810_KEY_MENU,SC8810_KEY_HOME,SC8810_KEY_BACK,SC8810_KEY_SEARCH};
   
static ssize_t virtual_keys_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,
		__stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE) ":159:529:40:40"
	   ":" __stringify(EV_KEY) ":" __stringify(KEY_MENU) ":59:529:40:40"
	   ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":259:529:40:40"
	   "\n");
}

static struct kobj_attribute virtual_keys_attr = {
    .attr = {
        .name = "virtualkeys.ms-msg21xx",
        .mode = S_IRUGO,
    },
    .show = &virtual_keys_show,
};

static struct attribute *properties_attrs[] = {
    &virtual_keys_attr.attr,
    NULL
};

static struct attribute_group properties_attr_group = {
    .attrs = properties_attrs,
};

static void pixcir_ts_virtual_keys_init(void)
{
    int ret;
    struct kobject *properties_kobj;
	
    PIXCIR_DBG("%s\n",__func__);
	
    properties_kobj = kobject_create_and_add("board_properties", NULL);
    if (properties_kobj)
        ret = sysfs_create_group(properties_kobj,
                     &properties_attr_group);
    if (!properties_kobj || ret)
        pr_err("failed to create board_properties\n");    
}


#endif

static void pixcir_ts_pwron(void)
{
	int err = 0;
	printk(KERN_INFO "==%s==\n", __func__);
	
	err =regulator_set_voltage(regu,2800000,2800000);
	if (err)
		pr_err("zinitix:could not set to 2800mv.\n");
	err =regulator_set_voltage(regu1,1800000,1800000);
	if (err)
		pr_err("zinitix:could not set to 1800mv.\n");

	regulator_enable(regu);
	regulator_enable(regu1);

	msleep(20);
}


static void pixcir_ts_pwroff(void)
{
	regulator_disable(regu);
	regulator_disable(regu1);

}

static int  pixcir_ts_config_pins(void)
{
	pixcir_ts_pwron();
	gpio_direction_input(GPIO_TOUCH_IRQ);	
	pixcir_irq = gpio_to_irq(GPIO_TOUCH_IRQ);
	pixcir_reset();

	return pixcir_irq;
}


static int attb_read_val(void)
{
	return gpio_get_value(GPIO_TOUCH_IRQ);
}

static void pixcir_reset(void)
{
	PIXCIR_DBG("%s\n",__func__);
	gpio_direction_output(GPIO_TOUCH_RESET, 1);
	msleep(3);
	gpio_set_value(GPIO_TOUCH_RESET, 0);
	msleep(10);
	gpio_set_value(GPIO_TOUCH_RESET,1);
	msleep(100);
}


static int pixcir_config_intmode(void)
{
	int error;
	unsigned char buf;

	msleep(60);
	error=pixcir_i2c_write_data(52, 0x0A);
	buf =  0x34;
	error=pixcir_i2c_rxdata(&buf, 1);
	PIXCIR_DBG("%s: buf=0x%x\n",__func__, buf);  
	return error;
}

static int  pixcir_init(void)
{
	int irq;
	PIXCIR_DBG("%s\n",__func__);
	irq = pixcir_ts_config_pins();
	//pixcir_config_intmode();
	return irq;
}


unsigned char tpd_check_sum(unsigned char *pval)
{
    int i, sum = 0;

    for(i = 0; i < 7; i++)
    {
        sum += pval[i];
    }

    return (unsigned char)((-sum) & 0xFF);
}


static void return_i2c_dev(struct i2c_dev *i2c_dev)
{
	spin_lock(&i2c_dev_list_lock);
	list_del(&i2c_dev->list);
	spin_unlock(&i2c_dev_list_lock);
	kfree(i2c_dev);
}

static struct i2c_dev *i2c_dev_get_by_minor(unsigned index)
{
	struct i2c_dev *i2c_dev;
	i2c_dev = NULL;

	spin_lock(&i2c_dev_list_lock);
	list_for_each_entry(i2c_dev, &i2c_dev_list, list)
	{
		if (i2c_dev->adap->nr == index)
			goto found;
	}
	i2c_dev = NULL;
	found: spin_unlock(&i2c_dev_list_lock);
	return i2c_dev;
}

static struct i2c_dev *get_free_i2c_dev(struct i2c_adapter *adap)
{
	struct i2c_dev *i2c_dev;

	if (adap->nr >= I2C_MINORS) {
		printk(KERN_ERR "i2c-dev: Out of device minors (%d)\n",
				adap->nr);
		return ERR_PTR(-ENODEV);
	}

	i2c_dev = kzalloc(sizeof(*i2c_dev), GFP_KERNEL);
	if (!i2c_dev)
		return ERR_PTR(-ENOMEM);

	i2c_dev->adap = adap;

	spin_lock(&i2c_dev_list_lock);
	list_add_tail(&i2c_dev->list, &i2c_dev_list);
	spin_unlock(&i2c_dev_list_lock);
	return i2c_dev;
}
/*********************************Bee-0928-bottom**************************************/

struct pixcir_i2c_ts_data {
	struct i2c_client *client;
	struct input_dev *input;
	//const struct pixcir_ts_platform_data *chip;
	bool exiting;
};


static struct point_node_t point_slot[MAX_FINGER_NUM*2];
static int keycode;
static int sy_rxdata(struct touch_info *cinfo, struct touch_info *pinfo)
 	{

	u32 retval;
	//static unsigned char tt_mode;
	//pinfo->count = cinfo->count;
	unsigned char reg_val[8] = {0};
	unsigned int dst_x=0,dst_y=0,xysawp_temp=0;
	unsigned int temp_checksum;
	struct TouchScreenInfo_t touchData;
	unsigned char touchkeycode = 0;
	static int preKeyStatus;
	
	//TPD_DEBUG("pinfo->count =%d\n",pinfo->count);
      cinfo->count  = 0; //touch end
      msg2133_i2c_read(reg_val, 8);
	

	cinfo->x1 =  ((reg_val[1] & 0xF0) << 4) | reg_val[2];	
	cinfo->y1  =  ((reg_val[1] & 0x0F) << 8) | reg_val[3];
       dst_x = ((reg_val[4] & 0xF0) << 4) | reg_val[5];
       dst_y = ((reg_val[4] & 0x0F) << 8) | reg_val[6];

       //PIXCIR_DBG("%s reg_val[8] = %d %d %d %d %d %d %d %d", __func__, reg_val[0],
	//	reg_val[1],reg_val[2],reg_val[3],reg_val[4],reg_val[5],reg_val[6],reg_val[7]);

       //PIXCIR_DBG(" x,y = %d %d, dst_x, y= %d %d ", cinfo->x1, cinfo->y1, dst_x, dst_y);

	temp_checksum = tpd_check_sum(reg_val);

	if((temp_checksum==reg_val[7])&&(reg_val[0] == 0x52))
	{

        if ((reg_val[1] == 0xFF) && (reg_val[2] == 0xFF) && (reg_val[3] == 0xFF) && (reg_val[4] == 0xFF) && (reg_val[6] == 0xFF))
        {
            cinfo->x1 = 0; // final X coordinate
            cinfo->y1 = 0; // final Y coordinate

           if((reg_val[5]==0x0)||(reg_val[5]==0xFF))
            {
                cinfo->count  = 0; //touch end
                touchData.nTouchKeyCode = 0; //TouchKeyMode
                touchData.nTouchKeyMode = 0; //TouchKeyMode
                 keycode=0;
            }
            else
            {
                touchData.nTouchKeyMode = 1; //TouchKeyMode
                touchData.nTouchKeyCode = reg_val[5]; //TouchKeyCode
                keycode= reg_val[5];
				if(tpd_proximity_flag)
				{	if (keycode == 0x80)
					{
						tpd_proximity_detect = 0;
						PIXCIR_DBG("#####enter 80#####\n");
						return 1;
					} else if (keycode == 0x40)
					{
						tpd_proximity_detect = 1;
						PIXCIR_DBG("#####enter 40#####\n");
						return 1;
					}
				} else 
				{
					if (keycode == 0x80 || keycode == 0x40)
					{
						PIXCIR_DBG("stefan kecode is %x\n", keycode);
						return 1;
					}
				}
		 if(keycode==1)
		 {
		 	cinfo->x1 = 59; // final X coordinate
            		cinfo->y1 = 529; // final Y coordinate

		 }
		 else if(keycode==2)
		 {
		 	cinfo->x1 = 159; // final X coordinate
            		cinfo->y1 = 529; // final Y coordinate

		 }
	 	else if(keycode==4)
		 {
		 	cinfo->x1 = 259; // final X coordinate
            		cinfo->y1 = 529; // final Y coordinate


		 }
		 else 
		 {
		 	cinfo->x1 = 359; // final X coordinate
            		cinfo->y1 = 529; // final Y coordinate

		 }

                cinfo->count  = 1;
            }
        }
	else
	{
		touchData.nTouchKeyMode = 0; //Touch on screen...

			if ((dst_x == 0) && (dst_y == 0))
			{
				cinfo->count  = 1; //one touch
								
				cinfo->x1 = (cinfo->x1 * MS_TS_MSG21XX_X_MAX) / 2048;
				cinfo->y1 = (cinfo->y1 * MS_TS_MSG21XX_Y_MAX) / 2048;
				
			}
			else
				{

				cinfo->count  = 2; //two touch


				if (dst_x > 2048)     //transform the unsigh value to sign value
				{
					dst_x -= 4096;
				}
				if (dst_y > 2048)
				{
					dst_y -= 4096;
				}

				cinfo->x2 = (cinfo->x1 + dst_x);
				cinfo->y2 = (cinfo->y1 + dst_y);	

				cinfo->x1 = (cinfo->x1 * MS_TS_MSG21XX_X_MAX) / 2048;
				
				cinfo->y1 = (cinfo->y1 * MS_TS_MSG21XX_Y_MAX) / 2048;
				

				cinfo->x2 = (cinfo->x2 * MS_TS_MSG21XX_X_MAX) / 2048;
				
				cinfo->y2 = (cinfo->y2 * MS_TS_MSG21XX_Y_MAX) / 2048;
				
				do
				{
					if((cinfo->y1 >= MS_TS_MSG21XX_Y_MAX)||(cinfo->y2 >= MS_TS_MSG21XX_Y_MAX))
					{
						if((cinfo->y2 >= MS_TS_MSG21XX_Y_MAX)&&(cinfo->y1 <= MS_TS_MSG21XX_Y_MAX))
						{
							if(cinfo->x2 <= 100)
							{
								cinfo->x1 = 59; // final X coordinate
		 				   		cinfo->y1 = 529; // final Y coordinate
								break;
							}
							else if((cinfo->x2 > 100 )&&(cinfo->x2 < 220))
							{
							 	cinfo->x1 = 159; // final X coordinate
						    		cinfo->y1 = 529; // final Y coordinate
								break;
							}
							else if((cinfo->x2 >= 220 )&&(cinfo->x2 < MS_TS_MSG21XX_X_MAX))
							{
							 	cinfo->x1 = 259; // final X coordinate
						    		cinfo->y1 = 529; // final Y coordinate
								break;
							}
						}
						else if((cinfo->y2 >= MS_TS_MSG21XX_Y_MAX)&&(cinfo->y1 >= MS_TS_MSG21XX_Y_MAX))
						{
							break;
						}
						else 
						{
							if(cinfo->x1 <= 100)
							{
								cinfo->x1 = 59; // final X coordinate
		 				   		cinfo->y1 = 529; // final Y coordinate
								break;
							}
							else if((cinfo->x1 > 100 )&&(cinfo->x1 < 220))
							{
							 	cinfo->x1 = 159; // final X coordinate
						    		cinfo->y1 = 529; // final Y coordinate
								break;
							}
							else if((cinfo->x1 >= 220 )&&(cinfo->x1 < MS_TS_MSG21XX_X_MAX))
							{
							 	cinfo->x1 = 259; // final X coordinate
						    		cinfo->y1 = 529; // final Y coordinate
								break;
							}
						}
					}
				}while(0);

			}
		}

        return 1;
    }
	 
	 return 1;
 }
static void pixcir_ts_poscheck(struct pixcir_i2c_ts_data *data)
{
	struct pixcir_i2c_ts_data *tsdata = data;
	struct touch_info cinfo, pinfo;

	int *p;
	unsigned char touch, button, pix_id,slot_id;
	unsigned char rdbuf[27];
	int ret, i;
	static int lastkey=0;
        PIXCIR_DBG("===%s===\n",__func__);
	rdbuf[0]=0;
	//pixcir_i2c_rxdata(rdbuf, 27);
        //touch = rdbuf[0]&0x07;
        sy_rxdata(&cinfo, &pinfo);
        touch = cinfo.count;
	//button = rdbuf[1];
	//p=&rdbuf[2];
        p=(int*)&cinfo;
        PIXCIR_DBG("===touch:%d===",touch);
	for (i=0; i<touch; i++)	{
		//pix_id = (*(p+4));
		//slot_id = ((pix_id & 7)<<1) | ((pix_id & 8)>>3);
		point_slot[i].active = 1;
		point_slot[i].finger_id = i;	
		point_slot[i].posx = *(p+i*2);
		point_slot[i].posy = *(p+i*2+1);
		               
	}

	if(touch) {
		input_report_key(tsdata->input, BTN_TOUCH, 1);
		//input_report_abs(tsdata->input, ABS_MT_TOUCH_MAJOR, 15);
		for (i=0; i<touch; i++) {
			if (point_slot[i].active == 1) {
				if(point_slot[i].posy<0) {					
				} else {
					if(point_slot[i].posx<0) {						
						point_slot[i].posx = 0;
					} else if (point_slot[i].posx>318) {					
						point_slot[i].posx = 318;
					}
					input_report_abs(tsdata->input, ABS_MT_POSITION_X,  point_slot[i].posx);
					input_report_abs(tsdata->input, ABS_MT_POSITION_Y,  point_slot[i].posy);
					input_mt_sync(tsdata->input);
					PIXCIR_DBG("%s: slot=%d,x%d=%d,y%d=%d\n",__func__, i, i/2,point_slot[i].posx, i/2, point_slot[i].posy);
				}
			}
		}
	} else {
		PIXCIR_DBG("%s: release\n",__func__);
		input_report_key(tsdata->input, BTN_TOUCH, 0);
		input_report_abs(tsdata->input, ABS_MT_TOUCH_MAJOR, 0);
	}
	PIXCIR_DBG("%s: keycode =%x\n",__func__,keycode);
	/*
	if(lastkey!=keycode)
	{
		for(i=0;i<3;i++)
		{
			input_report_key(tsdata->input, keymap[i], (keycode)&(1<<i));
		}
		lastkey=keycode;
	}*/
	input_sync(tsdata->input); 
	for (i=0; i<MAX_FINGER_NUM*2; i++) {
		if (point_slot[i].active == 0) {
			point_slot[i].posx = 0;
			point_slot[i].posy = 0;
		}
		point_slot[i].active = 0;
	}

}

static irqreturn_t pixcir_ts_isr(int irq, void *dev_id)
{
	struct pixcir_i2c_ts_data *tsdata = dev_id;
	
	disable_irq_nosync(irq);

 	while (!tsdata->exiting) {
		pixcir_ts_poscheck(tsdata);

#if 1
		if (attb_read_val()) {
			PIXCIR_DBG("%s: release\n",__func__);
			input_report_key(tsdata->input, BTN_TOUCH, 0);
			input_report_abs(tsdata->input, ABS_MT_TOUCH_MAJOR, 0);
			input_sync(tsdata->input);
			break;
		}
#endif
		//msleep(20);
		break;
	}

	enable_irq(irq);
	
	return IRQ_HANDLED;
}

#ifdef CONFIG_PM_SLEEP
static int pixcir_i2c_ts_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned char wrbuf[2] = { 0 };
	int ret;

	wrbuf[0] = 0x33;
	wrbuf[1] = 0x03;	//enter into freeze mode;
	/**************************************************************
	wrbuf[1]:	0x00: Active mode
			0x01: Sleep mode
			0xA4: Sleep mode automatically switch
			0x03: Freeze mode
	More details see application note 710 power manangement section
	****************************************************************/
	ret = i2c_master_send(client, wrbuf, 2);
	if(ret!=2) {
		dev_err(&client->dev,
			"%s: i2c_master_send failed(), ret=%d\n",
			__func__, ret);
	}

	if (device_may_wakeup(&client->dev))
		enable_irq_wake(client->irq);

	return 0;
}

static int pixcir_i2c_ts_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
///if suspend enter into freeze mode please reset TP
#if 1
	pixcir_reset();
#else
	unsigned char wrbuf[2] = { 0 };
	int ret;

	wrbuf[0] = 0x33;
	wrbuf[1] = 0;
	ret = i2c_master_send(client, wrbuf, 2);
	if(ret!=2) {
		dev_err(&client->dev,
			"%s: i2c_master_send failed(), ret=%d\n",
			__func__, ret);
	}
#endif
	if (device_may_wakeup(&client->dev))
		disable_irq_wake(client->irq);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(pixcir_dev_pm_ops,
			 pixcir_i2c_ts_suspend, pixcir_i2c_ts_resume);

static int tpd_get_bl_info (int show)
 {
   int retval = TPD_OK;
 
	retval = i2c_smbus_read_i2c_block_data(this_client, TPD_REG_BASE, 8, (unsigned char  *)&g_bootloader_data);
	retval += i2c_smbus_read_i2c_block_data(this_client, TPD_REG_BASE + 8, 8, (((unsigned char  *)&g_bootloader_data) + 8));
        TPD_DEBUG("ret=%d,first=0x%02x\n",retval,(unsigned char )(*((unsigned char  *)&g_bootloader_data)));
	if (show)
	 {
 
	   TPD_DEBUG("BL: bl_file = %02X, bl_status = %02X, bl_error = %02X, blver = %02X%02X, bld_blver = %02X%02X\n", \
				 g_bootloader_data.bl_file, \
				 g_bootloader_data.bl_status, \
				 g_bootloader_data.bl_error, \
				 g_bootloader_data.blver_hi, g_bootloader_data.blver_lo, \
				 g_bootloader_data.bld_blver_hi, g_bootloader_data.bld_blver_lo);
	
		 TPD_DEBUG("BL: ttspver = 0x%02X%02X, appid=0x%02X%02X, appver=0x%02X%02X\n", \
				 g_bootloader_data.ttspver_hi, g_bootloader_data.ttspver_lo, \
				 g_bootloader_data.appid_hi, g_bootloader_data.appid_lo, \
				 g_bootloader_data.appver_hi, g_bootloader_data.appver_lo);
	 
		 TPD_DEBUG("BL: cid = 0x%02X%02X%02X\n", \
				 g_bootloader_data.cid_0, \
				 g_bootloader_data.cid_1, \
				 g_bootloader_data.cid_2);
	 
	 }
        /*ergate-001*/
	//mdelay(100);
	
	return retval;
 }
 

static void sy_init()
{
        int retval = TPD_OK;
        int tries = 0;
        unsigned char  host_reg;
	host_reg = TPD_SOFT_RESET_MODE;
        PIXCIR_DBG("==sy_init==0==");
        retval = i2c_smbus_write_i2c_block_data(this_client,TPD_REG_BASE,sizeof(host_reg),&host_reg);
        printk("==sy_init==1==retval:%d==",retval);
        if(retval < TPD_OK)
	return retval;
 
         do{
	 mdelay(100);
	 retval = tpd_get_bl_info(1);
	 }while(!(retval < TPD_OK) && !GET_BOOTLOADERMODE(g_bootloader_data.bl_status)&& 
	 !(g_bootloader_data.bl_file == TPD_OP_MODE + TPD_LOW_PWR_MODE) && tries++ < 10);
         if(g_bootloader_data.bl_status != (unsigned char )0x11)
         {
	if(!(retval < 0))
		 {
		 host_reg = TPD_OP_MODE;
		 retval = i2c_smbus_write_i2c_block_data(this_client,TPD_REG_BASE,sizeof(host_reg),&host_reg);
		 mdelay(100);
		 }
	 if(!(retval < 0))
		 {
		 TPD_DEBUG("Switch to sysinfo mode \n");
		 host_reg = TPD_SYSINFO_MODE;
		 retval = i2c_smbus_write_i2c_block_data(this_client,TPD_REG_BASE,sizeof(host_reg),&host_reg);
		 mdelay(100);
 
		if(!(retval < TPD_OK))
		 {
		 retval = i2c_smbus_read_i2c_block_data(this_client,TPD_REG_BASE, 8, (unsigned char  *)&g_sysinfo_data);
		 retval += i2c_smbus_read_i2c_block_data(this_client,TPD_REG_BASE + 8, 8, (((unsigned char  *)(&g_sysinfo_data)) + 8));
		 retval += i2c_smbus_read_i2c_block_data(this_client,TPD_REG_BASE + 16, 8, (((unsigned char  *)(&g_sysinfo_data)) + 16));
		 retval += i2c_smbus_read_i2c_block_data (this_client,TPD_REG_BASE + 24, 8, (((unsigned char  *)(&g_sysinfo_data)) + 24));
		 
		 TPD_DEBUG("SI: hst_mode = 0x%02X, mfg_cmd = 0x%02X, mfg_stat = 0x%02X\n", \
					 g_sysinfo_data.hst_mode, \
					 g_sysinfo_data.mfg_cmd, \
					 g_sysinfo_data.mfg_stat);
			 TPD_DEBUG("SI: bl_ver = 0x%02X%02X\n", \
					 g_sysinfo_data.bl_verh, \
					 g_sysinfo_data.bl_verl);
			 TPD_DEBUG("SI: act_int = 0x%02X, tch_tmout = 0x%02X, lp_int = 0x%02X\n", \
					 g_sysinfo_data.act_intrvl, \
					 g_sysinfo_data.tch_tmout, \
					 g_sysinfo_data.lp_intrvl);
			 TPD_DEBUG("SI: tver = %02X%02X, a_id = %02X%02X, aver = %02X%02X\n", \
					 g_sysinfo_data.tts_verh, \
					 g_sysinfo_data.tts_verl, \
					 g_sysinfo_data.app_idh, \
					 g_sysinfo_data.app_idl, \
					 g_sysinfo_data.app_verh, \
					 g_sysinfo_data.app_verl);
			 TPD_DEBUG("SI: c_id = %02X%02X%02X\n", \
					 g_sysinfo_data.cid[0], \
					 g_sysinfo_data.cid[1], \
					 g_sysinfo_data.cid[2]);
		 }
 
		 TPD_DEBUG("Switch back to operational mode \n");
		  if (!(retval < TPD_OK)) 
			 {
			 host_reg = TPD_OP_MODE;
			 retval = i2c_smbus_write_i2c_block_data(this_client, TPD_REG_BASE, sizeof(host_reg), &host_reg);
			 mdelay(100);
			 }
		}
	}
	else
	{
	//update_support = 1;
         TPD_DEBUG("Switch back to operational mode \n");

		  if (!(retval < TPD_OK)) 
			 {
			 unsigned char  op_cmds[] = {0x00, 0x00, 0xFF, 0xA5, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
			 //retval = i2c_master_send_ext(i2c_client, op_cmds, sizeof(op_cmds));
                         retval = i2c_master_send(this_client,op_cmds, sizeof(op_cmds));
			 mdelay(100);
			 }
	}
        if(retval > TPD_OK)
	retval = TPD_OK;

	 return retval;
 
}

#if NEW_FUNC
static void tpd_ps_handler(void)
{
	// rohm弄继恕	

	PIXCIR_DBG("tenggaoyun tpd_proximity_detect=%d\n",tpd_proximity_detect);
//	tpd_proximity_detect = (tpd_proximity_detect==1) ? 0: 1;

	/* 0 is close, 1 is far */
	if(pre_value!=tpd_proximity_detect)
	{
		PIXCIR_DBG("report data");
		PIXCIR_DBG("\n/****value is %d*****//*\n", tpd_proximity_detect);
		pre_value=tpd_proximity_detect;
		input_report_abs(input_dev, ABS_DISTANCE, tpd_proximity_detect);
		input_sync(input_dev);
	}
	else
	{
		PIXCIR_DBG("no report");
	}
	mod_timer(&ps_timer, jiffies + msecs_to_jiffies(500) );  
	//queue_work(pls->ltr_work_queue,&pls->work);*/
	/*	PIXCIR_DBG("tenggaoyun tpd_proximity_detect=%d\n",tpd_proximity_detect);
		tpd_proximity_detect = (tpd_proximity_detect==1) ? 0: 1;
		input_report_abs(input_dev, ABS_DISTANCE, tpd_proximity_detect);                
		input_sync(input_dev);                                                             
		mod_timer(&ps_timer, jiffies + msecs_to_jiffies(500) );                        */
}

static int tpd_enable_ps(int enable)
{
	unsigned char ps_store_data[4];
	int err = 0;

	if (enable)
	{
		PIXCIR_DBG("tenggaoyun ctp ps function open\n");
		ps_store_data[0] = 0x52;
		ps_store_data[1] = 0x00;  //01  
		ps_store_data[2] = 0x62;  //24
		ps_store_data[3] = 0xA0;  //A1
		tpd_proximity_flag = 1;
	}
	else
	{
		PIXCIR_DBG("tenggaoyun ctp ps function close\n");
		ps_store_data[0] = 0x52;
		ps_store_data[1] = 0x00;  //01
		ps_store_data[2] = 0x62;  //24
		ps_store_data[3] = 0xA1;  //A0
		tpd_proximity_flag = 0;
	}
	err= i2c_master_send(this_client, &ps_store_data[0], 4);
	//return 0;
}

static int al3006_pls_enable(SENSOR_TYPE type)
{
	int ret;
	printk("tenggaoyun al3006_pls_enable\n");
	tpd_enable_ps(1);
	//stefan debug
//	input_report_abs(input_dev, ABS_DISTANCE, 1);   // 1
//	input_sync(input_dev);
	mod_timer(&ps_timer, jiffies + msecs_to_jiffies(200) );
	return ret;
}

static int al3006_pls_disable(SENSOR_TYPE type)
{	
	int ret;
	printk("tenggaoyun al3006_pls_disable\n");
	tpd_enable_ps(0);
	del_timer_sync(&ps_timer);
	return ret;
}

static int al3006_pls_open(struct inode *inode, struct file *file)
{
	printk("tenggaoyun %s\n", __func__);
	if (tpd_als_opened)
		return -EBUSY;
	tpd_als_opened = 1;
	return 0;
}

static int al3006_pls_release(struct inode *inode, struct file *file)
{
	printk("tenggaoyun %s\n", __func__);
	tpd_als_opened = 0;
	return 0; 	
}

static long al3006_pls_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int flag;
	unsigned char data;
	printk("tenggaoyun al3006_pls_ioctl\n");
	int flag1;

	//get ioctl parameter
	switch (cmd) {
		case LTR_IOCTL_SET_PFLAG:
		case LTR_IOCTL_SET_LFLAG:
			if (copy_from_user(&flag, argp, sizeof(flag))) {
				return -EFAULT;
			}
			if (flag < 0 || flag > 1) {
				return -EINVAL;
			}
			printk("tenggaoyun %s: set flag=%d\n", __func__, flag);
			break;
		default:
			break;
	} 

	//handle ioctl
	switch (cmd) {
		case LTR_IOCTL_GET_PFLAG:
			printk("tenggaoyun LTR_IOCTL_GET_PFLAG\n");
			break;

		case LTR_IOCTL_GET_LFLAG:
			printk("tenggaoyun LTR_IOCTL_GET_LFLAG\n");
			break;

		case LTR_IOCTL_GET_DATA:
			printk("tenggaoyun LTR_IOCTL_GET_DATA\n");
			break;

		case LTR_IOCTL_SET_PFLAG:	
			printk("tenggaoyun LTR_IOCTL_SET_PFLAG flag=%d\n",flag);
			if(flag==1){
				al3006_pls_enable(AL3006_PLS_PXY);
			}
			else if(flag==0) {
				al3006_pls_disable(AL3006_PLS_PXY);
			}
			break;
		case LTR_IOCTL_SET_LFLAG1:	
		case LTR_IOCTL_SET_LFLAG:
			break;	
		default:
			return -EINVAL;
	}

	//report ioctl
	switch (cmd) {
		case LTR_IOCTL_GET_PFLAG:
		case LTR_IOCTL_GET_LFLAG:
			if (copy_to_user(argp, &flag, sizeof(flag))) {
				return -EFAULT;
			}
			break;

		case LTR_IOCTL_GET_DATA:
			//al3006_pls_read_data(AL3006_PLS_REG_DATA,&data);
			if (copy_to_user(argp, &data, sizeof(data))) {
				return -EFAULT;
			}
			break;

		default:
			break;
	}

	return 0;

}

static struct file_operations al3006_pls_fops = {
	.owner	= THIS_MODULE,
	.open	= al3006_pls_open,
	.release	= al3006_pls_release,
	.unlocked_ioctl		= al3006_pls_ioctl,
};
static struct miscdevice al3006_pls_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = AL3006_PLS_DEVICE,
	.fops = &al3006_pls_fops,
};

/********stefan**************/

static ssize_t rohm_proximity_show_psenable(struct device* cd, struct device_attribute *attr, char* buf)
{
	ssize_t ret = 0;

	if(tpd_proximity_flag==1)
		//sprintf(buf, "Proximity Start\n");
		sprintf(buf, "1\n");
	else
		//sprintf(buf, "Proximity Stop\n");
		sprintf(buf, "0\n");
	ret = strlen(buf) + 1;
	return ret;
}

static ssize_t rohm_proximity_store_psenable(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
{
	unsigned long on_off = simple_strtoul(buf, NULL, 10);
	tpd_proximity_flag = on_off;
	if(on_off==1)
	{
		printk("Proximity Start\n");
		al3006_pls_enable(AL3006_PLS_PXY);
	}
	else
	{
		printk("Proximity Stop\n");
		al3006_pls_disable(AL3006_PLS_PXY);
//	pixcir_ts_pwron();
//	enable_irq(global_irq);
	} 
	return len;
}

static ssize_t rohm_proximity_show_debug(struct device* cd,struct device_attribute *attr, char* buf)
{
	ssize_t ret = 0;

	sprintf(buf, "ROHM Debug %d\n",debug_level);

	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t rohm_proximity_store_debug(struct device* cd, struct device_attribute *attr,
		const char* buf, size_t len)
{
	unsigned long on_off = simple_strtoul(buf, NULL, 10);
	debug_level = on_off;

	printk("%s: debug_level=%d\n",__func__, debug_level);

	return len;
}

static ssize_t rohm_proximity_show_version(struct device* cd,struct device_attribute *attr, char* buf)
{
	ssize_t ret = 0;

	sprintf(buf, "ROHM");	
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(psenable,0666 , rohm_proximity_show_psenable, rohm_proximity_store_psenable);
static DEVICE_ATTR(debug, 0666, rohm_proximity_show_debug, rohm_proximity_store_debug);
static DEVICE_ATTR(version_new, 0666, rohm_proximity_show_version, NULL);

//added by gary
static struct attribute *rohm_attributes[] = {
	&dev_attr_psenable.attr,
	&dev_attr_debug,
	&dev_attr_version_new.attr,
	NULL
};

static struct attribute_group rohm_attribute_group = {
	.attrs = rohm_attributes
};
//end

/****************************/

#endif

static int __devinit pixcir_i2c_ts_probe(struct i2c_client *client,
					 const struct i2c_device_id *id)
{
	//const struct pixcir_ts_platform_data *pdata = client->dev.platform_data;
	struct pixcir_i2c_ts_data *tsdata;
	struct input_dev *input;
	struct device *dev;
	struct i2c_dev *i2c_dev;
	int i, error;
        u32 retval;
#if NEW_FUNC
	int err = 0;
	int rc = -EIO;
#endif
	regu = regulator_get(&client->dev, REGU_NAME_TP);
	regu1 = regulator_get(&client->dev, REGU_NAME_TP1);

	this_client = client;
	client->irq = pixcir_ts_config_pins(); //reset pin set to 0 or 1 and platform init
	
	for(i=0; i<MAX_FINGER_NUM*2; i++) {
		point_slot[i].active = 0;
	}
//        sy_init();

	/*init_timer(&tp_timer);
	tp_timer.function = &tp_timer_handle;
	tp_timer.expires = jiffies +500;
	add_timer(&tp_timer);*/

	#ifdef TP_FIRMWARE_UPDATE
	    firmware_class = class_create(THIS_MODULE,"ms-touchscreen-msg20xx" );//client->name

	    if(IS_ERR(firmware_class))
	    {
	        pr_err("Failed to create class(firmware)!\n");
	    }

	    firmware_cmd_dev = device_create(firmware_class,
	                                     NULL, 0, NULL, "device");//device

	    if(IS_ERR(firmware_cmd_dev))
	    {
	        pr_err("Failed to create device(firmware_cmd_dev)!\n");
	    }

	    // version
	    if(device_create_file(firmware_cmd_dev, &dev_attr_version) < 0)
	    {
	        pr_err("Failed to create device file(%s)!\n", dev_attr_version.attr.name);
	    }

	    // update /sys/class/mtk-tpd/device/update

	    if(device_create_file(firmware_cmd_dev, &dev_attr_update) < 0)
	    {
	        pr_err("Failed to create device file(%s)!\n", dev_attr_update.attr.name);
	    }

	    // data
	    if(device_create_file(firmware_cmd_dev, &dev_attr_data) < 0)
	    {
	        pr_err("Failed to create device file(%s)!\n", dev_attr_data.attr.name);
	    }

	    if(device_create_file(firmware_cmd_dev, &dev_attr_clear) < 0)
	    {
	        pr_err("Failed to create device file(%s)!\n", dev_attr_clear.attr.name);
	    }

		//dev_set_drvdata(firmware_cmd_dev, NULL);
	#endif
	
#if 1
	tsdata = kzalloc(sizeof(*tsdata), GFP_KERNEL);
	input = input_allocate_device();
	if (!tsdata || !input) {
		dev_err(&client->dev, "Failed to allocate driver data!\n");
		error = -ENOMEM;
		goto err_free_mem;
	}
#ifdef TOUCH_VIRTUAL_KEYS
	pixcir_ts_virtual_keys_init();
#endif

	tsdata->client = client;
	tsdata->input = input;
	//tsdata->chip = pdata;
	global_irq = client->irq;

	input->name = client->name;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &client->dev;

	__set_bit(EV_KEY, input->evbit);
	__set_bit(EV_ABS, input->evbit);
	__set_bit(EV_SYN, input->evbit);
	__set_bit(BTN_TOUCH, input->keybit);

	__set_bit(ABS_MT_TOUCH_MAJOR, input->absbit);
	__set_bit(ABS_MT_POSITION_X, input->absbit);
	__set_bit(ABS_MT_POSITION_Y, input->absbit);
//	__set_bit(ABS_MT_WIDTH_MAJOR, input->absbit);

	__set_bit(KEY_MENU,  input->keybit);
	__set_bit(KEY_BACK,  input->keybit);
	__set_bit(KEY_HOMEPAGE,  input->keybit);
	
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, MS_TS_MSG21XX_X_MAX, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, MS_TS_MSG21XX_Y_MAX, 0, 0);
//	input_set_abs_params(input, ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);


	input_set_drvdata(input, tsdata);

	error = request_threaded_irq(client->irq, NULL, pixcir_ts_isr,
				     IRQF_TRIGGER_FALLING,
				     client->name, tsdata);
	if (error) {
		dev_err(&client->dev, "Unable to request touchscreen IRQ.\n");
		goto err_free_mem;
	}
	disable_irq_nosync(client->irq);

	
	error = input_register_device(input);
	if (error)
		goto err_free_irq;

	i2c_set_clientdata(client, tsdata);
	device_init_wakeup(&client->dev, 1);

	/*********************************Bee-0928-TOP****************************************/
	i2c_dev = get_free_i2c_dev(client->adapter);
	if (IS_ERR(i2c_dev)) {
		error = PTR_ERR(i2c_dev);
		return error;
	}

	dev = device_create(i2c_dev_class, &client->adapter->dev, MKDEV(I2C_MAJOR,
			client->adapter->nr), NULL, "pixcir_i2c_ts%d", 0);
	if (IS_ERR(dev)) {
		error = PTR_ERR(dev);
		return error;
	}
	/*********************************Bee-0928-BOTTOM****************************************/

	pixcir_early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	pixcir_early_suspend.suspend = pixcir_ts_suspend;
	pixcir_early_suspend.resume  = pixcir_ts_resume;
	register_early_suspend(&pixcir_early_suspend);

	/*if(pixcir_config_intmode()<0) {
		printk("%s: I2C error\n",__func__);
		goto err_free_irq;
	}*/
	pixcir_create_sysfs(client);

#if NEW_FUNC
	//register device
	err = misc_register(&al3006_pls_device);
	if (err)
	{
		printk("%s: al3006_pls_device register failed\n", __func__);
	} 
	// register input device 
	input_dev = input_allocate_device();
	if (!input_dev) 
	{
		printk("%s: input allocate device failed\n", __func__);
	}	
	input_dev->name = AL3006_PLS_INPUT_DEV;
	input_dev->phys  = AL3006_PLS_INPUT_DEV;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0010;

	__set_bit(EV_ABS, input_dev->evbit);	
	//for proximity
	input_set_abs_params(input_dev, ABS_DISTANCE, 0, 1, 0, 0);
	//for lightsensor
//	input_set_abs_params(input_dev, ABS_MISC, 0, 100001, 0, 0);

	err = input_register_device(input_dev);
	if (err < 0)
	{
		printk("%s: input device regist failed\n", __func__);
	}
	setup_timer(&ps_timer, tpd_ps_handler, 0 );
	//wake_lock_init(&pls_delayed_work_wake_lock, WAKE_LOCK_SUSPEND, "prox_delayed_work");
	rc = sysfs_create_group(&input_dev->dev.kobj, &rohm_attribute_group);//added by gary
#endif
	
	dev_err(&tsdata->client->dev, "insmod successfully!\n");
	
	enable_irq(client->irq);
	return 0;

err_free_irq:
	free_irq(client->irq, tsdata);
	free_irq(pixcir_irq,NULL);
err_free_mem:
	input_free_device(input);
	kfree(tsdata);
	#endif
	
	return error;
}

static int __devexit pixcir_i2c_ts_remove(struct i2c_client *client)
{
	int error;
	struct i2c_dev *i2c_dev;
	struct pixcir_i2c_ts_data *tsdata = i2c_get_clientdata(client);

	device_init_wakeup(&client->dev, 0);

	tsdata->exiting = true;
	mb();
	free_irq(client->irq, tsdata);

	/*********************************Bee-0928-TOP****************************************/
	i2c_dev = get_free_i2c_dev(client->adapter);
	if (IS_ERR(i2c_dev)) {
		error = PTR_ERR(i2c_dev);
		return error;
	}

	return_i2c_dev(i2c_dev);
	device_destroy(i2c_dev_class, MKDEV(I2C_MAJOR, client->adapter->nr));
	/*********************************Bee-0928-BOTTOM****************************************/
	unregister_early_suspend(&pixcir_early_suspend);
	free_irq(pixcir_irq,NULL);
	input_unregister_device(tsdata->input);
	kfree(tsdata);

	return 0;
}

/*************************************Bee-0928****************************************/
/*                        	     pixcir_open                                     */
/*************************************Bee-0928****************************************/
static int pixcir_open(struct inode *inode, struct file *file)
{
	int subminor;
	struct i2c_client *client;
	struct i2c_adapter *adapter;
	struct i2c_dev *i2c_dev;
	int ret = 0;
	PIXCIR_DBG("enter pixcir_open function\n");

	subminor = iminor(inode);

//	lock_kernel();
	i2c_dev = i2c_dev_get_by_minor(subminor);
	if (!i2c_dev) {
		printk("error i2c_dev\n");
		return -ENODEV;
	}

	adapter = i2c_get_adapter(i2c_dev->adap->nr);
	if (!adapter) {
		return -ENODEV;
	}
	
	client = kzalloc(sizeof(*client), GFP_KERNEL);
	if (!client) {
		i2c_put_adapter(adapter);
		ret = -ENOMEM;
	}

	snprintf(client->name, I2C_NAME_SIZE, "pixcir_i2c_ts%d", adapter->nr);
	client->driver = &pixcir_i2c_ts_driver;
	client->adapter = adapter;
	
	file->private_data = client;

	return 0;
}

/*************************************Bee-0928****************************************/
/*                        	     pixcir_ioctl                                    */
/*************************************Bee-0928****************************************/
static long pixcir_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client *) file->private_data;

	PIXCIR_DBG("pixcir_ioctl(),cmd = %d,arg = %ld\n", cmd, arg);


	switch (cmd)
	{
	case CALIBRATION_FLAG:	//CALIBRATION_FLAG = 1
		client->addr = SLAVE_ADDR;
		status_reg = CALIBRATION_FLAG;
		break;

	case BOOTLOADER:	//BOOTLOADER = 7
		client->addr = BOOTLOADER_ADDR;
		status_reg = BOOTLOADER;

		pixcir_reset();
		mdelay(5);
		break;

	case RESET_TP:		//RESET_TP = 9
		pixcir_reset();
		break;
		
	case ENABLE_IRQ:	//ENABLE_IRQ = 10
		status_reg = 0;
		enable_irq(global_irq);
		break;
		
	case DISABLE_IRQ:	//DISABLE_IRQ = 11
		disable_irq_nosync(global_irq);
		break;

	case BOOTLOADER_STU:	//BOOTLOADER_STU = 12
		client->addr = BOOTLOADER_ADDR;
		status_reg = BOOTLOADER_STU;

		pixcir_reset();
		mdelay(5);

	case ATTB_VALUE:	//ATTB_VALUE = 13
		client->addr = SLAVE_ADDR;
		status_reg = ATTB_VALUE;
		break;

	default:
		client->addr = SLAVE_ADDR;
		status_reg = 0;
		break;
	}
	return 0;
}

/***********************************Bee-0928****************************************/
/*                        	  pixcir_read                                      */
/***********************************Bee-0928****************************************/
static ssize_t pixcir_read (struct file *file, char __user *buf, size_t count,loff_t *offset)
{
	struct i2c_client *client = (struct i2c_client *)file->private_data;
	unsigned char *tmp, bootloader_stu[4], attb_value[1];
	int ret = 0;

	switch(status_reg)
	{
	case BOOTLOADER_STU:
		i2c_master_recv(client, bootloader_stu, sizeof(bootloader_stu));
		if (ret!=sizeof(bootloader_stu)) {
			dev_err(&client->dev,
				"%s: BOOTLOADER_STU: i2c_master_recv() failed, ret=%d\n",
				__func__, ret);
			return -EFAULT;
		}

		ret = copy_to_user(buf, bootloader_stu, sizeof(bootloader_stu));
		if(ret)	{
			dev_err(&client->dev,
				"%s: BOOTLOADER_STU: copy_to_user() failed.\n",	__func__);
			return -EFAULT;
		}else {
			ret = 4;
		}
		break;

	case ATTB_VALUE:
		attb_value[0] = attb_read_val();
		if(copy_to_user(buf, attb_value, sizeof(attb_value))) {
			dev_err(&client->dev,
				"%s: ATTB_VALUE: copy_to_user() failed.\n", __func__);
			return -EFAULT;
		}else {
			ret = 1;
		}
		break;

	default:
		tmp = kmalloc(count,GFP_KERNEL);
		if (tmp==NULL)
			return -ENOMEM;

		ret = i2c_master_recv(client, tmp, count);
		if (ret != count) {
			dev_err(&client->dev,
				"%s: default: i2c_master_recv() failed, ret=%d\n",
				__func__, ret);
			return -EFAULT;
		}

		if(copy_to_user(buf, tmp, count)) {
			dev_err(&client->dev,
				"%s: default: copy_to_user() failed.\n", __func__);
			kfree(tmp);
			return -EFAULT;
		}
		kfree(tmp);
		break;
	}
	return ret;
}

/***********************************Bee-0928****************************************/
/*                        	  pixcir_write                                     */
/***********************************Bee-0928****************************************/
static ssize_t pixcir_write(struct file *file,const char __user *buf,size_t count, loff_t *ppos)
{
	struct i2c_client *client;
	unsigned char *tmp, bootload_data[143];
	int ret=0, i=0;

	client = file->private_data;

	switch(status_reg)
	{
	case CALIBRATION_FLAG:	//CALIBRATION_FLAG=1
		tmp = kmalloc(count,GFP_KERNEL);
		if (tmp==NULL)
			return -ENOMEM;

		if (copy_from_user(tmp,buf,count)) { 	
			dev_err(&client->dev,
				"%s: CALIBRATION_FLAG: copy_from_user() failed.\n", __func__);
			kfree(tmp);
			return -EFAULT;
		}

		ret = i2c_master_send(client,tmp,count);
		if (ret!=count ) {
			dev_err(&client->dev,
				"%s: CALIBRATION: i2c_master_send() failed, ret=%d\n",
				__func__, ret);
			kfree(tmp);
			return -EFAULT;
		}

		while(!attb_read_val()) {
			msleep(100);
			i++;
			if(i>99)
				break;  //10s no high aatb break
		}	//waiting to finish the calibration.(pixcir application_note_710_v3 p43)

		kfree(tmp);
		break;

	case BOOTLOADER:
		memset(bootload_data, 0, sizeof(bootload_data));

		if (copy_from_user(bootload_data, buf, count)) {
			dev_err(&client->dev,
				"%s: BOOTLOADER: copy_from_user() failed.\n", __func__);
			return -EFAULT;
		}

		ret = i2c_master_send(client, bootload_data, count);
		if(ret!=count) {
			dev_err(&client->dev,
				"%s: BOOTLOADER: i2c_master_send() failed, ret = %d\n",
				__func__, ret);
			return -EFAULT;
		}
		break;

	default:
		tmp = kmalloc(count,GFP_KERNEL);
		if (tmp==NULL)
			return -ENOMEM;

		if (copy_from_user(tmp,buf,count)) { 	
			dev_err(&client->dev,
				"%s: default: copy_from_user() failed.\n", __func__);
			kfree(tmp);
			return -EFAULT;
		}
		
		ret = i2c_master_send(client,tmp,count);
		if (ret!=count ) {
			dev_err(&client->dev,
				"%s: default: i2c_master_send() failed, ret=%d\n",
				__func__, ret);
			kfree(tmp);
			return -EFAULT;
		}
		kfree(tmp);
		break;
	}
	return ret;
}

/***********************************Bee-0928****************************************/
/*                        	  pixcir_release                                   */
/***********************************Bee-0928****************************************/
static int pixcir_release(struct inode *inode, struct file *file)
{
	struct i2c_client *client = file->private_data;

	PIXCIR_DBG("enter pixcir_release funtion\n");

	i2c_put_adapter(client->adapter);
	kfree(client);
	file->private_data = NULL;

	return 0;
}

/*********************************Bee-0928-TOP****************************************/
static const struct file_operations pixcir_i2c_ts_fops =
{	.owner		= THIS_MODULE,
	.read		= pixcir_read,
	.write		= pixcir_write,
	.open		= pixcir_open,
	.unlocked_ioctl = pixcir_ioctl,
	.release	= pixcir_release,
};
/*********************************Bee-0928-BOTTOM****************************************/


static const struct i2c_device_id pixcir_i2c_ts_id[] = {
	{ "ms-msg21xx", 0x26 },//0
	{ }
};
MODULE_DEVICE_TABLE(i2c, pixcir_i2c_ts_id);


static struct i2c_driver pixcir_i2c_ts_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "ms-msg21xx",
		//.pm	= &pixcir_dev_pm_ops,
	},
	.probe		= pixcir_i2c_ts_probe,
	.remove		= __devexit_p(pixcir_i2c_ts_remove),
	.id_table	= pixcir_i2c_ts_id,
};

static int __init pixcir_i2c_ts_init(void)
{
	printk("%s\n",__FUNCTION__);
	int ret;
	/*********************************Bee-0928-TOP****************************************/
	ret = register_chrdev(I2C_MAJOR,"pixcir_i2c_ts",&pixcir_i2c_ts_fops);
	if (ret) {
		printk(KERN_ERR "%s:register chrdev failed\n",__FILE__);
		return ret;
	}

	i2c_dev_class = class_create(THIS_MODULE, "pixcir_i2c_dev");
	if (IS_ERR(i2c_dev_class)) {
		ret = PTR_ERR(i2c_dev_class);
		class_destroy(i2c_dev_class);
	}
	/********************************Bee-0928-BOTTOM******************************************/

	return i2c_add_driver(&pixcir_i2c_ts_driver);

}
module_init(pixcir_i2c_ts_init);

static void __exit pixcir_i2c_ts_exit(void)
{
	i2c_del_driver(&pixcir_i2c_ts_driver);
	/********************************Bee-0928-TOP******************************************/
	class_destroy(i2c_dev_class);
	unregister_chrdev(I2C_MAJOR,"pixcir_i2c_ts");
	/********************************Bee-0928-BOTTOM******************************************/
}







module_exit(pixcir_i2c_ts_exit);

MODULE_AUTHOR("Jianchun Bian <jcbian@pixcir.com.cn>");
MODULE_DESCRIPTION("Pixcir I2C Touchscreen Driver");
MODULE_LICENSE("GPL");

