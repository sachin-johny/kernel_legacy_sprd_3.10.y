/*
 * drivers/input/touchscreen/ft5x0x_ts.c
 *
 * FocalTech ft5x0x TouchScreen driver.
 *
 * Copyright (c) 2010  Focal tech Ltd.
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
 * VERSION      	DATE			AUTHOR
 *    1.0		  2010-01-05			WenFS
 *
 * note: only support mulititouch	Wenfs 2010-10-01
 */

#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/i2c/ft53x6_ts.h>
#include <mach/regulator.h>
#include <linux/input/mt.h>

#ifdef CONFIG_I2C_SPRD
#include <mach/i2c-sprd.h>
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

//#define FT53X6_DBG
#ifdef FT53X6_DBG
#define ENTER printk(KERN_INFO "[FT53X6_DBG] func: %s  line: %04d\n", __func__, __LINE__);
#define PRINT_DBG(x...)  printk(KERN_INFO "[FT53X6_DBG] " x)
#define PRINT_INFO(x...)  printk(KERN_INFO "[FT53X6_INFO] " x)
#define PRINT_WARN(x...)  printk(KERN_INFO "[FT53X6_WARN] " x)
#define PRINT_ERR(format,x...)  printk(KERN_ERR "[FT53X6_ERR] func: %s  line: %04d  info: " format, __func__, __LINE__, ## x)
#else
#define ENTER
#define PRINT_DBG(x...)
#define PRINT_INFO(x...)  printk(KERN_INFO "[FT53X6_INFO] " x)
#define PRINT_WARN(x...)  printk(KERN_INFO "[FT53X6_WARN] " x)
#define PRINT_ERR(format,x...)  printk(KERN_ERR "[FT53X6_ERR] func: %s  line: %04d  info: " format, __func__, __LINE__, ## x)
#endif

#define	USE_THREADED_IRQ	1
#define	TOUCH_VIRTUAL_KEYS
#define	MULTI_PROTOCOL_TYPE_B	0
#define	TS_MAX_FINGER		5

#define	FTS_PACKET_LENGTH	128

static unsigned char FT5316_FW[]=
{
#include "ft5316_720p.i"
};

static unsigned char FT5306_FW[]=
{
#include "ft5306_qHD.i"
};

static unsigned char *CTPM_FW = FT5306_FW;
static int fw_size;

static struct ft5x0x_ts_data *g_ft5x0x_ts;
static struct i2c_client *this_client;

static unsigned char suspend_flag = 0;

/* Attribute */
static ssize_t ft5x0x_show_suspend(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t ft5x0x_store_suspend(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t ft5x0x_show_version(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t ft5x0x_update(struct device* cd, struct device_attribute *attr, const char* buf, size_t len);

static unsigned char ft5x0x_read_fw_ver(void);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ft5x0x_ts_suspend(struct early_suspend *handler);
static void ft5x0x_ts_resume(struct early_suspend *handler);
#endif
//static int fts_ctpm_fw_update(void);
static int fts_ctpm_fw_upgrade_with_i_file(void);

struct ts_event {
	u16	x1;
	u16	y1;
	u16	x2;
	u16	y2;
	u16	x3;
	u16	y3;
	u16	x4;
	u16	y4;
	u16	x5;
	u16	y5;
	u16	pressure;
    u8  touch_point;
};

struct ft5x0x_ts_data {
	struct input_dev	*input_dev;
	struct i2c_client	*client;
	struct ts_event	event;
#if !USE_THREADED_IRQ
	struct work_struct	pen_event_work;
	struct workqueue_struct	*ts_workqueue;
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct work_struct       resume_work;
	struct workqueue_struct *ts_resume_workqueue;
	struct early_suspend	early_suspend;
#endif
	struct ft5x0x_ts_platform_data	*platform_data;
};


static DEVICE_ATTR(suspend, S_IRUGO | S_IWUSR, ft5x0x_show_suspend, ft5x0x_store_suspend);
static DEVICE_ATTR(update, S_IRUGO | S_IWUSR, ft5x0x_show_version, ft5x0x_update);

static ssize_t ft5x0x_show_suspend(struct device* cd,
				     struct device_attribute *attr, char* buf)
{
	ssize_t ret = 0;

	if(suspend_flag==1)
		sprintf(buf, "FT5x0x Suspend\n");
	else
		sprintf(buf, "FT5x0x Resume\n");

	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t ft5x0x_store_suspend(struct device* cd, struct device_attribute *attr,
		       const char* buf, size_t len)
{
	unsigned long on_off = simple_strtoul(buf, NULL, 10);
	suspend_flag = on_off;

	if(on_off==1)
	{
		pr_info("FT5x0x Entry Suspend\n");
	#ifdef CONFIG_HAS_EARLYSUSPEND
		ft5x0x_ts_suspend(NULL);
	#endif
	}
	else
	{
		pr_info("FT5x0x Entry Resume\n");
	#ifdef CONFIG_HAS_EARLYSUSPEND
		ft5x0x_ts_resume(NULL);
	#endif
	}

	return len;
}

static ssize_t ft5x0x_show_version(struct device* cd,
				     struct device_attribute *attr, char* buf)
{
	ssize_t ret = 0;
	unsigned char uc_reg_value; 

	uc_reg_value = ft5x0x_read_fw_ver();

	sprintf(buf, "ft5x0x firmware id is V%x\n", uc_reg_value);

	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t ft5x0x_update(struct device* cd, struct device_attribute *attr,
		       const char* buf, size_t len)
{
	unsigned long on_off = simple_strtoul(buf, NULL, 10);
	unsigned char uc_reg_value;

	uc_reg_value = ft5x0x_read_fw_ver();

	if(on_off==1)
	{
		pr_info("ft5x0x update, current firmware id is V%x\n", uc_reg_value);
		//fts_ctpm_fw_update();
		fts_ctpm_fw_upgrade_with_i_file();
	}

	return len;
}

static int ft5x0x_create_sysfs(struct i2c_client *client)
{
	int err;
	struct device *dev = &(client->dev);

	err = device_create_file(dev, &dev_attr_suspend);
	err = device_create_file(dev, &dev_attr_update);

	return err;
}

static int ft5x0x_i2c_rxdata(char *rxdata, int length)
{
	int ret = 0;

	struct i2c_msg msgs[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rxdata,
		},
		{
			.addr	= this_client->addr,
			.flags	= I2C_M_RD,
			.len	= length,
			.buf	= rxdata,
		},
	};

	if (i2c_transfer(this_client->adapter, msgs, 2) != 2) {
		ret = -EIO;
		pr_err("msg %s i2c read error: %d\n", __func__, ret);
	}

	return ret;
}

static int ft5x0x_i2c_txdata(char *txdata, int length)
{
	int ret = 0;

	struct i2c_msg msg[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= length,
			.buf	= txdata,
		},
	};

	if (i2c_transfer(this_client->adapter, msg, 1) != 1) {
		ret = -EIO;
		pr_err("%s i2c write error: %d\n", __func__, ret);
	}

	return ret;
}

/***********************************************************************************************
Name	:	 ft5x0x_write_reg

Input	:	addr -- address
                     para -- parameter

Output	:

function	:	write register of ft5x0x

***********************************************************************************************/
static int ft5x0x_write_reg(u8 addr, u8 para)
{
	u8 buf[3];
	int ret = -1;

	buf[0] = addr;
	buf[1] = para;
	ret = ft5x0x_i2c_txdata(buf, 2);
	if (ret < 0) {
		pr_err("write reg failed! %#x ret: %d", buf[0], ret);
		return -1;
	}

	return 0;
}

/***********************************************************************************************
Name	:	ft5x0x_read_reg

Input	:	addr
                     pdata

Output	:

function	:	read register of ft5x0x

***********************************************************************************************/
static int ft5x0x_read_reg(u8 addr, u8 *pdata)
{
	int ret = 0;
	u8 buf[2] = {addr, 0};

	struct i2c_msg msgs[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= buf,
		},
		{
			.addr	= this_client->addr,
			.flags	= I2C_M_RD,
			.len	= 1,
			.buf	= buf+1,
		},
	};

	if (i2c_transfer(this_client->adapter, msgs, 2) != 2) {
		ret = -EIO;
		pr_err("msg %s i2c read error: %d\n", __func__, ret);
	}

	*pdata = buf[1];
	return ret;
}
#ifdef TOUCH_VIRTUAL_KEYS

static ssize_t virtual_keys_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	unsigned char uc_reg_value;

	ft5x0x_read_reg(FT5X0X_REG_CIPHER, &uc_reg_value);

	if (uc_reg_value == 0x0a || uc_reg_value == 0x0)
		return sprintf(buf,
        		 __stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":133:1360:107:87"
	 		":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE)   ":373:1360:107:87"
	 		":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":627:1360:107:87"
			"\n");
	else
		return sprintf(buf,
         		__stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":100:1020:80:65"
	 		":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE)   ":280:1020:80:65"
	 		":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":470:1020:80:65"
	 		"\n");
}

static struct kobj_attribute virtual_keys_attr = {
    .attr = {
        .name = "virtualkeys.ft5x0x_ts",
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

static void ft5x0x_ts_virtual_keys_init(void)
{
    int ret = 0;
    struct kobject *properties_kobj;

    pr_info("[FST] %s\n",__func__);

    properties_kobj = kobject_create_and_add("board_properties", NULL);
    if (properties_kobj)
        ret = sysfs_create_group(properties_kobj,
                     &properties_attr_group);
    if (!properties_kobj || ret)
        pr_err("failed to create board_properties\n");
}

#endif

/***********************************************************************************************
Name	:	 ft5x0x_read_fw_ver

Input	:	 void

Output	:	 firmware version

function	:	 read TP firmware version

***********************************************************************************************/
static unsigned char ft5x0x_read_fw_ver(void)
{
	unsigned char ver;
	ft5x0x_read_reg(FT5X0X_REG_FIRMID, &ver);
	return(ver);
}

#define CONFIG_SUPPORT_FTS_CTP_UPG
#ifdef CONFIG_SUPPORT_FTS_CTP_UPG

typedef enum
{
    ERR_OK,
    ERR_MODE,
    ERR_READID,
    ERR_ERASE,
    ERR_STATUS,
    ERR_ECC,
    ERR_DL_ERASE_FAIL,
    ERR_DL_PROGRAM_FAIL,
    ERR_DL_VERIFY_FAIL
}E_UPGRADE_ERR_TYPE;

typedef unsigned char         FTS_BYTE;     //8 bit
typedef unsigned short        FTS_WORD;    //16 bit
typedef unsigned int          FTS_DWRD;    //16 bit
typedef unsigned char         FTS_BOOL;    //8 bit

#define FTS_NULL                0x0
#define FTS_TRUE                0x01
#define FTS_FALSE              0x0

#define I2C_CTPM_ADDRESS       0x70

/*
[function]:
    callback: read data from ctpm by i2c interface,implemented by special user;
[parameters]:
    bt_ctpm_addr[in]    :the address of the ctpm;
    pbt_buf[out]        :data buffer;
    dw_lenth[in]        :the length of the data buffer;
[return]:
    FTS_TRUE     :success;
    FTS_FALSE    :fail;
*/
FTS_BOOL i2c_read_interface(FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
    int ret;

    ret=i2c_master_recv(this_client, pbt_buf, dw_lenth);

    if(ret<=0)
    {
        printk("[TSP]i2c_read_interface error\n");
        return FTS_FALSE;
    }

    return FTS_TRUE;
}

/*
[function]:
    callback: write data to ctpm by i2c interface,implemented by special user;
[parameters]:
    bt_ctpm_addr[in]    :the address of the ctpm;
    pbt_buf[in]        :data buffer;
    dw_lenth[in]        :the length of the data buffer;
[return]:
    FTS_TRUE     :success;
    FTS_FALSE    :fail;
*/
FTS_BOOL i2c_write_interface(FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
    int ret;
    ret=i2c_master_send(this_client, pbt_buf, dw_lenth);
    if(ret<=0)
    {
        printk("[TSP]i2c_write_interface error line = %d, ret = %d\n", __LINE__, ret);
        return FTS_FALSE;
    }

    return FTS_TRUE;
}

/*
[function]:
    send a command to ctpm.
[parameters]:
    btcmd[in]        :command code;
    btPara1[in]    :parameter 1;
    btPara2[in]    :parameter 2;
    btPara3[in]    :parameter 3;
    num[in]        :the valid input parameter numbers, if only command code needed and no parameters followed,then the num is 1;
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
FTS_BOOL cmd_write(FTS_BYTE btcmd,FTS_BYTE btPara1,FTS_BYTE btPara2,FTS_BYTE btPara3,FTS_BYTE num)
{
    FTS_BYTE write_cmd[4] = {0};

    write_cmd[0] = btcmd;
    write_cmd[1] = btPara1;
    write_cmd[2] = btPara2;
    write_cmd[3] = btPara3;
    return i2c_write_interface(I2C_CTPM_ADDRESS, write_cmd, num);
}

/*
[function]:
    write data to ctpm , the destination address is 0.
[parameters]:
    pbt_buf[in]    :point to data buffer;
    bt_len[in]        :the data numbers;
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
FTS_BOOL byte_write(FTS_BYTE* pbt_buf, FTS_DWRD dw_len)
{
    return i2c_write_interface(I2C_CTPM_ADDRESS, pbt_buf, dw_len);
}

/*
[function]:
    read out data from ctpm,the destination address is 0.
[parameters]:
    pbt_buf[out]    :point to data buffer;
    bt_len[in]        :the data numbers;
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
FTS_BOOL byte_read(FTS_BYTE* pbt_buf, FTS_BYTE bt_len)
{
    return i2c_read_interface(I2C_CTPM_ADDRESS, pbt_buf, bt_len);
}


/*
[function]:
    burn the FW to ctpm.
[parameters]:(ref. SPEC)
    pbt_buf[in]    :point to Head+FW ;
    dw_lenth[in]:the length of the FW + 6(the Head length);
    bt_ecc[in]    :the ECC of the FW
[return]:
    ERR_OK        :no error;
    ERR_MODE    :fail to switch to UPDATE mode;
    ERR_READID    :read id fail;
    ERR_ERASE    :erase chip fail;
    ERR_STATUS    :status error;
    ERR_ECC        :ecc error.
*/

E_UPGRADE_ERR_TYPE  fts_ctpm_fw_upgrade(FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
    FTS_BYTE reg_val[2] = {0};
    FTS_DWRD i = 0;

    FTS_DWRD  packet_number;
    FTS_DWRD  j;
    FTS_DWRD  temp;
    FTS_DWRD  lenght;
    FTS_BYTE  packet_buf[FTS_PACKET_LENGTH + 6];
    FTS_BYTE  auc_i2c_write_buf[10];
    FTS_BYTE bt_ecc;
    int      i_ret;

    /*********Step 1:Reset  CTPM *****/
    /*write 0xaa to register 0xfc*/
    ft5x0x_write_reg(0xfc,0xaa);
    msleep(50);
     /*write 0x55 to register 0xfc*/
    ft5x0x_write_reg(0xfc,0x55);
    printk("[TSP] Step 1: Reset CTPM test, bin-length=%d\n",dw_lenth);

    msleep(100);

    /*********Step 2:Enter upgrade mode *****/
    do
    {
        i ++;
        auc_i2c_write_buf[0] = 0x55;
	i_ret = ft5x0x_i2c_txdata(auc_i2c_write_buf, 1);
        i_ret |= auc_i2c_write_buf[0] = 0xaa;
        ft5x0x_i2c_txdata(auc_i2c_write_buf, 1);
        msleep(5);
    }while(i_ret <= 0 && i < 5 );
    /*********Step 3:check READ-ID***********************/
    cmd_write(0x90,0x00,0x00,0x00,4);
    byte_read(reg_val,2);
    if (reg_val[0] == 0x79 && (reg_val[1] == 0x07 || reg_val[1] == 0x03))
    {
        printk("[TSP] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
    }
    else
    {
        printk("%s: ERR_READID, ID1 = 0x%x,ID2 = 0x%x\n", __func__,reg_val[0],reg_val[1]);
        return ERR_READID;
        //i_is_new_protocol = 1;
    }

     /*********Step 4:erase app*******************************/
    cmd_write(0x61,0x00,0x00,0x00,1);

    msleep(1500);
    printk("[TSP] Step 4: erase.\n");

    /*********Step 5:write firmware(FW) to ctpm flash*********/
    bt_ecc = 0;
    printk("[TSP] Step 5: start upgrade.\n");
    dw_lenth = dw_lenth - 8;
    packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
    packet_buf[0] = 0xbf;
    packet_buf[1] = 0x00;
    for (j=0;j<packet_number;j++)
    {
        temp = j * FTS_PACKET_LENGTH;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;
        lenght = FTS_PACKET_LENGTH;
        packet_buf[4] = (FTS_BYTE)(lenght>>8);
        packet_buf[5] = (FTS_BYTE)lenght;

        for (i=0;i<FTS_PACKET_LENGTH;i++)
        {
            packet_buf[6+i] = pbt_buf[j*FTS_PACKET_LENGTH + i];
            bt_ecc ^= packet_buf[6+i];
        }

        byte_write(&packet_buf[0],FTS_PACKET_LENGTH + 6);
        msleep(FTS_PACKET_LENGTH/6 + 1);
        if ((j * FTS_PACKET_LENGTH % 1024) == 0)
        {
              printk("[TSP] upgrade the 0x%x th byte.\n", ((unsigned int)j) * FTS_PACKET_LENGTH);
        }
    }

    if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
    {
        temp = packet_number * FTS_PACKET_LENGTH;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;

        temp = (dw_lenth) % FTS_PACKET_LENGTH;
        packet_buf[4] = (FTS_BYTE)(temp>>8);
        packet_buf[5] = (FTS_BYTE)temp;

        for (i=0;i<temp;i++)
        {
            packet_buf[6+i] = pbt_buf[ packet_number*FTS_PACKET_LENGTH + i]; 
            bt_ecc ^= packet_buf[6+i];
        }

        byte_write(&packet_buf[0],temp+6);
        msleep(20);
    }

    //send the last six byte
    for (i = 0; i<6; i++)
    {
        temp = 0x6ffa + i;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;
        temp =1;
        packet_buf[4] = (FTS_BYTE)(temp>>8);
        packet_buf[5] = (FTS_BYTE)temp;
        packet_buf[6] = pbt_buf[ dw_lenth + i];
        bt_ecc ^= packet_buf[6];

        byte_write(&packet_buf[0],7);
        msleep(20);
    }

    /*********Step 6: read out checksum***********************/
    /*send the opration head*/
    cmd_write(0xcc,0x00,0x00,0x00,1);
    byte_read(reg_val,1);
    printk("[TSP] Step 6:  ecc read 0x%x, new firmware 0x%x. \n", reg_val[0], bt_ecc);
    if(reg_val[0] != bt_ecc)
    {
        printk("%s: ERR_ECC\n", __func__);
        return ERR_ECC;
    }

    /*********Step 7: reset the new FW***********************/
    cmd_write(0x07,0x00,0x00,0x00,1);

    return ERR_OK;
}

int fts_ctpm_auto_clb(void)
{
    unsigned char uc_temp;
    unsigned char i ;

    printk("[FTS] start auto CLB.\n");
    msleep(200);
    ft5x0x_write_reg(0, 0x40);
    msleep(100);   //make sure already enter factory mode
    ft5x0x_write_reg(2, 0x4);  //write command to start calibration
    msleep(300);
    for(i=0;i<100;i++)
    {
        ft5x0x_read_reg(0,&uc_temp);
        if ( ((uc_temp&0x70)>>4) == 0x0)  //return to normal mode, calibration finish
        {
            break;
        }
        msleep(200);
        printk("[FTS] waiting calibration %d\n",i);
    }
    printk("[FTS] calibration OK.\n");

    msleep(300);
    ft5x0x_write_reg(0, 0x40);  //goto factory mode
    msleep(100);   //make sure already enter factory mode
    ft5x0x_write_reg(2, 0x5);  //store CLB result
    msleep(300);
    ft5x0x_write_reg(0, 0x0); //return to normal mode
    msleep(300);
    printk("[FTS] store CLB result OK.\n");
    return 0;
}

int fts_ctpm_fw_upgrade_with_i_file(void)
{
   FTS_BYTE*     pbt_buf = FTS_NULL;
   int i_ret;

    //=========FW upgrade========================*/
   pbt_buf = CTPM_FW;
   /*call the upgrade function*/
   i_ret =  fts_ctpm_fw_upgrade(pbt_buf,fw_size);
   if (i_ret != 0)
   {
	printk("[FTS] upgrade failed i_ret = %d.\n", i_ret);
       //error handling ...
       //TBD
   }
   else
   {
       printk("[FTS] upgrade successfully.\n");
       fts_ctpm_auto_clb();  //start auto CLB
   }

   return i_ret;
}

#if 0
static int fts_ctpm_fw_update(void)
{
    int ret = 0;
    const struct firmware *fw;
    unsigned char *fw_buf;
    struct platform_device *pdev;

    pdev = platform_device_register_simple("ft5206_ts", 0, NULL, 0);
    if (IS_ERR(pdev)) {
        printk("%s: Failed to register firmware\n", __func__);
        return -1;
    }

    ret = request_firmware(&fw, "ft5306_fw.bin", &pdev->dev);
    if (ret) {
		printk("%s: request_firmware error\n",__func__);
		platform_device_unregister(pdev);
        return -1;
    }

    platform_device_unregister(pdev);
    printk("%s:fw size=%d\n", __func__,fw->size);

    fw_buf = kzalloc(fw->size, GFP_KERNEL | GFP_DMA);
    memcpy(fw_buf, fw->data, fw->size);

    fts_ctpm_fw_upgrade(fw_buf, fw->size);

    printk("%s: update finish\n", __func__);
    release_firmware(fw);
    kfree(fw_buf);

    return 0;
}
#endif
#endif

static void ft5x0x_clear_report_data(struct ft5x0x_ts_data *ft5x0x_ts)
{
	int i;

	for(i = 0; i < TS_MAX_FINGER; i++) {
	#if MULTI_PROTOCOL_TYPE_B
		input_mt_slot(ft5x0x_ts->input_dev, i);
		input_mt_report_slot_state(ft5x0x_ts->input_dev, MT_TOOL_FINGER, false);
	#endif
		input_report_key(ft5x0x_ts->input_dev, BTN_TOUCH, 0);
	#if !MULTI_PROTOCOL_TYPE_B
		input_mt_sync(ft5x0x_ts->input_dev);
	#endif
	}
	input_sync(ft5x0x_ts->input_dev);
}

static int ft5x0x_update_data(void)
{
	struct ft5x0x_ts_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;
	u8 buf[32] = {0};
	int ret = -1;
	int i;
	u16 x , y;

	ret = ft5x0x_i2c_rxdata(buf, 31);

	if (ret < 0) {
		pr_err("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
		return ret;
	}

	memset(event, 0, sizeof(struct ts_event));
	event->touch_point = buf[2] & 0x07;

	for(i = 0; i < TS_MAX_FINGER; i++) {
		if((buf[6*i+3] & 0xc0) == 0xc0)
			continue;
		x = (s16)(buf[6*i+3] & 0x0F)<<8 | (s16)buf[6*i+4];	
		y = (s16)(buf[6*i+5] & 0x0F)<<8 | (s16)buf[6*i+6];
		if((buf[6*i+3] & 0x40) == 0x0) {
		#if MULTI_PROTOCOL_TYPE_B
			input_mt_slot(data->input_dev, buf[6*i+5]>>4);
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, true);
		#endif
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, x);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, y);
			input_report_key(data->input_dev, BTN_TOUCH, 1);
		#if !MULTI_PROTOCOL_TYPE_B
			input_mt_sync(data->input_dev);
		#endif
			pr_debug("===x%d = %d,y%d = %d ====",i, x, i, y);
		}
		else {
		#if MULTI_PROTOCOL_TYPE_B
			input_mt_slot(data->input_dev, buf[6*i+5]>>4);
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
		#endif
		}
	}
	if(event->touch_point==0)
	{
		input_report_key(data->input_dev, BTN_TOUCH, 0);
		#if !MULTI_PROTOCOL_TYPE_B
			input_mt_sync(data->input_dev);
		#endif

	}
	input_sync(data->input_dev);

	return 0;
}

#if !USE_THREADED_IRQ
static void ft5x0x_ts_pen_irq_work(struct work_struct *work)
{
	ft5x0x_update_data();
	enable_irq(this_client->irq);
}
#endif

static irqreturn_t ft5x0x_ts_interrupt(int irq, void *dev_id)
{
#if !USE_THREADED_IRQ
	struct ft5x0x_ts_data *ft5x0x_ts = (struct ft5x0x_ts_data *)dev_id;

	if (!work_pending(&ft5x0x_ts->pen_event_work)) {
		queue_work(ft5x0x_ts->ts_workqueue, &ft5x0x_ts->pen_event_work);
	}
#else
	ft5x0x_update_data();
#endif

	return IRQ_HANDLED;
}

static void ft5x0x_ts_reset(void)
{
	struct ft5x0x_ts_platform_data *pdata = g_ft5x0x_ts->platform_data;

	gpio_direction_output(pdata->reset_gpio_number, 1);
	msleep(1);
	gpio_set_value(pdata->reset_gpio_number, 0);
	msleep(10);
	gpio_set_value(pdata->reset_gpio_number, 1);
	msleep(200);
}

#if 0
//for future use

struct regulator *vdd28 = NULL;

static void ft53x6_power_off(void)
{
	if(vdd28 != NULL)
		regulator_force_disable(vdd28);
	PRINT_INFO("power off\n");
}

static void ft53x6_power_on(void)
{
	int err = 0;

	if(vdd28 == NULL) {
		vdd28 = regulator_get(NULL, "vdd28");
		if (IS_ERR(vdd28)) {
			PRINT_ERR("regulator_get failed\n");
			return;
		}
		err = regulator_set_voltage(vdd28,2800000,2800000);
		if (err)
			PRINT_ERR("regulator_set_voltage failed\n");
	}
	regulator_enable(vdd28);

	PRINT_INFO("power on\n");
}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ft5x0x_ts_suspend(struct early_suspend *handler)
{
	int ret = -1;
	int count = 5;
	pr_info("==%s==\n", __FUNCTION__);
	ret = ft5x0x_write_reg(FT5X0X_REG_PMODE, PMODE_HIBERNATE);
	while(ret == 0 && count != 0) {
			PRINT_ERR("trying to enter hibernate again. ret = %d\n", ret);
			msleep(10);
			ret = ft5x0x_write_reg(FT5X0X_REG_PMODE, PMODE_HIBERNATE);
			count--;
	}
	disable_irq(this_client->irq);
	ft5x0x_clear_report_data(g_ft5x0x_ts);
	//gpio_set_value(g_ft5x0x_ts->platform_data->reset_gpio_number, 0);//for future use
}

static void ft5x0x_ts_resume(struct early_suspend *handler)
{
	struct ft5x0x_ts_data  *ft5x0x_ts = (struct ft5x0x_ts_data *)i2c_get_clientdata(this_client);
	queue_work(ft5x0x_ts->ts_resume_workqueue, &ft5x0x_ts->resume_work);
}

static void ft5x0x_ts_resume_work(struct work_struct *work)
{
	pr_info("==%s==\n", __FUNCTION__);
	enable_irq(this_client->irq);
	ft5x0x_ts_reset();
	ft5x0x_write_reg(FT5X0X_REG_PERIODACTIVE, 7);
}
#endif

static void ft5x0x_ts_hw_init(struct ft5x0x_ts_data *ft5x0x_ts)
{
	struct regulator *reg_vdd;
	struct i2c_client *client = ft5x0x_ts->client;
	struct ft5x0x_ts_platform_data *pdata = ft5x0x_ts->platform_data;

	pr_info("[FST] %s [irq=%d];[rst=%d]\n",__func__,
		pdata->irq_gpio_number,pdata->reset_gpio_number);
	gpio_request(pdata->irq_gpio_number, "ts_irq_pin");
	gpio_request(pdata->reset_gpio_number, "ts_rst_pin");
	gpio_direction_output(pdata->reset_gpio_number, 1);
	gpio_direction_input(pdata->irq_gpio_number);

	reg_vdd = regulator_get(&client->dev, pdata->vdd_name);
	if (!WARN(IS_ERR(reg_vdd), "[FST] ft5x0x_ts_hw_init regulator: failed to get %s.\n", pdata->vdd_name)) {
		regulator_set_voltage(reg_vdd, 2800000, 2800000);
		regulator_enable(reg_vdd);
	}
	msleep(100);
	ft5x0x_ts_reset();
}

static int ft5x0x_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ft5x0x_ts_data *ft5x0x_ts;
	struct input_dev *input_dev;
	struct ft5x0x_ts_platform_data *pdata = client->dev.platform_data;
	int err = 0;
	unsigned char uc_reg_value;

	pr_info("[FST] %s: probe\n",__func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	ft5x0x_ts = kzalloc(sizeof(*ft5x0x_ts), GFP_KERNEL);
	if (!ft5x0x_ts)	{
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	g_ft5x0x_ts = ft5x0x_ts;
	ft5x0x_ts->platform_data = pdata;
	this_client = client;
	ft5x0x_ts->client = client;
	ft5x0x_ts_hw_init(ft5x0x_ts);
	i2c_set_clientdata(client, ft5x0x_ts);
	client->irq = gpio_to_irq(pdata->irq_gpio_number);

	#ifdef CONFIG_I2C_SPRD
	sprd_i2c_ctl_chg_clk(client->adapter->nr, 400000);
	#endif

	err = ft5x0x_read_reg(FT5X0X_REG_CIPHER, &uc_reg_value);
	if (err < 0)
	{
		pr_err("[FST] read chip id error %x\n", uc_reg_value);
		err = -ENODEV;
		goto exit_alloc_data_failed;
	}

	pr_info("[FST] FT5X0X_REG_CIPHER is 0x%x\n",uc_reg_value);

	/*
	 * Normally FT5306 chip id is 0x55, FT5316 chip id is 0x0a,
	 * but if the device power off when downloading firmware,
	 * FT5306 chip id changes to 0xa3, FT5316 changes to 0x0,
	 * and the firmware need to be downloaded first next time.
	 */
	if (uc_reg_value == 0x55 || uc_reg_value == 0xa3) {
		CTPM_FW = FT5306_FW;
		fw_size = sizeof(FT5306_FW);
		if(uc_reg_value == 0xa3) {
			msleep(100);
			fts_ctpm_fw_upgrade_with_i_file();
		}
	} else if (uc_reg_value == 0x0a || uc_reg_value == 0x0) {
		CTPM_FW = FT5316_FW;
		fw_size = sizeof(FT5316_FW);
		if(uc_reg_value == 0x0) {
			msleep(100);
			fts_ctpm_fw_upgrade_with_i_file();
		}
	} else {
		pr_err("[FST] unknown chip id %x\n", uc_reg_value);
		err = -ENODEV;
		goto exit_chip_check_failed;
	}

	/* set report rate, about 70HZ */
	ft5x0x_write_reg(FT5X0X_REG_PERIODACTIVE, 7);
#if !USE_THREADED_IRQ
	INIT_WORK(&ft5x0x_ts->pen_event_work, ft5x0x_ts_pen_irq_work);

	ft5x0x_ts->ts_workqueue = create_singlethread_workqueue(dev_name(&client->dev));
	if (!ft5x0x_ts->ts_workqueue) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	INIT_WORK(&ft5x0x_ts->resume_work, ft5x0x_ts_resume_work);
	ft5x0x_ts->ts_resume_workqueue = create_singlethread_workqueue("ft5x0x_ts_resume_work");
	if (!ft5x0x_ts->ts_resume_workqueue) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}
#endif
	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "[FST] failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}
#ifdef TOUCH_VIRTUAL_KEYS
	ft5x0x_ts_virtual_keys_init();
#endif
	ft5x0x_ts->input_dev = input_dev;

	__set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	__set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	__set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	__set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);
	__set_bit(KEY_MENU,  input_dev->keybit);
	__set_bit(KEY_BACK,  input_dev->keybit);
	__set_bit(KEY_HOMEPAGE,  input_dev->keybit);
	__set_bit(BTN_TOUCH, input_dev->keybit);

#if MULTI_PROTOCOL_TYPE_B
	input_mt_init_slots(input_dev, TS_MAX_FINGER);
#endif

	/*ft5306's firmware is qhd, ft5316's firmware is 720p*/
	if (uc_reg_value == 0x0a || uc_reg_value == 0x0) {
		input_set_abs_params(input_dev,
			     ABS_MT_POSITION_X, 0, 720, 0, 0);
		input_set_abs_params(input_dev,
			     ABS_MT_POSITION_Y, 0, 1280, 0, 0);
	} else {
		input_set_abs_params(input_dev,
			     ABS_MT_POSITION_X, 0, 540, 0, 0);
		input_set_abs_params(input_dev,
			     ABS_MT_POSITION_Y, 0, 960, 0, 0);
	}
	input_set_abs_params(input_dev,
			     ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);

	set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);

	input_dev->name = FT5X0X_NAME;
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
		"[FST] ft5x0x_ts_probe: failed to register input device: %s\n",
		dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}

#if USE_THREADED_IRQ
	err = request_threaded_irq(client->irq, NULL, ft5x0x_ts_interrupt, 
		IRQF_TRIGGER_FALLING | IRQF_ONESHOT, client->name, ft5x0x_ts);
#else
	err = request_irq(client->irq, ft5x0x_ts_interrupt,
		IRQF_TRIGGER_FALLING | IRQF_ONESHOT, client->name, ft5x0x_ts);
#endif
	if (err < 0) {
		dev_err(&client->dev, "[FST] ft5x0x_probe: request irq failed %d\n",err);
		goto exit_irq_request_failed;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	ft5x0x_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ft5x0x_ts->early_suspend.suspend = ft5x0x_ts_suspend;
	ft5x0x_ts->early_suspend.resume	= ft5x0x_ts_resume;
	register_early_suspend(&ft5x0x_ts->early_suspend);
#endif

	/* get chip firmware version */
	uc_reg_value = ft5x0x_read_fw_ver();
	pr_info("[FST] Firmware version = 0x%x\n", uc_reg_value);
	pr_info("[FST] New Firmware version = 0x%x\n", CTPM_FW[fw_size-2]);

	if(uc_reg_value != CTPM_FW[fw_size-2])
	{
		fts_ctpm_fw_upgrade_with_i_file();
	}

	ft5x0x_create_sysfs(client);

	return 0;

exit_irq_request_failed:
	input_unregister_device(input_dev);
exit_input_register_device_failed:
	input_free_device(input_dev);
exit_input_dev_alloc_failed:
exit_create_singlethread:
exit_chip_check_failed:
	kfree(ft5x0x_ts);
exit_alloc_data_failed:
exit_check_functionality_failed:
	ft5x0x_ts = NULL;
	i2c_set_clientdata(client, ft5x0x_ts);
	return err;
}

static int __devexit ft5x0x_ts_remove(struct i2c_client *client)
{
	struct ft5x0x_ts_data *ft5x0x_ts = i2c_get_clientdata(client);

	pr_info("==ft5x0x_ts_remove=\n");

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ft5x0x_ts->early_suspend);
#endif
	free_irq(client->irq, ft5x0x_ts);
	input_unregister_device(ft5x0x_ts->input_dev);
	input_free_device(ft5x0x_ts->input_dev);
#if !USE_THREADED_IRQ
	cancel_work_sync(&ft5x0x_ts->pen_event_work);
	destroy_workqueue(ft5x0x_ts->ts_workqueue);
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
	cancel_work_sync(&ft5x0x_ts->resume_work);
	destroy_workqueue(ft5x0x_ts->ts_resume_workqueue);
#endif
	kfree(ft5x0x_ts);
	ft5x0x_ts = NULL;
	i2c_set_clientdata(client, ft5x0x_ts);

	return 0;
}

static const struct i2c_device_id ft5x0x_ts_id[] = {
	{ FT5X0X_NAME, 0 },{ }
};

static int ft5x0x_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret = -1;
	int count = 5;
	PRINT_INFO("ft5x0x_suspend\n");
	ret = ft5x0x_write_reg(FT5X0X_REG_PMODE, PMODE_HIBERNATE);
	if(ret != 0)
		PRINT_INFO("CTP is in hibernate mode. ret = %d\n", ret);
	while(ret == 0 && count != 0) {
			PRINT_ERR("trying to enter hibernate again. ret = %d\n", ret);
			msleep(10);
			ret = ft5x0x_write_reg(FT5X0X_REG_PMODE, PMODE_HIBERNATE);
			count--;
	}
	return 0;
}
static int ft5x0x_resume(struct i2c_client *client)
{
	PRINT_INFO("ft5x0x_resume\n");
	return 0;
}

MODULE_DEVICE_TABLE(i2c, ft5x0x_ts_id);

static struct i2c_driver ft5x0x_ts_driver = {
	.probe		= ft5x0x_ts_probe,
	.remove		= __devexit_p(ft5x0x_ts_remove),
	.id_table	= ft5x0x_ts_id,
	.driver	= {
		.name	= FT5X0X_NAME,
		.owner	= THIS_MODULE,
	},
	.suspend = ft5x0x_suspend,
	.resume = ft5x0x_resume,
};

static int __init ft5x0x_ts_init(void)
{
	return i2c_add_driver(&ft5x0x_ts_driver);
}

static void __exit ft5x0x_ts_exit(void)
{
	i2c_del_driver(&ft5x0x_ts_driver);
}

module_init(ft5x0x_ts_init);
module_exit(ft5x0x_ts_exit);

MODULE_AUTHOR("<wenfs@Focaltech-systems.com>");
MODULE_DESCRIPTION("FocalTech ft5x0x TouchScreen driver");
MODULE_LICENSE("GPL");
