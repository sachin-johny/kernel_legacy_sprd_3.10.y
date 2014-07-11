/* 
 * drivers/input/touchscreen/msg2133_ts.c
 *
 * FocalTech msg2133 TouchScreen driver.
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
 * VERSION                              AUTHOR      DATE
 *    1.0		                        WenFS    2010-01-05
 *
 * note: only support mulititouch       Wenfs    2010-10-01

 */


#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/earlysuspend.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#include <linux/firmware.h>
#include <linux/platform_device.h>

#include <linux/slab.h>
#include <mach/regulator.h>

#include <linux/i2c/msg2133a.h>


#if TS_DEBUG_MSG
#define MSG2133_DBG(format, ...)	printk(KERN_INFO "MSG2133A " format "\n", ## __VA_ARGS__)
#else
#define MSG2133_DBG(format, ...)
#endif

static int sprd_3rdparty_gpio_tp_rst;
static int sprd_3rdparty_gpio_tp_irq;
static struct regulator *reg_vdd;

static struct i2c_client *this_client;


#ifdef GST_TP_COMPATIBLE
struct msg2133_fw_version {
	unsigned short major;
	unsigned short minor;
};
static struct msg2133_fw_version fw_v;
#endif

// Add for TP proximity support;
#ifdef TP_PROXIMITY_SENSOR
static int PROXIMITY_SWITCH;
static int PROXIMITY_STATE;
static int INCALLING;

static struct spinlock proximity_switch_lock;
static struct spinlock proximity_state_lock;
#endif

/******* firmware update part *********/
static void msg2133_reset(void);
static void msg2133_device_power_on(void);
static u8 g_dwiic_info_data[1024];   // Buffer for info data

#ifdef __FIRMWARE_UPDATE__
#define FW_ADDR_MSG21XX          (0xC4 >> 1)
#define FW_ADDR_MSG21XX_TP       (0x4C >> 1)
#define FW_UPDATE_ADDR_MSG21XX   (0x92 >> 1)
static u8 **temp = NULL;

static u32 crc_tab[256];

static int FwDataCnt;
struct class *firmware_class;
struct device *firmware_cmd_dev;

static void HalTscrCReadI2CSeq(u8 addr, u8* read_data, u16 size)
{
   	//according to your platform.
	int rc;

	struct i2c_msg msgs[] =
    	{
		{
			.addr = addr,
			.flags = I2C_M_RD,
			.len = size,
			.buf = read_data,
		},
	};

	rc = i2c_transfer(this_client->adapter, msgs, 1);
	if( rc < 0 )
    	{
		MSG2133_DBG("HalTscrCReadI2CSeq error %d\n", rc);
	}
}

static void HalTscrCDevWriteI2CSeq(u8 addr, u8* data, u16 size)
{
    //according to your platform.
   	int rc;
	struct i2c_msg msgs[] =
    	{
		{
			.addr = addr,
			.flags = 0,
			.len = size,
			.buf = data,
		},
	};
	rc = i2c_transfer(this_client->adapter, msgs, 1);
	if( rc < 0 )
    	{
		MSG2133_DBG("HalTscrCDevWriteI2CSeq error %d,addr = %d\n", rc,addr);
	}
}

/*
static void Get_Chip_Version(void)
{
    MSG2133_DBG("[%s]: Enter!\n", __func__);
    unsigned char dbbus_tx_data[3];
    unsigned char dbbus_rx_data[2];

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0xCE;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, &dbbus_tx_data[0], 3);
    HalTscrCReadI2CSeq(FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2);
    if (dbbus_rx_data[1] == 0)
    {
        // it is Catch2
        MSG2133_DBG(MSG2133_DBG("*** Catch2 ***\n");)
        //FwVersion  = 2;// 2 means Catch2
    }
    else
    {
        // it is catch1
        MSG2133_DBG(MSG2133_DBG("*** Catch1 ***\n");)
        //FwVersion  = 1;// 1 means Catch1
    }

}
*/

static void dbbusDWIICEnterSerialDebugMode(void)
{
	u8 data[5]; // Enter the Serial Debug Mode
	data[0] = 0x53;
	data[1] = 0x45;
	data[2] = 0x52;
	data[3] = 0x44;
	data[4] = 0x42;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 5);
}

static void dbbusDWIICStopMCU(void)
{
	u8 data[1];

	// Stop the MCU
	data[0] = 0x37;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);
}

static void dbbusDWIICIICUseBus(void)
{
	u8 data[1];

	// IIC Use Bus
	data[0] = 0x35;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);
}

static void dbbusDWIICIICReshape(void)
{
	u8 data[1];

	// IIC Re-shape
	data[0] = 0x71;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);
}

static void dbbusDWIICIICNotUseBus(void)
{
	u8 data[1];

	// IIC Not Use Bus
	data[0] = 0x34;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);
}

static void dbbusDWIICNotStopMCU(void)
{
	u8 data[1];

	// Not Stop the MCU
	data[0] = 0x36;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);
}

static void dbbusDWIICExitSerialDebugMode(void)
{
	u8 data[1];

	// Exit the Serial Debug Mode
	data[0] = 0x45;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);

	// Delay some interval to guard the next transaction
	udelay ( 150);//200 );        // delay about 0.2ms
}

static void drvISP_EntryIspMode(void)
{
	u8 bWriteData[5] =
	{
	    0x4D, 0x53, 0x54, 0x41, 0x52
	};
	    MSG2133_DBG("\n******%s come in*******\n",__FUNCTION__);
	HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 5);
	udelay ( 150 );//200 );        // delay about 0.1ms
}

static u8 drvISP_Read(u8 n, u8* pDataToRead)    //First it needs send 0x11 to notify we want to get flash data back.
{
	u8 Read_cmd = 0x11;
	unsigned char dbbus_rx_data[2] = {0};
	HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &Read_cmd, 1);
	//msctpc_LoopDelay ( 1 );        // delay about 100us*****
	udelay( 800 );//200);
	if (n == 1)
	{
	    HalTscrCReadI2CSeq(FW_UPDATE_ADDR_MSG21XX, &dbbus_rx_data[0], 2);
	    *pDataToRead = dbbus_rx_data[0];
	    //MSG2133_DBG("dbbus=%d,%d===drvISP_Read=====\n",dbbus_rx_data[0],dbbus_rx_data[1]);
	    }
	else
	{
	    HalTscrCReadI2CSeq(FW_UPDATE_ADDR_MSG21XX, pDataToRead, n);
	}

	return 0;
}

static void drvISP_WriteEnable(void)
{
	u8 bWriteData[2] =
	{
	    0x10, 0x06
	};
	u8 bWriteData1 = 0x12;
	HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 2);
	udelay(150);//1.16
	HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);
}


static void drvISP_ExitIspMode(void)
{
	u8 bWriteData = 0x24;
	HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData, 1);
	udelay( 150 );//200);
}

static u8 drvISP_ReadStatus(void)
{
	u8 bReadData = 0;
	u8 bWriteData[2] =
	{
	    0x10, 0x05
	};
	u8 bWriteData1 = 0x12;

	HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 2);
	//msctpc_LoopDelay ( 1 );        // delay about 100us*****
	udelay(150);//200);
	drvISP_Read(1, &bReadData);
	HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);
	return bReadData;
}


#if 0
static void drvISP_BlockErase(u32 addr)
{
    u8 bWriteData[5] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
    u8 bWriteData1 = 0x12;
	MSG2133_DBG("\n******%s come in*******\n",__FUNCTION__);
	u32 timeOutCount=0;
    drvISP_WriteEnable();

    //Enable write status register
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x50;
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 2);
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);

    //Write Status
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x01;
    bWriteData[2] = 0x00;
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 3);
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);

    //Write disable
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x04;
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 2);
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);
	//msctpc_LoopDelay ( 1 );        // delay about 100us*****
	udelay(150);//200);
    timeOutCount=0;
	while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
	{
		timeOutCount++;
		if ( timeOutCount >= 100000 ) break; /* around 1 sec timeout */
	}
    drvISP_WriteEnable();

    bWriteData[0] = 0x10;
    bWriteData[1] = 0xC7;//0xD8;        //Block Erase
    //bWriteData[2] = ((addr >> 16) & 0xFF) ;
    //bWriteData[3] = ((addr >> 8) & 0xFF) ;
    //bWriteData[4] = (addr & 0xFF) ;
	HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 2);
    //HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData, 5);
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);
		//msctpc_LoopDelay ( 1 );        // delay about 100us*****
	udelay(150);//200);
	timeOutCount=0;
	while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
	{
		timeOutCount++;
		if ( timeOutCount >= 500000 ) break; /* around 5 sec timeout */
	}
}
#endif

static void drvISP_Program(u16 k, u8* pDataToWrite)
{
	u16 i = 0;
	u16 j = 0;
	//u16 n = 0;
	u8 TX_data[133];
	u8 bWriteData1 = 0x12;
	u32 addr = k * 1024;
		u32 timeOutCount=0;
	for (j = 0; j < 8; j++)   //128*8 cycle
	{
		TX_data[0] = 0x10;
		TX_data[1] = 0x02;// Page Program CMD
		TX_data[2] = (addr + 128 * j) >> 16;
		TX_data[3] = (addr + 128 * j) >> 8;
		TX_data[4] = (addr + 128 * j);
		for (i = 0; i < 128; i++)
		{
		    TX_data[5 + i] = pDataToWrite[j * 128 + i];
		}
		//msctpc_LoopDelay ( 1 );        // delay about 100us*****
		udelay(150);//200);

		timeOutCount=0;
		while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
		{
			timeOutCount++;
			if ( timeOutCount >= 100000 ) break; /* around 1 sec timeout */
		}
  
		drvISP_WriteEnable();
		HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, TX_data, 133);   //write 133 byte per cycle
		HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);
    	}
}

static ssize_t firmware_update_show ( struct device *dev,
                                      struct device_attribute *attr, char *buf )
{
	return sprintf ( buf, "%03d%03d\n", fw_v.major,fw_v.minor);
}

static void drvISP_Verify ( u16 k, u8* pDataToVerify )
{
    u16 i = 0, j = 0;
    u8 bWriteData[5] ={ 0x10, 0x03, 0, 0, 0 };
    u8 RX_data[256];
    u8 bWriteData1 = 0x12;
    u32 addr = k * 1024;
    u8 index = 0;
    u32 timeOutCount;
    for ( j = 0; j < 8; j++ ) //128*8 cycle
    {
        bWriteData[2] = ( u8 ) ( ( addr + j * 128 ) >> 16 );
        bWriteData[3] = ( u8 ) ( ( addr + j * 128 ) >> 8 );
        bWriteData[4] = ( u8 ) ( addr + j * 128 );
        udelay ( 100 );        // delay about 100us*****

        timeOutCount = 0;
        while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
        {
            timeOutCount++;
            if ( timeOutCount >= 100000 ) break; /* around 1 sec timeout */
        }

        HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, bWriteData, 5 ); //write read flash addr
        udelay ( 100 );        // delay about 100us*****
        drvISP_Read ( 128, RX_data );
        HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1 ); //cmd end
        for ( i = 0; i < 128; i++ ) //log out if verify error
        {
            if ( ( RX_data[i] != 0 ) && index < 10 )
            {
                //MSG2133_DBG("j=%d,RX_data[%d]=0x%x\n",j,i,RX_data[i]);
                index++;
            }
            if ( RX_data[i] != pDataToVerify[128 * j + i] )
            {
               // MSG2133_DBG ( "k=%d,j=%d,i=%d===============Update Firmware Error================", k, j, i );
            }
        }
    }
}

static void drvISP_ChipErase()
{
    u8 bWriteData[5] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
    u8 bWriteData1 = 0x12;
    u32 timeOutCount = 0;
    drvISP_WriteEnable();

    //Enable write status register
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x50;
    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, bWriteData, 2 );
    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1 );

    //Write Status
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x01;
    bWriteData[2] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, bWriteData, 3 );
    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1 );

    //Write disable
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x04;
    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, bWriteData, 2 );
    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1 );
    udelay ( 100 );        // delay about 100us*****
    timeOutCount = 0;
    while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
    {
        timeOutCount++;
        if ( timeOutCount >= 100000 ) break; /* around 1 sec timeout */
    }
    drvISP_WriteEnable();

    bWriteData[0] = 0x10;
    bWriteData[1] = 0xC7;

    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, bWriteData, 2 );
    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1 );
    udelay ( 100 );        // delay about 100us*****
    timeOutCount = 0;
    while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
    {
        timeOutCount++;
        if ( timeOutCount >= 500000 ) break; /* around 5 sec timeout */
    }
}

/* update the firmware part, used by apk*/
/*show the fw version*/

static ssize_t firmware_update_c2 ( struct device *dev,
                                    struct device_attribute *attr, const char *buf, size_t size )
	{
		u8 i;
		u8 dbbus_tx_data[4];
		unsigned char dbbus_rx_data[2] = {0};
	
	
			msg2133_reset();
			//1.Erase TP Flash first
		  dbbusDWIICEnterSerialDebugMode();
			dbbusDWIICStopMCU();
			dbbusDWIICIICUseBus();
			dbbusDWIICIICReshape();
			mdelay(300);
			
					
			// Disable the Watchdog
			dbbus_tx_data[0] = 0x10;
			dbbus_tx_data[1] = 0x3C;
			dbbus_tx_data[2] = 0x60;
			dbbus_tx_data[3] = 0x55;
			HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);
			dbbus_tx_data[0] = 0x10;
			dbbus_tx_data[1] = 0x3C;
			dbbus_tx_data[2] = 0x61;
			dbbus_tx_data[3] = 0xAA;
			HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);
	
		  //Stop MCU
			dbbus_tx_data[0] = 0x10;
			dbbus_tx_data[1] = 0x0F;
			dbbus_tx_data[2] = 0xE6;
			dbbus_tx_data[3] = 0x01;
			HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);
	
	
		  //set FRO to 50M
		dbbus_tx_data[0] = 0x10;
		dbbus_tx_data[1] = 0x11;
		dbbus_tx_data[2] = 0xE2;
		HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 3);
		dbbus_rx_data[0] = 0;
		dbbus_rx_data[1] = 0;
		HalTscrCReadI2CSeq(FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2);
   		 MSG2133_DBG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
		dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 3
		HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);
	
	
	
			//set MCU clock,SPI clock =FRO
			dbbus_tx_data[0] = 0x10;
			dbbus_tx_data[1] = 0x1E;
			dbbus_tx_data[2] = 0x22;
			dbbus_tx_data[2] = 0x00;
			HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);
			
			dbbus_tx_data[0] = 0x10;
			dbbus_tx_data[1] = 0x1E;
			dbbus_tx_data[2] = 0x23;
			dbbus_tx_data[2] = 0x00;
			HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);
	
			
		  // Enable slave's ISP ECO mode
			dbbus_tx_data[0] = 0x10;
			dbbus_tx_data[1] = 0x08;
			dbbus_tx_data[2] = 0x0c;
			dbbus_tx_data[3] = 0x08;
			HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);
	
		  //Enable SPI Pad
			dbbus_tx_data[0] = 0x10;
			dbbus_tx_data[1] = 0x1E;
			dbbus_tx_data[2] = 0x02;
			HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 3);
			HalTscrCReadI2CSeq(FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2);
			 MSG2133_DBG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
			dbbus_tx_data[3] = (dbbus_rx_data[0] | 0x20);  //Set Bit 5
			HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);
	
	
		  //WP overwrite
			dbbus_tx_data[0] = 0x10;
			dbbus_tx_data[1] = 0x1E;
			dbbus_tx_data[2] = 0x0E;
			dbbus_tx_data[3] = 0x02;
			HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);
	
	
		  //set pin high
			dbbus_tx_data[0] = 0x10;
			dbbus_tx_data[1] = 0x1E;
			dbbus_tx_data[2] = 0x10;
			dbbus_tx_data[3] = 0x08;
			HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);
	
			dbbusDWIICIICNotUseBus();
			dbbusDWIICNotStopMCU();
			dbbusDWIICExitSerialDebugMode();
			
			
			
		drvISP_EntryIspMode();
		drvISP_ChipErase();
		msg2133_reset();
		mdelay(300);
		
		//2.Program and Verify
		dbbusDWIICEnterSerialDebugMode();
			dbbusDWIICStopMCU();
			dbbusDWIICIICUseBus();
			dbbusDWIICIICReshape();
	
	
			
			// Disable the Watchdog
			dbbus_tx_data[0] = 0x10;
			dbbus_tx_data[1] = 0x3C;
			dbbus_tx_data[2] = 0x60;
			dbbus_tx_data[3] = 0x55;
			HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);
			dbbus_tx_data[0] = 0x10;
			dbbus_tx_data[1] = 0x3C;
			dbbus_tx_data[2] = 0x61;
			dbbus_tx_data[3] = 0xAA;
			HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);
	
		  //Stop MCU
			dbbus_tx_data[0] = 0x10;
			dbbus_tx_data[1] = 0x0F;
			dbbus_tx_data[2] = 0xE6;
			dbbus_tx_data[3] = 0x01;
			HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);
	
	
		  //set FRO to 50M
		dbbus_tx_data[0] = 0x10;
		dbbus_tx_data[1] = 0x11;
		dbbus_tx_data[2] = 0xE2;
		HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 3);
		dbbus_rx_data[0] = 0;
		dbbus_rx_data[1] = 0;
		HalTscrCReadI2CSeq(FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2);
		MSG2133_DBG("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);
		dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 3
		HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);
	
	
	
			//set MCU clock,SPI clock =FRO
			dbbus_tx_data[0] = 0x10;
			dbbus_tx_data[1] = 0x1E;
			dbbus_tx_data[2] = 0x22;
			dbbus_tx_data[2] = 0x00;
			HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);
			
			dbbus_tx_data[0] = 0x10;
			dbbus_tx_data[1] = 0x1E;
			dbbus_tx_data[2] = 0x23;
			dbbus_tx_data[2] = 0x00;
			HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);
	
		  // Enable slave's ISP ECO mode
			dbbus_tx_data[0] = 0x10;
			dbbus_tx_data[1] = 0x08;
			dbbus_tx_data[2] = 0x0c;
			dbbus_tx_data[3] = 0x08;
			HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);
			
		  //Enable SPI Pad
			dbbus_tx_data[0] = 0x10;
			dbbus_tx_data[1] = 0x1E;
			dbbus_tx_data[2] = 0x02;
			HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 3);
			HalTscrCReadI2CSeq(FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2);
			MSG2133_DBG("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);
			dbbus_tx_data[3] = (dbbus_rx_data[0] | 0x20);  //Set Bit 5
			HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);
	
	
		  //WP overwrite
			dbbus_tx_data[0] = 0x10;
			dbbus_tx_data[1] = 0x1E;
			dbbus_tx_data[2] = 0x0E;
			dbbus_tx_data[3] = 0x02;
			HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);
	
	
		  //set pin high
			dbbus_tx_data[0] = 0x10;
			dbbus_tx_data[1] = 0x1E;
			dbbus_tx_data[2] = 0x10;
			dbbus_tx_data[3] = 0x08;
			HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);
	
			dbbusDWIICIICNotUseBus();
			dbbusDWIICNotStopMCU();
			dbbusDWIICExitSerialDebugMode();
	
		///////////////////////////////////////
		// Start to load firmware
		///////////////////////////////////////
		drvISP_EntryIspMode();
	
		for (i = 0; i < 94; i++)   // total  94 KB : 1 byte per R/W
		{
			drvISP_Program(i, temp[i]);    // program to slave's flash
			//drvISP_Verify ( i, temp[i] ); //verify data
		}
			MSG2133_DBG("update OK\n");
		drvISP_ExitIspMode();
		msg2133_reset();
        FwDataCnt = 0;
    	enable_irq(this_client->irq);
		return size;
	}


static u32 Reflect ( u32 ref, char ch ) //unsigned int Reflect(unsigned int ref, char ch)
{
    u32 value = 0;
    u32 i = 0;

    for ( i = 1; i < ( ch + 1 ); i++ )
    {
        if ( ref & 1 )
        {
            value |= 1 << ( ch - i );
        }
        ref >>= 1;
    }
    return value;
}

u32 Get_CRC ( u32 text, u32 prevCRC, u32 *crc32_table )
{
    u32  ulCRC = prevCRC;
	ulCRC = ( ulCRC >> 8 ) ^ crc32_table[ ( ulCRC & 0xFF ) ^ text];
    return ulCRC ;
}
static void Init_CRC32_Table ( u32 *crc32_table )
{
    u32 magicnumber = 0x04c11db7;
    u32 i = 0, j;

    for ( i = 0; i <= 0xFF; i++ )
    {
        crc32_table[i] = Reflect ( i, 8 ) << 24;
        for ( j = 0; j < 8; j++ )
        {
            crc32_table[i] = ( crc32_table[i] << 1 ) ^ ( crc32_table[i] & ( 0x80000000L ) ? magicnumber : 0 );
        }
        crc32_table[i] = Reflect ( crc32_table[i], 32 );
    }
}

typedef enum
{
	EMEM_ALL = 0,
	EMEM_MAIN,
	EMEM_INFO,
} EMEM_TYPE_t;

static void drvDB_WriteReg8Bit ( u8 bank, u8 addr, u8 data )
{
    u8 tx_data[4] = {0x10, bank, addr, data};
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, tx_data, 4 );
}

static void drvDB_WriteReg ( u8 bank, u8 addr, u16 data )
{
    u8 tx_data[5] = {0x10, bank, addr, data & 0xFF, data >> 8};
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, tx_data, 5 );
}

static unsigned short drvDB_ReadReg ( u8 bank, u8 addr )
{
    u8 tx_data[3] = {0x10, bank, addr};
    u8 rx_data[2] = {0};

    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, tx_data, 3 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &rx_data[0], 2 );
    return ( rx_data[1] << 8 | rx_data[0] );
}

static int drvTP_erase_emem_c32 ( void )
{
    /////////////////////////
    //Erase  all
    /////////////////////////
    
    //enter gpio mode
    drvDB_WriteReg ( 0x16, 0x1E, 0xBEAF );

    // before gpio mode, set the control pin as the orginal status
    drvDB_WriteReg ( 0x16, 0x08, 0x0000 );
    drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x10 );
    mdelay ( 10 ); //MCR_CLBK_DEBUG_DELAY ( 10, MCU_LOOP_DELAY_COUNT_MS );

    // ptrim = 1, h'04[2]
    drvDB_WriteReg8Bit ( 0x16, 0x08, 0x04 );
    drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x10 );
    mdelay ( 10 ); //MCR_CLBK_DEBUG_DELAY ( 10, MCU_LOOP_DELAY_COUNT_MS );

    // ptm = 6, h'04[12:14] = b'110
    drvDB_WriteReg8Bit ( 0x16, 0x09, 0x60 );
    drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x10 );

    // pmasi = 1, h'04[6]
    drvDB_WriteReg8Bit ( 0x16, 0x08, 0x44 );
    // pce = 1, h'04[11]
    drvDB_WriteReg8Bit ( 0x16, 0x09, 0x68 );
    // perase = 1, h'04[7]
    drvDB_WriteReg8Bit ( 0x16, 0x08, 0xC4 );
    // pnvstr = 1, h'04[5]
    drvDB_WriteReg8Bit ( 0x16, 0x08, 0xE4 );
    // pwe = 1, h'04[9]
    drvDB_WriteReg8Bit ( 0x16, 0x09, 0x6A );
    // trigger gpio load
    drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x10 );

    return ( 1 );
}

static ssize_t firmware_update_c32 ( struct device *dev, struct device_attribute *attr,
                                     const char *buf, size_t size,  EMEM_TYPE_t emem_type )
{
    // u8 dbbus_tx_data[4];
    // u8 dbbus_rx_data[2] = {0};
    // Buffer for slave's firmware

    u32 i, j;
    u32 crc_main, crc_main_tp;
    u32 crc_info, crc_info_tp;
    u16 reg_data = 0;

    crc_main = 0xffffffff;
    crc_info = 0xffffffff;

#if 1
    /////////////////////////
    // Erase  all
    /////////////////////////
    drvTP_erase_emem_c32();
    mdelay ( 1000 ); //MCR_CLBK_DEBUG_DELAY ( 1000, MCU_LOOP_DELAY_COUNT_MS );

    //ResetSlave();
    msg2133_reset();
    //drvDB_EnterDBBUS();
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 300 );

    // Reset Watchdog
    drvDB_WriteReg8Bit ( 0x3C, 0x60, 0x55 );
    drvDB_WriteReg8Bit ( 0x3C, 0x61, 0xAA );

    /////////////////////////
    // Program
    /////////////////////////

    //polling 0x3CE4 is 0x1C70
    do
    {
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while ( reg_data != 0x1C70 );


    drvDB_WriteReg ( 0x3C, 0xE4, 0xE38F );  // for all-blocks

    //polling 0x3CE4 is 0x2F43
    do
    {
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while ( reg_data != 0x2F43 );


    //calculate CRC 32
    Init_CRC32_Table ( &crc_tab[0] );

    for ( i = 0; i < 33; i++ ) // total  33 KB : 2 byte per R/W
    {
        if ( i < 32 )   //emem_main
        {
            if ( i == 31 )
            {
                temp[i][1014] = 0x5A; //Fmr_Loader[1014]=0x5A;
                temp[i][1015] = 0xA5; //Fmr_Loader[1015]=0xA5;

                for ( j = 0; j < 1016; j++ )
                {
                    //crc_main=Get_CRC(Fmr_Loader[j],crc_main,&crc_tab[0]);
                    crc_main = Get_CRC ( temp[i][j], crc_main, &crc_tab[0] );
                }
            }
            else
            {
                for ( j = 0; j < 1024; j++ )
                {
                    //crc_main=Get_CRC(Fmr_Loader[j],crc_main,&crc_tab[0]);
                    crc_main = Get_CRC ( temp[i][j], crc_main, &crc_tab[0] );
                }
            }
        }
        else  // emem_info
        {
            for ( j = 0; j < 1024; j++ )
            {
                //crc_info=Get_CRC(Fmr_Loader[j],crc_info,&crc_tab[0]);
                crc_info = Get_CRC ( temp[i][j], crc_info, &crc_tab[0] );
            }
        }

        //drvDWIIC_MasterTransmit( DWIIC_MODE_DWIIC_ID, 1024, Fmr_Loader );
        HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP, temp[i], 1024 );

        // polling 0x3CE4 is 0xD0BC
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0xD0BC );

        drvDB_WriteReg ( 0x3C, 0xE4, 0x2F43 );
    }

    //write file done
    drvDB_WriteReg ( 0x3C, 0xE4, 0x1380 );

    mdelay ( 10 ); //MCR_CLBK_DEBUG_DELAY ( 10, MCU_LOOP_DELAY_COUNT_MS );
    // polling 0x3CE4 is 0x9432
    do
    {
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while ( reg_data != 0x9432 );

    crc_main = crc_main ^ 0xffffffff;
    crc_info = crc_info ^ 0xffffffff;

    // CRC Main from TP
    crc_main_tp = drvDB_ReadReg ( 0x3C, 0x80 );
    crc_main_tp = ( crc_main_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0x82 );
 
    //CRC Info from TP
    crc_info_tp = drvDB_ReadReg ( 0x3C, 0xA0 );
    crc_info_tp = ( crc_info_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0xA2 );

   // MSG2133_DBG ( "crc_main=0x%x, crc_info=0x%x, crc_main_tp=0x%x, crc_info_tp=0x%x\n",
       //        crc_main, crc_info, crc_main_tp, crc_info_tp );

    //drvDB_ExitDBBUS();
    if ( ( crc_main_tp != crc_main ) || ( crc_info_tp != crc_info ) )
    {
        MSG2133_DBG ( "update FAILED\n" );
		msg2133_reset();
        FwDataCnt = 0;
    	enable_irq(this_client->irq);		
        return ( 0 );
    }

    MSG2133_DBG ( "update OK\n" );
	msg2133_reset();
    FwDataCnt = 0;
	enable_irq(this_client->irq);

    return size;
#endif
}

static int drvTP_erase_emem_c33 ( EMEM_TYPE_t emem_type )
{
    // stop mcu
    drvDB_WriteReg ( 0x0F, 0xE6, 0x0001 );

    //disable watch dog
    drvDB_WriteReg8Bit ( 0x3C, 0x60, 0x55 );
    drvDB_WriteReg8Bit ( 0x3C, 0x61, 0xAA );

    // set PROGRAM password
    drvDB_WriteReg8Bit ( 0x16, 0x1A, 0xBA );
    drvDB_WriteReg8Bit ( 0x16, 0x1B, 0xAB );

    //proto.MstarWriteReg(F1.loopDevice, 0x1618, 0x80);
    drvDB_WriteReg8Bit ( 0x16, 0x18, 0x80 );

    if ( emem_type == EMEM_ALL )
    {
        drvDB_WriteReg8Bit ( 0x16, 0x08, 0x10 ); //mark
    }

    drvDB_WriteReg8Bit ( 0x16, 0x18, 0x40 );
    mdelay ( 10 );

    drvDB_WriteReg8Bit ( 0x16, 0x18, 0x80 );

    // erase trigger
    if ( emem_type == EMEM_MAIN )
    {
        drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x04 ); //erase main
    }
    else
    {
        drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x08 ); //erase all block
    }

    return ( 1 );
}

#if 0
static int drvTP_read_emem_dbbus_c33 ( EMEM_TYPE_t emem_type, u16 addr, size_t size, u8 *p, size_t set_pce_high )
{
    u32 i;

    // Set the starting address ( must before enabling burst mode and enter riu mode )
    drvDB_WriteReg ( 0x16, 0x00, addr );

    // Enable the burst mode ( must before enter riu mode )
    drvDB_WriteReg ( 0x16, 0x0C, drvDB_ReadReg ( 0x16, 0x0C ) | 0x0001 );

    // Set the RIU password
    drvDB_WriteReg ( 0x16, 0x1A, 0xABBA );

    // Enable the information block if pifren is HIGH
    if ( emem_type == EMEM_INFO )
    {
        // Clear the PCE
        drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0080 );
        mdelay ( 10 );

        // Set the PIFREN to be HIGH
        drvDB_WriteReg ( 0x16, 0x08, 0x0010 );
    }

    // Set the PCE to be HIGH
    drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0040 );
    mdelay ( 10 );

    // Wait pce becomes 1 ( read data ready )
    while ( ( drvDB_ReadReg ( 0x16, 0x10 ) & 0x0004 ) != 0x0004 );

    for ( i = 0; i < size; i += 4 )
    {
        // Fire the FASTREAD command
        drvDB_WriteReg ( 0x16, 0x0E, drvDB_ReadReg ( 0x16, 0x0E ) | 0x0001 );

        // Wait the operation is done
        while ( ( drvDB_ReadReg ( 0x16, 0x10 ) & 0x0001 ) != 0x0001 );

        p[i + 0] = drvDB_ReadReg ( 0x16, 0x04 ) & 0xFF;
        p[i + 1] = ( drvDB_ReadReg ( 0x16, 0x04 ) >> 8 ) & 0xFF;
        p[i + 2] = drvDB_ReadReg ( 0x16, 0x06 ) & 0xFF;
        p[i + 3] = ( drvDB_ReadReg ( 0x16, 0x06 ) >> 8 ) & 0xFF;
    }

    // Disable the burst mode
    drvDB_WriteReg ( 0x16, 0x0C, drvDB_ReadReg ( 0x16, 0x0C ) & ( ~0x0001 ) );

    // Clear the starting address
    drvDB_WriteReg ( 0x16, 0x00, 0x0000 );

    //Always return to main block
    if ( emem_type == EMEM_INFO )
    {
        // Clear the PCE before change block
        drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0080 );
        mdelay ( 10 );
        // Set the PIFREN to be LOW
        drvDB_WriteReg ( 0x16, 0x08, drvDB_ReadReg ( 0x16, 0x08 ) & ( ~0x0010 ) );

        drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0040 );
        while ( ( drvDB_ReadReg ( 0x16, 0x10 ) & 0x0004 ) != 0x0004 );
    }

    // Clear the RIU password
    drvDB_WriteReg ( 0x16, 0x1A, 0x0000 );

    if ( set_pce_high )
    {
        // Set the PCE to be HIGH before jumping back to e-flash codes
        drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0040 );
        while ( ( drvDB_ReadReg ( 0x16, 0x10 ) & 0x0004 ) != 0x0004 );
    }

    return ( 1 );
}
#endif


static int drvTP_read_info_dwiic_c33 ( void )
{
    u8 dwiic_tx_data[5];
    // u8 dwiic_rx_data[4];
    u16 reg_data = 0;

    mdelay(300);

    // Stop Watchdog
    drvDB_WriteReg8Bit ( 0x3C, 0x60, 0x55 );
    drvDB_WriteReg8Bit ( 0x3C, 0x61, 0xAA );

    drvDB_WriteReg ( 0x3C, 0xE4, 0xA4AB );

	drvDB_WriteReg ( 0x1E, 0x04, 0x7d60 );

    // TP SW reset
    drvDB_WriteReg ( 0x1E, 0x04, 0x829F );
	mdelay ( 1 );
    dwiic_tx_data[0] = 0x10;
    dwiic_tx_data[1] = 0x0F;
    dwiic_tx_data[2] = 0xE6;
    dwiic_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dwiic_tx_data, 4 );	
    mdelay ( 100 );

    do{
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while ( reg_data != 0x5B58 );

    dwiic_tx_data[0] = 0x72;
    dwiic_tx_data[1] = 0x80;
    dwiic_tx_data[2] = 0x00;
    dwiic_tx_data[3] = 0x04;
    dwiic_tx_data[4] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP , dwiic_tx_data, 5 );

    mdelay ( 50 );

    // recive info data
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX_TP, &g_dwiic_info_data[0], 1024 );

    return ( 1 );
}

#if 0
static int drvTP_info_updata_C33 ( u16 start_index, u8 *data, u16 size )
{
    // size != 0, start_index+size !> 1024
    u16 i;
    for ( i = 0; i < size; i++ )
    {
        g_dwiic_info_data[start_index] = * ( data + i );
        start_index++;
    }
    return ( 1 );
}
#endif

static ssize_t firmware_update_c33 ( struct device *dev, struct device_attribute *attr,
                                     const char *buf, size_t size, EMEM_TYPE_t emem_type )
{
    // u8  dbbus_tx_data[4];
    // u8  dbbus_rx_data[2] = {0};
    u8  life_counter[2];
    u32 i, j;
    u32 crc_main, crc_main_tp;
    u32 crc_info, crc_info_tp;
  
    int update_pass = 1;
    u16 reg_data = 0;

    crc_main = 0xffffffff;
    crc_info = 0xffffffff;

    drvTP_read_info_dwiic_c33();
	
    if ( g_dwiic_info_data[0] == 'M' && g_dwiic_info_data[1] == 'S' && g_dwiic_info_data[2] == 'T' && g_dwiic_info_data[3] == 'A' && g_dwiic_info_data[4] == 'R' && g_dwiic_info_data[5] == 'T' && g_dwiic_info_data[6] == 'P' && g_dwiic_info_data[7] == 'C' )
    {
        // updata FW Version
        //drvTP_info_updata_C33 ( 8, &temp[32][8], 5 );

		g_dwiic_info_data[8]=temp[32][8];
		g_dwiic_info_data[9]=temp[32][9];
		g_dwiic_info_data[10]=temp[32][10];
		g_dwiic_info_data[11]=temp[32][11];
        // updata life counter
        life_counter[1] = (( ( (g_dwiic_info_data[13] << 8 ) | g_dwiic_info_data[12]) + 1 ) >> 8 ) & 0xFF;
        life_counter[0] = ( ( (g_dwiic_info_data[13] << 8 ) | g_dwiic_info_data[12]) + 1 ) & 0xFF;
		g_dwiic_info_data[12]=life_counter[0];
		g_dwiic_info_data[13]=life_counter[1];
        //drvTP_info_updata_C33 ( 10, &life_counter[0], 3 );
        drvDB_WriteReg ( 0x3C, 0xE4, 0x78C5 );
		drvDB_WriteReg ( 0x1E, 0x04, 0x7d60 );
        // TP SW reset
        drvDB_WriteReg ( 0x1E, 0x04, 0x829F );

        mdelay ( 50 );

        //polling 0x3CE4 is 0x2F43
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );

        }
        while ( reg_data != 0x2F43 );

        // transmit lk info data
        HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP , &g_dwiic_info_data[0], 1024 );

        //polling 0x3CE4 is 0xD0BC
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0xD0BC );

    }

    //erase main
    drvTP_erase_emem_c33 ( EMEM_MAIN );
    mdelay ( 1000 );

    do {
            //ResetSlave();
            msg2133_reset();

            //drvDB_EnterDBBUS();
            dbbusDWIICEnterSerialDebugMode();
            dbbusDWIICStopMCU();
            dbbusDWIICIICUseBus();
            dbbusDWIICIICReshape();
            mdelay ( 300 );

            /////////////////////////
            // Program
            /////////////////////////

            //polling 0x3CE4 is 0x1C70
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }while( reg_data != 0x1C70);

    switch ( emem_type )
    {
        case EMEM_ALL:
            drvDB_WriteReg ( 0x3C, 0xE4, 0xE38F );  // for all-blocks
            break;
        case EMEM_MAIN:
            drvDB_WriteReg ( 0x3C, 0xE4, 0x7731 );  // for main block
            break;
        case EMEM_INFO:
            drvDB_WriteReg ( 0x3C, 0xE4, 0x7731 );  // for info block

            drvDB_WriteReg8Bit ( 0x0F, 0xE6, 0x01 );

            drvDB_WriteReg8Bit ( 0x3C, 0xE4, 0xC5 ); //
            drvDB_WriteReg8Bit ( 0x3C, 0xE5, 0x78 ); //

            drvDB_WriteReg8Bit ( 0x1E, 0x04, 0x9F );
            drvDB_WriteReg8Bit ( 0x1E, 0x05, 0x82 );

            drvDB_WriteReg8Bit ( 0x0F, 0xE6, 0x00 );
            mdelay ( 100 );
            break;
    }

    // polling 0x3CE4 is 0x2F43
    do
    {
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while ( reg_data != 0x2F43 );

    // calculate CRC 32
    Init_CRC32_Table ( &crc_tab[0] );

    for ( i = 0; i < 33; i++ ) // total  33 KB : 2 byte per R/W
    {
        if ( emem_type == EMEM_INFO )
			i = 32;

        if ( i < 32 )   //emem_main
        {
            if ( i == 31 )
            {
                temp[i][1014] = 0x5A; //Fmr_Loader[1014]=0x5A;
                temp[i][1015] = 0xA5; //Fmr_Loader[1015]=0xA5;

                for ( j = 0; j < 1016; j++ )
                {
                    //crc_main=Get_CRC(Fmr_Loader[j],crc_main,&crc_tab[0]);
                    crc_main = Get_CRC ( temp[i][j], crc_main, &crc_tab[0] );
                }
            }
            else
            {
                for ( j = 0; j < 1024; j++ )
                {
                    //crc_main=Get_CRC(Fmr_Loader[j],crc_main,&crc_tab[0]);
                    crc_main = Get_CRC ( temp[i][j], crc_main, &crc_tab[0] );
                }
            }
        }
        else  //emem_info
        {
            for ( j = 0; j < 1024; j++ )
            {
                //crc_info=Get_CRC(Fmr_Loader[j],crc_info,&crc_tab[0]);
                crc_info = Get_CRC ( g_dwiic_info_data[j], crc_info, &crc_tab[0] );
            }
            if ( emem_type == EMEM_MAIN ) break;
        }

        //drvDWIIC_MasterTransmit( DWIIC_MODE_DWIIC_ID, 1024, Fmr_Loader );
        HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP, temp[i], 1024 );

        // polling 0x3CE4 is 0xD0BC
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0xD0BC );

        drvDB_WriteReg ( 0x3C, 0xE4, 0x2F43 );
    }

    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        // write file done and check crc
        drvDB_WriteReg ( 0x3C, 0xE4, 0x1380 );
    }
    mdelay ( 10 ); //MCR_CLBK_DEBUG_DELAY ( 10, MCU_LOOP_DELAY_COUNT_MS );

    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        // polling 0x3CE4 is 0x9432
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }while ( reg_data != 0x9432 );
    }

    crc_main = crc_main ^ 0xffffffff;
    crc_info = crc_info ^ 0xffffffff;

    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        // CRC Main from TP
        crc_main_tp = drvDB_ReadReg ( 0x3C, 0x80 );
        crc_main_tp = ( crc_main_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0x82 );

        // CRC Info from TP
        crc_info_tp = drvDB_ReadReg ( 0x3C, 0xA0 );
        crc_info_tp = ( crc_info_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0xA2 );
    }
    //MSG2133_DBG ( "crc_main=0x%x, crc_info=0x%x, crc_main_tp=0x%x, crc_info_tp=0x%x\n",
    //           crc_main, crc_info, crc_main_tp, crc_info_tp );

    //drvDB_ExitDBBUS();

    update_pass = 1;
	if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        if ( crc_main_tp != crc_main )
            update_pass = 0;

        if ( crc_info_tp != crc_info )
            update_pass = 0;
    }

    if ( !update_pass )
    {
        MSG2133_DBG ( "update FAILED\n" );
		msg2133_reset();
        FwDataCnt = 0;
    	enable_irq(this_client->irq);
        return ( 0 );
    }

    MSG2133_DBG ( "update OK\n" );
	msg2133_reset();
    FwDataCnt = 0;
    enable_irq(this_client->irq);

    if(temp != NULL)
    {
        for(i = 0 ;i < 33 ; i ++)
        {
            free_page((u32)temp[i]);
        }
        kfree(temp);
        temp = NULL;
    }
    return size;
}

#define _FW_UPDATE_C3_
#ifdef _FW_UPDATE_C3_
static ssize_t firmware_update_store ( struct device *dev,
                                       struct device_attribute *attr, const char *buf, size_t size )
{
    // u8 i;
    u8 dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};
	disable_irq(this_client->irq);

    msg2133_reset();

    // Erase TP Flash first
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 300 );

    // Disable the Watchdog
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    // Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    /////////////////////////
    // Difference between C2 and C3
    /////////////////////////
	// c2:2133 c32:2133a(2) c33:2138
    //check id
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0xCC;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    MSG2133_DBG ( "dbbus_rx version[0]=0x%x", dbbus_rx_data[0] );
    if ( dbbus_rx_data[0] == 2 )
    {
        // check version
        dbbus_tx_data[0] = 0x10;
        dbbus_tx_data[1] = 0x3C;
        dbbus_tx_data[2] = 0xEA;
        HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
        HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
        //MSG2133_DBG ( "dbbus_rx version[0]=0x%x", dbbus_rx_data[0] );

        if ( dbbus_rx_data[0] == 3 ){
            return firmware_update_c33 ( dev, attr, buf, size, EMEM_MAIN );
		}
        else{

            return firmware_update_c32 ( dev, attr, buf, size, EMEM_ALL );
        }
    }
    else
    {
        return firmware_update_c2 ( dev, attr, buf, size );
    } 
}
#else
static ssize_t firmware_update_store ( struct device *dev,
                                       struct device_attribute *attr, const char *buf, size_t size )
{
    u8 i;
    u8 dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};

    msg2133_reset();

    // 1. Erase TP Flash first
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 300 );

    // Disable the Watchdog
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set FRO to 50M
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
   // MSG2133_DBG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 3
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set MCU clock,SPI clock =FRO
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x22;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x23;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Enable slave's ISP ECO mode
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x08;
    dbbus_tx_data[2] = 0x0c;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Enable SPI Pad
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
   // MSG2133_DBG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = ( dbbus_rx_data[0] | 0x20 ); //Set Bit 5
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // WP overwrite
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x0E;
    dbbus_tx_data[3] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set pin high
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x10;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();

    drvISP_EntryIspMode();
    drvISP_ChipErase();
    msg2133_reset();
    mdelay ( 300 );

    // 2.Program and Verify
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();

    // Disable the Watchdog
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set FRO to 50M
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
   // MSG2133_DBG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 3
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set MCU clock,SPI clock =FRO
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x22;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x23;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Enable slave's ISP ECO mode
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x08;
    dbbus_tx_data[2] = 0x0c;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Enable SPI Pad
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
 //   MSG2133_DBG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = ( dbbus_rx_data[0] | 0x20 ); //Set Bit 5
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // WP overwrite
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x0E;
    dbbus_tx_data[3] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set pin high
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x10;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();

    ///////////////////////////////////////
    // Start to load firmware
    ///////////////////////////////////////
    drvISP_EntryIspMode();

    for ( i = 0; i < 94; i++ ) // total  94 KB : 1 byte per R/W
    {
        drvISP_Program ( i, temp[i] ); // program to slave's flash
        drvISP_Verify ( i, temp[i] ); //verify data
    }
 //   MSG2133_DBG ( "update OK\n" );
    drvISP_ExitIspMode();
    FwDataCnt = 0;
    
    return size;
}
#endif
static DEVICE_ATTR(update, 0777, firmware_update_show, firmware_update_store);
#if 0
/*test=================*/
static ssize_t firmware_clear_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
	MSG2133_DBG(" +++++++ [%s] Enter!++++++\n", __func__);
	u16 k=0,i = 0, j = 0;
	u8 bWriteData[5] =
	{
        0x10, 0x03, 0, 0, 0
	};
	u8 RX_data[256];
	u8 bWriteData1 = 0x12;
	u32 addr = 0;
	u32 timeOutCount=0;
	for (k = 0; k < 94; i++)   // total  94 KB : 1 byte per R/W
	{
		addr = k * 1024;
		for (j = 0; j < 8; j++)   //128*8 cycle
		{
			bWriteData[2] = (u8)((addr + j * 128) >> 16);
			bWriteData[3] = (u8)((addr + j * 128) >> 8);
			bWriteData[4] = (u8)(addr + j * 128);
			//msctpc_LoopDelay ( 1 );        // delay about 100us*****
			udelay(150);//200);

			timeOutCount=0;
			while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
			{
				timeOutCount++;
				if ( timeOutCount >= 100000 ) 
					break; /* around 1 sec timeout */
	  		}
        
			HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 5);    //write read flash addr
			//msctpc_LoopDelay ( 1 );        // delay about 100us*****
			udelay(150);//200);
			drvISP_Read(128, RX_data);
			HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);    //cmd end
			for (i = 0; i < 128; i++)   //log out if verify error
			{
				if (RX_data[i] != 0xFF)
				{
					//MSG2133_DBG(MSG2133_DBG("k=%d,j=%d,i=%d===============erase not clean================",k,j,i);)
					MSG2133_DBG("k=%d,j=%d,i=%d  erase not clean !!",k,j,i);
				}
			}
		}
	}
	MSG2133_DBG("read finish\n");
	return sprintf(buf, "%s\n", fw_version);
}

static ssize_t firmware_clear_store(struct device *dev,
                                     struct device_attribute *attr, const char *buf, size_t size)
{

	u8 dbbus_tx_data[4];
	unsigned char dbbus_rx_data[2] = {0};
	MSG2133_DBG(" +++++++ [%s] Enter!++++++\n", __func__);
	//msctpc_LoopDelay ( 100 ); 	   // delay about 100ms*****

	// Enable slave's ISP ECO mode

	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x08;
	dbbus_tx_data[2] = 0x0c;
	dbbus_tx_data[3] = 0x08;
	
	// Disable the Watchdog
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x11;
	dbbus_tx_data[2] = 0xE2;
	dbbus_tx_data[3] = 0x00;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x3C;
	dbbus_tx_data[2] = 0x60;
	dbbus_tx_data[3] = 0x55;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x3C;
	dbbus_tx_data[2] = 0x61;
	dbbus_tx_data[3] = 0xAA;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

	//Stop MCU
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x0F;
	dbbus_tx_data[2] = 0xE6;
	dbbus_tx_data[3] = 0x01;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

	//Enable SPI Pad
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x1E;
	dbbus_tx_data[2] = 0x02;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 3);
	HalTscrCReadI2CSeq(FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2);
	MSG2133_DBG(MSG2133_DBG("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);)
	dbbus_tx_data[3] = (dbbus_rx_data[0] | 0x20);  //Set Bit 5
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x1E;
	dbbus_tx_data[2] = 0x25;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 3);

	dbbus_rx_data[0] = 0;
	dbbus_rx_data[1] = 0;
	HalTscrCReadI2CSeq(FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2);
	MSG2133_DBG(MSG2133_DBG("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);)
	dbbus_tx_data[3] = dbbus_rx_data[0] & 0xFC;  //Clear Bit 1,0
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

	//WP overwrite
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x1E;
	dbbus_tx_data[2] = 0x0E;
	dbbus_tx_data[3] = 0x02;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);


	//set pin high
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x1E;
	dbbus_tx_data[2] = 0x10;
	dbbus_tx_data[3] = 0x08;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);
	//set FRO to 50M
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x11;
	dbbus_tx_data[2] = 0xE2;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 3);
	dbbus_rx_data[0] = 0;
	dbbus_rx_data[1] = 0;
	HalTscrCReadI2CSeq(FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2);
	MSG2133_DBG(MSG2133_DBG("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);)
	dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 1,0
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

	dbbusDWIICIICNotUseBus();
	dbbusDWIICNotStopMCU();
	dbbusDWIICExitSerialDebugMode();

    ///////////////////////////////////////
    // Start to load firmware
    ///////////////////////////////////////
    drvISP_EntryIspMode();
	MSG2133_DBG(MSG2133_DBG("chip erase+\n");)
    drvISP_BlockErase(0x00000);
	MSG2133_DBG(MSG2133_DBG("chip erase-\n");)
    drvISP_ExitIspMode();
    return size;
}
static DEVICE_ATTR(clear, 0777, firmware_clear_show, firmware_clear_store);
#endif //0
/*test=================*/
/*Add by Tracy.Lin for update touch panel firmware and get fw version*/

static ssize_t firmware_version_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%03d%03d\n", fw_v.major,fw_v.minor);
}

static ssize_t firmware_version_store(struct device *dev,
                                      struct device_attribute *attr, const char *buf, size_t size)
{
    unsigned char dbbus_tx_data[3];
    unsigned char dbbus_rx_data[4] ;
    unsigned short major=0, minor=0;
/*
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();

*/
    //fw_version = kzalloc(sizeof(char), GFP_KERNEL);

    //Get_Chip_Version();
    dbbus_tx_data[0] = 0x53;
    dbbus_tx_data[1] = 0x00;
    dbbus_tx_data[2] = 0x74;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX_TP, &dbbus_tx_data[0], 3);
    HalTscrCReadI2CSeq(FW_ADDR_MSG21XX_TP, &dbbus_rx_data[0], 4);

    fw_v.major = (dbbus_rx_data[1]<<8)+dbbus_rx_data[0];
    fw_v.minor = (dbbus_rx_data[3]<<8)+dbbus_rx_data[2];


    MSG2133_DBG("****firmware version is %03d%03d****\n",fw_v.major,fw_v.minor);

    return size;
}
static DEVICE_ATTR(version, 0777, firmware_version_show, firmware_version_store);

static ssize_t firmware_data_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
	return FwDataCnt;
}

static ssize_t firmware_data_store(struct device *dev,
                                   struct device_attribute *attr, const char *buf, size_t size)
{

	int i;

	if(temp == NULL)
	{
	       temp = kmalloc(94 * sizeof(u8 *),GFP_KERNEL);
	       for(i = 0 ; i < 94 ; i ++)
	       {
		    temp[i] = (u8 *) get_zeroed_page(GFP_KERNEL);
	       }
	}

	    //MSG2133_DBG("***FwDataCnt = %d ***\n", FwDataCnt);
	for (i = 0; i < 1024; i++)
	{
	    memcpy(temp[FwDataCnt], buf, 1024);
	}
	FwDataCnt++;
	return size;

}

static DEVICE_ATTR(data, 0777, firmware_data_show, firmware_data_store);

#endif  //__FIRMWARE_UPDATE__
static void msg2133_reset(void)
{
	MSG2133_DBG("%s(). line: %d.\n", __FUNCTION__, __LINE__);

	gpio_direction_output(sprd_3rdparty_gpio_tp_rst, 0);
	msleep(60);//20
	gpio_direction_output(sprd_3rdparty_gpio_tp_rst, 1);
	msleep(300);
}

static void msg2133_device_power_on(void)
{
	MSG2133_DBG("tyd-tp: %s()\n", __FUNCTION__);

	reg_vdd = regulator_get(NULL, "vdd28");
	regulator_set_voltage(reg_vdd, 2800000, 2800000);
	regulator_enable(reg_vdd);
    	msleep(10); //wait for stable
}

static void msg2133_device_power_off(void)
{
	MSG2133_DBG("tyd-tp: %s()\n", __FUNCTION__);

    // LDO_TurnOffLDO(LDO_LDO_SIM2);
}


#ifdef TP_PROXIMITY_SENSOR
static int TP_face_get_mode(void)
{
	MSG2133_DBG(">>>> PROXIMITY_SWITCH is %d <<<<\n", PROXIMITY_SWITCH);

	return PROXIMITY_SWITCH;
}

static int TP_face_mode_state(void)
{
	MSG2133_DBG(">>>> PROXIMITY_STATE  is %d <<<<\n", PROXIMITY_STATE);

	return PROXIMITY_STATE;
}

static int TP_face_mode_switch(int on)
{
	int proximity_switch;
	u8 data[4];

	data[0] = 0x52;
	data[1] = 0x00;//0x01
	data[2] = 0x4A;//0x24
	data[3] = 0xA0; //0x52, 0x01, 0x24, 0xA0
	proximity_switch = TP_face_get_mode();

	MSG2133_DBG(">>>> begin, proximity_switch is 0x%.2X, on is 0x%.2X, INCALLING is 0x%.2X. <<<<\n\n",PROXIMITY_SWITCH, on, INCALLING);

	spin_lock(&proximity_switch_lock);

	if ((1 == on) && (0 == proximity_switch|| 0 == INCALLING))
	{
		data[3] = 0xA0;
		PROXIMITY_SWITCH = 1;
		INCALLING        = 1;

		goto OUT;
    	}
	else if ((0 == on) && (1 == proximity_switch|| 1 == INCALLING))
	{
		data[3] = 0xA1;
		PROXIMITY_SWITCH = 0;
		INCALLING	 = 0;

		goto OUT;
	}
	else
	{
		spin_unlock(&proximity_switch_lock);
		MSG2133_DBG(">>>> On is %d and proximity_switch is %d, this is invalid! <<<<\n", on, proximity_switch);

		return -EINVAL;
	}


OUT:
	spin_unlock(&proximity_switch_lock);

	MSG2133_DBG(">>>> %s(%d), PROXIMITY_SWITCH is %d, PROXIMITY_STATE is %d <<<<\n", __FUNCTION__, on, PROXIMITY_SWITCH, PROXIMITY_STATE);

	return ((this_client) ? (i2c_master_send(this_client, &data[0], 4)) : (-ENODEV));
}

static ssize_t tp_face_mode_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int proximity_switch, proximity_state;

	proximity_switch = TP_face_get_mode();
	proximity_state  = TP_face_mode_state();

	return sprintf(buf,
		__stringify(%d) ":" __stringify(%d) "\n", proximity_switch, proximity_state);
}

static ssize_t tp_face_mode_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	char proximity_buf[PROXIMITY_BUF_MAX_SIZE + 1];

	// memcpy(&proximity_buf, buf, sizeof(proximity_buf) / sizeof(proximity_buf[0]));
	memcpy(&proximity_buf, buf, 1);

	switch (proximity_buf[0])
	{
		case '0':
		    TP_face_mode_switch(0);
		    break;

		case '1':
		    TP_face_mode_switch(1);
		    break;

		default:
		    return -EINVAL;
	}

	return 1;
}

static struct kobj_attribute tp_face_mode_attr =
{
	.attr =
	{
		.name = "facemode",     // Path: /sys/board_properties/facemode;
		.mode = S_IRUGO | S_IWUGO,
	},
	.show  = &tp_face_mode_show, // For read the proximity state by upper level;
	.store = &tp_face_mode_store, // For open/close the proximity_switch switch by upper level;
};
#endif

#if VIRTUAL_KEYS
static ssize_t virtual_keys_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,
		__stringify(EV_KEY) ":" __stringify(KEY_MENU) ":40:%d:50:50"
		":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE) ":120:%d:50:50"
		":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":200:%d:50:50"
		"\n", PRINT_KEY_Y, PRINT_KEY_Y, PRINT_KEY_Y);
}

static struct kobj_attribute virtual_keys_attr = {
	.attr = {
	    .name = "virtualkeys.focaltech_ts", //virtualkeys.msg2133
	    .mode = S_IRUGO,
	},
	.show = &virtual_keys_show
};

static struct attribute *properties_attrs[] = {
	&virtual_keys_attr.attr,
    #ifdef TP_PROXIMITY_SENSOR
    &tp_face_mode_attr.attr,
    #endif
	NULL
};

static struct attribute_group properties_attr_group = {
	.attrs = properties_attrs,
};

static struct kobject *properties_kobj = NULL;

static void virtual_keys_init(void)
{

	int ret;

	MSG2133_DBG("%s()\n", __FUNCTION__);

	properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (properties_kobj)
	{
	    ret = sysfs_create_group(properties_kobj, &properties_attr_group);
	}

	if (!properties_kobj || ret)
	{
	    pr_err("failed to create board_properties\n");
	}

}
#endif

static unsigned char msg2133_check_sum(unsigned char *pval)
{

	int i, sum = 0;

	for(i = 0; i < 7; i++)
	{
	    sum += pval[i];
	}

	return (unsigned char)((-sum) & 0xFF);

}


static int msg2133_read_data(struct i2c_client *client)
{

	int ret, keycode;
	u8 reg_val[8] = {0};
	int dst_x = 0, dst_y = 0; // xysawp_temp=0;
	u32 temp_checksum;
	struct TouchScreenInfo_t touchData;

	struct msg2133_ts_data *data  = i2c_get_clientdata(this_client);
	struct ts_event        *event = &data->event;

	event->touch_point = 0;
	ret = i2c_master_recv(client, reg_val, 8);

	MSG2133_DBG("%s: ret = %d of i2c_master_recv().\n", __FUNCTION__, ret);
	if (ret <= 0)
	{
		ret = -EINVAL;
		goto OUT;
	}

	event->y1 = ((reg_val[1] & 0xF0) << 4) | reg_val[2];
	event->x1 = ((reg_val[1] & 0x0F) << 8) | reg_val[3];
	dst_y     = ((reg_val[4] & 0xF0) << 4) | reg_val[5];
	dst_x     = ((reg_val[4] & 0x0F) << 8) | reg_val[6];
	//SWAP_XY(event->x1,event->y1);
	//SWAP_XY(dst_x,dst_y);
	temp_checksum = msg2133_check_sum(reg_val);


#ifdef TP_PROXIMITY_SENSOR
    if (PROXIMITY_SWITCH == 1)
    {
            if (reg_val[5] == 0x80)
            {
                if(!!PROXIMITY_STATE)
                {
                    MSG2133_DBG("%s() Line:%d Tp already to near\n", __FUNCTION__, __LINE__);
                }
                else
                {
                    MSG2133_DBG("%s() Line:%d Tp is to near\n", __FUNCTION__, __LINE__);

                    spin_lock(&proximity_state_lock);
                    PROXIMITY_STATE = 1;
                    spin_unlock(&proximity_state_lock);

		            input_report_abs(data->input_dev, ABS_DISTANCE, !PROXIMITY_STATE);
                    input_sync(data->input_dev);
                }

                ret = 1;
                goto OUT;
            }
            else if (reg_val[5] == 0x40)
            {
                if(!PROXIMITY_STATE)
                {
                    MSG2133_DBG("%s() Line:%d Tp already to far\n",__FUNCTION__,__LINE__);
                }
                else
                {
                    MSG2133_DBG("%s() Line:%d Tp is to far\n",__FUNCTION__,__LINE__);

                    spin_lock(&proximity_state_lock);
                    PROXIMITY_STATE = 0;
                    spin_unlock(&proximity_state_lock);

		            input_report_abs(data->input_dev, ABS_DISTANCE, !PROXIMITY_STATE);
                    input_sync(data->input_dev);
                }

                ret = 1;
                goto OUT;
            }
            else
            {
                // Fixme, when in proximity feature is opened, it couldn't goto this way;
            }
    }
#endif


	if ((temp_checksum != reg_val[7]) || (reg_val[0] != 0x52))
	{
		ret = -EINVAL;
		goto OUT;
	}
	else
	{
		if ((reg_val[1] == 0xFF) && (reg_val[2] == 0xFF) && (reg_val[3] == 0xFF) && (reg_val[4] == 0xFF) && (reg_val[6] == 0xFF))
		{
			event->x1 = 0; // final X coordinate
			event->y1 = 0; // final Y coordinate

			if ((reg_val[5] == 0x0) || (reg_val[5] == 0xFF))
			{
				event->touch_point      = 0; //touch end
				touchData.nTouchKeyCode = 0; //TouchKeyMode
				touchData.nTouchKeyMode = 0; //TouchKeyMode
				keycode                 = 0;
			}
			else
			{
				touchData.nTouchKeyMode = 1;          //TouchKeyMode
				touchData.nTouchKeyCode = reg_val[5]; //TouchKeyCode
				keycode                 = reg_val[5];

				// Virtual key;
				switch (keycode)
				{
					case 1:
                        			event->x1 = 40;
                        			break;

                    			case 2:
                        			event->x1 = 120;
                        			break;

                    			case 4:
                        			event->x1 = 200;
                        			break;

                    			default:
                        		// Fixme;
                        		event->x1 = 220;
                        		break;
                		}

				event->y1 = PRINT_KEY_Y;
				event->touch_point  = 1;
				MSG2133_DBG("*** %s, key x,y:%d,%d\n", __func__,event->x1,event->y1);
			}
		} // Endif ((reg_val[1] == 0xFF) && (reg_val[2] == 0xFF) && (reg_val[3] == 0xFF) && (reg_val[4] == 0xFF) && (reg_val[6] == 0xFF));
		else
		{
			touchData.nTouchKeyMode = 0; //Touch on screen...

			if ((dst_x == 0) && (dst_y == 0))
            		{
				event->touch_point = 1; //one touch

				event->x1 = TS_WIDTH_MAX - (event->x1 * TS_WIDTH_MAX) / 2048;
				event->y1 = (event->y1 * TS_HEIGHT_MAX) / 2048;
				MSG2133_DBG("*** %s, x,y:%d,%d\n", __func__,event->x1,event->y1);
			}
            		else
            		{
				event->touch_point = 2; //two touch

                //transform the unsigh value to sign value
				if (dst_x > 2048)
                		{
					dst_x -= 4096;
				}

				if (dst_y > 2048)
                		{
					dst_y -= 4096;
				}

				event->x2 = (event->x1 + dst_x);
				event->y2 = (event->y1 + dst_y);

				event->x1 = TS_WIDTH_MAX - (event->x1 * TS_WIDTH_MAX) / 2048;
				event->y1 = (event->y1 * TS_HEIGHT_MAX) / 2048;

				event->x2 = TS_WIDTH_MAX - (event->x2 * TS_WIDTH_MAX) / 2048;
				event->y2 = (event->y2 * TS_HEIGHT_MAX) / 2048;
				MSG2133_DBG("*** %s, [x1,y1:%d,%d] [x2,y2:%d,%d]\n", __func__,event->x1,event->y1,event->x2,event->y2);
			}
		}
	} // End else if((temp_checksum == reg_val[7]) && (reg_val[0] == 0x52));

	ret = 0;
	goto OUT;

OUT:
	return ret;
}

static void msg2133_report_value(struct i2c_client *client)
{
	struct msg2133_ts_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event       = &data->event;

	// MSG2133_DBG("%s, point = %d.\n", __FUNCTION__, event->touch_point);

	if (event->touch_point)
    	{
		input_report_key(data->input_dev, BTN_TOUCH, 1);

		switch (event->touch_point)
        	{
			case 2:
				// MSG2133_DBG("%s, x2 = %d, y2 = %d\n", __FUNCTION__, event->x2, event->y2);
				input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 15);
				input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x2);
				input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y2);
				input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
				input_mt_sync(data->input_dev);

			case 1:
				// MSG2133_DBG("%s, x1 = %d, y1 = %d\n", __FUNCTION__, event->x1, event->y1);
				input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 15);
				input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x1);
				input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y1);
				input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
				input_mt_sync(data->input_dev);
			default:
				input_report_key(data->input_dev, BTN_TOUCH, 1);
                // MSG2133_DBG("==touch_point default =\n");
				break;
		} // End switch(event->touch_point)
	} // Endif if (event->touch_point);
	else
	{
		input_report_key(data->input_dev, BTN_TOUCH, 0);
		input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 0);
	}

	input_sync(data->input_dev);

}	/*end msg2133_report_value*/


static void msg2133_ts_pen_irq_work(struct work_struct *work)
{

	if (0 == msg2133_read_data(this_client))
    	{
		msg2133_report_value(this_client);
	}

	enable_irq(this_client->irq);

}

static irqreturn_t msg2133_ts_interrupt(int irq, void *dev_id)
{

	struct msg2133_ts_data *msg2133_ts = (struct msg2133_ts_data *)dev_id;

	// MSG2133_DBG("%s\n", __FUNCTION__);

	disable_irq_nosync(this_client->irq);

	if (0 == work_pending(&msg2133_ts->pen_event_work))
    	{
		queue_work(msg2133_ts->ts_workqueue, &msg2133_ts->pen_event_work);
	}

	return IRQ_HANDLED;

}

static unsigned char suspend_flags = 0;

static void msg2133_ts_suspend(struct early_suspend *handler)
{
	// MSG2133_DBG("== %s() ==\n", __FUNCTION__);
	#ifdef TP_PROXIMITY_SENSOR
	if (!!PROXIMITY_SWITCH || !!INCALLING)
	{
	    MSG2133_DBG("MSG2133's proximity already open,suspend do nothing!\n");
	    suspend_flags = 0;
	}
	else
	{
    	#endif
		disable_irq_nosync(this_client->irq);
		msleep(3);

		gpio_direction_output(sprd_3rdparty_gpio_tp_rst, 1);
		gpio_set_value(sprd_3rdparty_gpio_tp_rst, 0);
		msleep(10);
		suspend_flags = 1;
	#ifdef TP_PROXIMITY_SENSOR
	}
	#endif
}

static void msg2133_ts_resume(struct early_suspend *handler)
{
	// MSG2133_DBG("== %s() == start ==\n", __FUNCTION__);
	u8 data[4] = {0x52,0x00,0x4A,0xA0};
	#ifdef TP_PROXIMITY_SENSOR
	if ((!!PROXIMITY_SWITCH || !!INCALLING) && (suspend_flags == 0))
	{
	}
	else
	{
	#endif

		gpio_set_value(sprd_3rdparty_gpio_tp_rst, 0);
		msg2133_device_power_on();
		msleep(20);

		gpio_set_value(sprd_3rdparty_gpio_tp_rst,1);
		msleep(100);
		if(!!PROXIMITY_SWITCH)
		{
			i2c_master_send(this_client,&data[0],4);//enable proximity function
		}
		enable_irq(this_client->irq);

	#ifdef TP_PROXIMITY_SENSOR
	}
	#endif
}

static int msg2133_ts_config_pins(void)
{

	int msg2133_irq;

	MSG2133_DBG("== %s() ==\n", __FUNCTION__);

	reg_vdd = regulator_get(NULL, "vdd28");
	regulator_set_voltage(reg_vdd, 2800000, 2800000);
	regulator_enable(reg_vdd);
    	msleep(10); //wait for stable
		
	gpio_direction_output(sprd_3rdparty_gpio_tp_rst, 1);
	gpio_direction_input(sprd_3rdparty_gpio_tp_irq);
	gpio_set_value(sprd_3rdparty_gpio_tp_rst, 1);

	msg2133_irq = gpio_to_irq(sprd_3rdparty_gpio_tp_irq);
	if (msg2133_irq < 0)
	{
		MSG2133_DBG(">>>> %s(), sprd_alloc_gpio_irq(%d) is failed! <<<<\n", __FUNCTION__, sprd_3rdparty_gpio_tp_irq);
    	}

	msleep(100); //wait for stable

	return msg2133_irq;

}

#ifdef GST_TP_COMPATIBLE
static int msg2133_read_id(void)
{

	int err = -1;
	u8 dbbus_tx_data[3];
	u8 dbbus_rx_data[4] = {0};

	dbbus_tx_data[0] = 0x53;
	dbbus_tx_data[1] = 0x00;
	dbbus_tx_data[2] = 0x74;

	err = i2c_master_send(this_client, &dbbus_tx_data[0], sizeof(dbbus_tx_data) / sizeof(dbbus_tx_data[0]));
	if (err < 0)
	{
		MSG2133_DBG("%s() Line: %d err: %d.\n", __FUNCTION__, __LINE__, err);
		return err;
	}

	msleep(200);

	err = i2c_master_recv(this_client, &dbbus_rx_data[0], sizeof(dbbus_rx_data) / sizeof(dbbus_rx_data[0]));
	if (err < 0)
	{
		MSG2133_DBG("%s() Line: %d err is %d",__FUNCTION__, __LINE__, err);
		return err;
	}

	err = dbbus_rx_data[1];

	fw_v.major = (dbbus_rx_data[1]<<8)+dbbus_rx_data[0];
	fw_v.minor = (dbbus_rx_data[3]<<8)+dbbus_rx_data[2];

	MSG2133_DBG(">>>> %s() id is 0x%.2x, firmware version is %03d%03d.\n", __FUNCTION__, err, fw_v.major,fw_v.minor);

	return err;

}
#endif

static int msg2133_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct msg2133a_ts_platform_data *pdata = client->dev.platform_data;
	struct msg2133_ts_data *msg2133_ts;
	struct input_dev *input_dev;
	int err = 0;

	MSG2133_DBG("%s()\n", __FUNCTION__);
	sprd_3rdparty_gpio_tp_rst = pdata->reset_gpio_number;
	sprd_3rdparty_gpio_tp_irq = pdata->irq_gpio_number;
	this_client = client;
	client->irq = msg2133_ts_config_pins();

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		err = -ENODEV;
		MSG2133_DBG("%s() Line: %d, i2c_check_functionality(), err = %d.\n", __FUNCTION__, __LINE__, err);

		goto exit_check_functionality_failed;
	}

#ifdef GST_TP_COMPATIBLE
	//msg2133_reset();

	err = msg2133_read_id();
	if( err != 0 )
	{
		MSG2133_DBG(">>>> %s(), The TP ic is not MSG2133A! %s() is auto terminal. <<<<\n", __FUNCTION__, __FUNCTION__);
	        goto exit_read_id_failed;
	}
#endif

	msg2133_ts = kzalloc(sizeof(*msg2133_ts), GFP_KERNEL);
	if (!msg2133_ts)
    	{
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	i2c_set_clientdata(client, msg2133_ts);

	MSG2133_DBG(">>>> %s(), Line: %d, MSG2133A I2C ADDR = 0x%x. <<<<\n", __FUNCTION__, __LINE__, client->addr);


	INIT_WORK(&msg2133_ts->pen_event_work, msg2133_ts_pen_irq_work);
	msg2133_ts->ts_workqueue = create_singlethread_workqueue(dev_name(&client->dev));
	if (!msg2133_ts->ts_workqueue)
   	{
		err = -ESRCH;
		goto exit_create_singlethread;
	}

	MSG2133_DBG("%s(): request_irq() %s IRQ number is %d", __FUNCTION__, client->name, client->irq);

	//err = request_irq(client->irq, msg2133_ts_interrupt, IRQF_TRIGGER_FALLING, client->name, msg2133_ts);
	request_threaded_irq(client->irq,NULL,msg2133_ts_interrupt, IRQF_TRIGGER_FALLING |IRQF_ONESHOT, client->name, msg2133_ts);
	if (err < 0)
   	{
        	MSG2133_DBG("%s() Line: %d, request irq failed, err = %d.\n", __FUNCTION__, __LINE__, err);

		goto exit_irq_request_failed;
	}

	disable_irq(client->irq);

	// MSG2133_DBG("==input_allocate_device=\n");
	input_dev = input_allocate_device();
	if (!input_dev)
    	{
		err = -ENOMEM;
       		MSG2133_DBG("%s() Line: %d, input_allocate_device failed, err = %d.\n", __FUNCTION__, __LINE__, err);

		goto exit_input_dev_alloc_failed;
	}
	msg2133_ts->input_dev = input_dev;

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_SYN, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);


	__set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	__set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	__set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	__set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);
#ifdef TP_PROXIMITY_SENSOR
	__set_bit(ABS_DISTANCE, input_dev->absbit);
#endif

#if VIRTUAL_KEYS
	__set_bit(KEY_MENU, input_dev->keybit);
	__set_bit(KEY_BACK, input_dev->keybit);
	__set_bit(KEY_HOMEPAGE, input_dev->keybit);
	__set_bit(KEY_SEARCH, input_dev->keybit);
#endif

	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, TS_WIDTH_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, TS_HEIGHT_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);

#ifdef TP_PROXIMITY_SENSOR
    input_set_abs_params(input_dev, ABS_DISTANCE, 0, 1, 0, 0);

    spin_lock_init(&proximity_switch_lock);
    spin_lock_init(&proximity_state_lock);
#endif	
	input_dev->name	= "focaltech_ts"; //MSG2133_TS_NAME

	err = input_register_device(input_dev);
	if (err)
    	{
		dev_err(&client->dev, "msg2133_ts_probe: failed to register input device: %s\n", dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}


	MSG2133_DBG("==%s(), register_early_suspend ==\n", __FUNCTION__);

	msg2133_ts->early_suspend.level   = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 3;
	msg2133_ts->early_suspend.suspend = msg2133_ts_suspend;
	msg2133_ts->early_suspend.resume  = msg2133_ts_resume;

	register_early_suspend(&msg2133_ts->early_suspend);

#ifdef __FIRMWARE_UPDATE__
	firmware_class = class_create(THIS_MODULE, "ms-touchscreen-msg20xx");
	if (IS_ERR(firmware_class))
	{
	    pr_err("Failed to create class(firmware)!\n");
	}

	firmware_cmd_dev = device_create(firmware_class, NULL, 0, NULL, "device");
	if (IS_ERR(firmware_cmd_dev))
	{
	    pr_err("Failed to create device(firmware_cmd_dev)!\n");
	}

	// version
	if (device_create_file(firmware_cmd_dev, &dev_attr_version) < 0)
	{
	    pr_err("Failed to create device file(%s)!\n", dev_attr_version.attr.name);
	}
	// update
	if (device_create_file(firmware_cmd_dev, &dev_attr_update) < 0)
	{
	    pr_err("Failed to create device file(%s)!\n", dev_attr_update.attr.name);
	}
	// data
	if (device_create_file(firmware_cmd_dev, &dev_attr_data) < 0)
	{
	    pr_err("Failed to create device file(%s)!\n", dev_attr_data.attr.name);
	}

#if 0
	// clear
	if (device_create_file(firmware_cmd_dev, &dev_attr_clear) < 0)
	{
	    pr_err("Failed to create device file(%s)!\n", dev_attr_clear.attr.name);
	}
#endif

	dev_set_drvdata(firmware_cmd_dev, NULL);

#endif //__FIRMWARE_UPDATE__


	msleep(50);
	//get some register information

#if VIRTUAL_KEYS
	virtual_keys_init();
#endif

	enable_irq(client->irq);

	err = 0;
	goto OUT;

	// It seems didn't be called;
	// unregister_early_suspend(&msg2133_ts->early_suspend);

exit_input_register_device_failed:
	input_free_device(input_dev);
	input_dev = NULL;

exit_input_dev_alloc_failed:
	free_irq(client->irq, msg2133_ts);

exit_irq_request_failed:
	cancel_work_sync(&msg2133_ts->pen_event_work);
	destroy_workqueue(msg2133_ts->ts_workqueue);

exit_create_singlethread:
	i2c_set_clientdata(client, NULL);

	kfree(msg2133_ts);
	msg2133_ts = NULL;

exit_alloc_data_failed:
exit_check_functionality_failed:
exit_read_id_failed:
	gpio_free(client->irq);


OUT:
	return err;
}

static int msg2133_ts_remove(struct i2c_client *client)
{

	struct msg2133_ts_data *msg2133_ts = i2c_get_clientdata(client);

	MSG2133_DBG("%s()\n", __FUNCTION__);

	if (msg2133_ts)
    	{
		#if VIRTUAL_KEYS
		if (properties_kobj)
		{
		    sysfs_remove_group(properties_kobj, &properties_attr_group);
		    kobject_del(properties_kobj);
		    properties_kobj = NULL;
		}
		#endif

		unregister_early_suspend(&msg2133_ts->early_suspend);

		input_unregister_device(msg2133_ts->input_dev);
		input_free_device(msg2133_ts->input_dev);
		msg2133_ts->input_dev = NULL;

		free_irq(client->irq, msg2133_ts);

		cancel_work_sync(&msg2133_ts->pen_event_work);
		destroy_workqueue(msg2133_ts->ts_workqueue);

		i2c_set_clientdata(client, NULL);

		kfree(msg2133_ts);
		msg2133_ts = NULL;

		gpio_free(client->irq);

		// Do nothing;
		msg2133_device_power_off();

	}

	return 0;

}


static const struct i2c_device_id msg2133_ts_id[] = {
	{ MSG2133_TS_NAME, 0 }, { }
};

MODULE_DEVICE_TABLE(i2c, msg2133_ts_id);

static struct i2c_driver msg2133_ts_driver = {
	.probe		= msg2133_ts_probe,
	.remove		= msg2133_ts_remove,
	.id_table	= msg2133_ts_id,
	.driver	= {
		.name	= "MSG2133A",
		.owner	= THIS_MODULE,
	},
};

static int __init msg2133_init_module(void)
{
	MSG2133_DBG("%s()\n", __FUNCTION__);
		
	return i2c_add_driver(&msg2133_ts_driver);
}

static void __exit msg2133_exit_module(void)
{
	MSG2133_DBG("%s()\n", __FUNCTION__);

	i2c_del_driver(&msg2133_ts_driver);

}

module_init(msg2133_init_module);
module_exit(msg2133_exit_module);

MODULE_LICENSE("GPL");
