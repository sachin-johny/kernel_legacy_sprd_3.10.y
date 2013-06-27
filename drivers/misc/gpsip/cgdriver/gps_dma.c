/******************************************************************************
 *                                LEGAL NOTICE                                *
 *                                                                            *
 *  USE OF THIS SOFTWARE (including any copy or compiled version thereof) AND *
 *  DOCUMENTATION IS SUBJECT TO THE SOFTWARE LICENSE AND RESTRICTIONS AND THE *
 *  WARRANTY DISLCAIMER SET FORTH IN LEGAL_NOTICE.TXT FILE. IF YOU DO NOT     *
 *  FULLY ACCEPT THE TERMS, YOU MAY NOT INSTALL OR OTHERWISE USE THE SOFTWARE *
 *  OR DOCUMENTATION.                                                         *
 *  NOTWITHSTANDING ANYTHING TO THE CONTRARY IN THIS NOTICE, INSTALLING OR    *
 *  OTHERISE USING THE SOFTWARE OR DOCUMENTATION INDICATES YOUR ACCEPTANCE OF *
 *  THE LICENSE TERMS AS STATED.                                              *
 *                                                                            *
 ******************************************************************************/
/* Version: 1.8.9\3686 */
/* Build  : 13 */
/* Date   : 12/08/2012 */

/**
	\file
	\brief SPRD_FPGA DMA support

	Functions to access DMA unit in SPRD_FPGA CPU.
	\attention This file is not completed, and must be updated for SPRD_FPGA (Please verify all 'TODO' comments are handled)
*/

// ===========================================================================
#include "platform.h"
#include <linux/delay.h>
#include "CgCpu.h"
#include "platform.h"
#include "gps_dma.h"


#include <linux/kthread.h>
#include <linux/signal.h>
#include <mach/board.h>
#include <mach/hardware.h>

#include <mach/dma.h>
//#include <linux/sched.h>


// ===========================================================================

// extern unsigned long SPRD_FPGA_reserved_gps_virmem ;/*modify by paul for test compile*/
// extern unsigned long SPRD_FPGA_reserved_gps_phymem;/*modified by paul*/


unsigned long SPRD_FPGA_reserved_gps_virmem = 0x850000 ;/*modify by paul for test compile*/
unsigned long SPRD_FPGA_reserved_gps_phymem =0x86000;/*modified by paul*/
//static int g_nodecnt = 0;

/*it is normal function,added by paul for 8810*/
#define BURST_LEN 32 // should be equal to size of device FIFO
#define BUF_LEN 1024

#define INVALIDE_DMA_CHN (-1)

int cg_GpsDma_channel = INVALIDE_DMA_CHN;

static char *CgGps_Dev_name = "CgGps_Dev";

//bxd add for statistics how many block is be required from cellgudie.
int blocks_required;


extern void CCgCpuDmaHandle(int irq,void* dev_id);
static struct sci_dma_cfg dma_cfg;

// ===========================================================================

TCgReturnCode CgCpuDmaCreate(U32 aDmaChannel, U32 aIpDataSourceAddress)
{
	DBG_FUNC_NAME("CgCpuDmaCreate");
	DBGMSG("entry");


	cg_GpsDma_channel = sci_dma_request(CgGps_Dev_name, FULL_DMA_CHN);

	if(cg_GpsDma_channel < 0)
	{

		printk("CgCpuDmaCreate fail err code = %d\n",cg_GpsDma_channel);
		return cg_GpsDma_channel;

	}
	printk("CgCpuDmaCreate succ chn = %d\n",cg_GpsDma_channel);

	return ECgOk;

}


TCgReturnCode CgCpuDmaDestroy(U32 aDmaChannel, U32 aIpDataSourceAddress)
{
	int rc;
	printk("%s\n",__func__);

	rc = sci_dma_stop(cg_GpsDma_channel, DMA_GPS);
	if(rc < 0)
	{
		printk("CgCpuDmaDestroy stop chn fail, chn num = %d, err code = %d\n",cg_GpsDma_channel,rc);
	}

	rc = sci_dma_free(cg_GpsDma_channel);
	if(rc < 0)
	{
		printk("CgCpuDmaDestroy free chn fail, chn num = %d, err code = %d\n",cg_GpsDma_channel,rc);
	}


	cg_GpsDma_channel = INVALIDE_DMA_CHN;
	return ECgOk ;
}



TCgReturnCode CgCpuDmaIsReady(U32 aDmaChannel)
{
	TCgReturnCode rc = ECgOk;
	DBG_FUNC_NAME("CgCpuDmaIsReady");
	DBGMSG("entry");

	return rc;
}

TCgReturnCode CgCpuDmaStop(U32 aDmaChannel)
{
	int rc;
	printk("%s\n",__func__);
	rc = sci_dma_stop(cg_GpsDma_channel, DMA_GPS);
	if(rc < 0)
	{
		printk("CgCpuDmaStop stop chn fail, chn num = %d, err code = %d\n",cg_GpsDma_channel,rc);
	}

	return ECgOk;

}

TCgReturnCode CgCpuDmaCurCount(U32 aDmaChannel, U32 *apCount)
{
    TCgReturnCode rc = ECgOk;
	DBG_FUNC_NAME("CgCpuDmaCurCount");
	DBGMSG("entry");

	return rc;
}

// Get the DMA channel requested bytes count.
TCgReturnCode CgCpuDmaRequestedCount(U32 aDmaChannel, U32 *apCount)
{

	TCgReturnCode rc = ECgOk;
	DBG_FUNC_NAME("CgCpuDmaRequestedCount");
	DBGMSG("entry");

	return rc;
}


/*It is normal function which has been modified ...paul.luo add


*/
TCgReturnCode CgCpuDmaStart(U32 aDmaChannel)
{
	int rc;
	printk("%s\n",__func__);
	CgCpuCacheSync();
    rc = sci_dma_start(cg_GpsDma_channel, DMA_GPS);
	if(rc < 0)
	{
		printk("CgCpuDmaStart start chn fail, chn num = %d, err code = %d\n",cg_GpsDma_channel,rc);
	}
	//CgCpuCacheSync();

	return ECgOk;
}

int dma_int_count;

#if 1
u32 dma_block_count;
u32 block_size;
u32 dma_block_size;
u32 total_len;


// Configure DMA to read data from GPS device.
TCgReturnCode CgCpuDmaSetupFromGps(TCgCpuDmaTask *apDmaTask, U32 aDmaTaskCount, U32 aDmaChannel, U32 aDeviceAddress)
{

	//DBG_FUNC_NAME("CgCpuDmaSetupFromGps")
	int rc;
	u32 block_len = 0,total_size = 0;
	//int i = 0;

	if(aDmaTaskCount == 0)
	{
		return ECgBadArgument;
	}

	dma_block_count = 0;
	block_len = apDmaTask[0].length;
	total_size = block_len*(aDmaTaskCount-1)+apDmaTask[aDmaTaskCount-1].length;
	block_size = block_len;

	if(blocks_required <= 1)
	{
		if(block_len >= 128*1024)
			block_len = 128*1024-16;
	}
	else if(block_len >= 64*1024)
	{
		dma_block_count = block_len/(64*1024);
		block_len = 64*1024;
		//if(block_len%16)
		//	dma_block_count += 1;
	}
	total_len = total_size;
	dma_block_size = block_len;

	//for(i=0;i<aDmaTaskCount;i++)
	{
		//printk("rCgCpuDmaSetupFromGps:apDmaTask[%d].address 0x%x\n", i,(unsigned int)apDmaTask[i].address);
		//printk("rCgCpuDmaSetupFromGps:apDmaTask[0].length 0x%x\n",(unsigned int)apDmaTask[i].length);
		//printk("rCgCpuDmaSetupFromGps:aDmaTaskCount %d\n", (int)aDmaTaskCount);
	}
	printk("block_len:%d\n",(int)block_len);
	printk("total_size:%d\n",(int)total_size);
	printk("dma_block_count:%d\n",(int)dma_block_count);

	memset(&dma_cfg, 0x0, sizeof(dma_cfg));
	dma_cfg.datawidth = WORD_WIDTH;
	dma_cfg.src_addr = aDeviceAddress;
	dma_cfg.des_addr = apDmaTask[0].address;
	dma_cfg.src_step = 4;
	dma_cfg.des_step = 4;
	dma_cfg.wrap_ptr = 0x21c0007c;
	dma_cfg.wrap_to  = 0x21c00070;
	dma_cfg.fragmens_len = 16;
	dma_cfg.block_len = block_len;//apDmaTask[0].length;
	dma_cfg.transcation_len = total_size;//apDmaTask[0].length*aDmaTaskCount;
	dma_cfg.req_mode = FRAG_REQ_MODE;


	printk("read  dma_cfg.src_addr :0x%x\n",dma_cfg.src_addr);
	printk("read  dma_cfg.des_addr :0x%x\n",dma_cfg.des_addr);

	rc = sci_dma_config(cg_GpsDma_channel, &dma_cfg, 1, NULL);
	if(rc < 0)
	{
		printk("CgCpuDmaSetupFromGps cfg chn fail, chn num = %d, err code = %d\n",cg_GpsDma_channel,rc);
		return rc;
	}
	printk("blocks_required :%d\n",(int)blocks_required);
	if(blocks_required<=1)
	{
		sci_dma_register_irqhandle(cg_GpsDma_channel,TRANS_DONE,CCgCpuDmaHandle,NULL);
	}
	else
	{
		sci_dma_register_irqhandle(cg_GpsDma_channel,BLK_DONE,CCgCpuDmaHandle,NULL);
	}

	return ECgOk;
}
#endif


#if 0
// Configure DMA to read data from GPS device.
TCgReturnCode CgCpuDmaSetupFromGps(TCgCpuDmaTask *apDmaTask, U32 aDmaTaskCount, U32 aDmaChannel, U32 aDeviceAddress)
{

	//DBG_FUNC_NAME("CgCpuDmaSetupFromGps")
	int rc;
	u32 block_len = 0,total_size = 0;
	//int i = 0;

	if(aDmaTaskCount == 0)
	{
		return ECgBadArgument;
	}

	block_len = apDmaTask[0].length;
	total_size = block_len*(aDmaTaskCount-1)+apDmaTask[aDmaTaskCount-1].length;
	if(blocks_required <= 1)
	{
		if(block_len >= 128*1024)
			block_len = 128*1024-16;
		if(total_size >= 256*1024*1024)
			total_size = 256*1024*1024-1;
	}

	//for(i=0;i<aDmaTaskCount;i++)
	{
		//printk("rCgCpuDmaSetupFromGps:apDmaTask[%d].address 0x%x\n", i,apDmaTask[i].address);
		//printk("rCgCpuDmaSetupFromGps:apDmaTask[0].length 0x%x\n",apDmaTask[0].length);
		printk("rCgCpuDmaSetupFromGps:aDmaTaskCount %d\n", (int)aDmaTaskCount);
	}
	printk("block_len:%d\n",(int)block_len);
	printk("total_size:%d\n",(int)total_size);
	#if 1
	memset(&dma_cfg, 0x0, sizeof(dma_cfg));
	dma_cfg.datawidth = WORD_WIDTH;
	dma_cfg.src_addr = aDeviceAddress;
	dma_cfg.des_addr = apDmaTask[0].address;
	dma_cfg.src_step = 4;
	dma_cfg.des_step = 4;
	dma_cfg.wrap_ptr = 0x21c0007c;
	dma_cfg.wrap_to  = 0x21c00070;
	dma_cfg.fragmens_len = 16;
	dma_cfg.block_len = block_len;//apDmaTask[0].length;
	dma_cfg.transcation_len = total_size;//apDmaTask[0].length*aDmaTaskCount;
	dma_cfg.req_mode = FRAG_REQ_MODE;
	#endif

#if 0
	dma_cfg.datawidth = WORD_WIDTH;
	dma_cfg.src_addr = 0x21c00044;
	dma_cfg.des_addr = p_addr_dest;//apDmaTask[0].address;
	dma_cfg.src_step = 4;
	dma_cfg.des_step = 4;
	dma_cfg.wrap_ptr = 0x21c0004c;
	dma_cfg.wrap_to  = 0x21c00044;
	dma_cfg.fragmens_len = 4;
	dma_cfg.block_len = 4;
	dma_cfg.transcation_len = 24;//apDmaTask[0].length;
	dma_cfg.req_mode = TRANS_REQ_MODE;
#endif
	printk("read 111111111 dma_cfg.src_addr :0x%x\n",dma_cfg.src_addr);
	printk("read 111111111 dma_cfg.des_addr :0x%x\n",dma_cfg.des_addr);

	rc = sci_dma_config(cg_GpsDma_channel, &dma_cfg, 1, NULL);
	if(rc < 0)
	{
		printk("CgCpuDmaSetupFromGps cfg chn fail, chn num = %d, err code = %d\n",cg_GpsDma_channel,rc);
		return rc;
	}
	printk("blocks_required :%d\n",(int)blocks_required);
	if(blocks_required<=1)
	{
		sci_dma_register_irqhandle(cg_GpsDma_channel,TRANS_DONE,CCgCpuDmaHandle,NULL);
	}
	else
	{
		sci_dma_register_irqhandle(cg_GpsDma_channel,BLK_DONE,CCgCpuDmaHandle,NULL);
	}

	return ECgOk;
}
#endif
//bxd
#if 0
void SPRD_FPGA_dma_init(void)
{
    DBG_FUNC_NAME("SPRD_FPGA_dma_init");
    U32 reg_data = 0;
    U32 value = 0;

    return;
}


// configure the dma channel register (cx_config)
int SPRD_FPGA_dma_config_channel_reg(U32 *value)
{
	DBG_FUNC_NAME("SPRD_FPGA_dma_config_channel_reg");


    return 0;
}


// clear dma interrupt register
int SPRD_FPGA_dma_clear_intr(void)
{
	DBG_FUNC_NAME("SPRD_FPGA_dma_clear_intr");
    return 0;
}


/* enable/disable DMA channel */
int SPRD_FPGA_dma_channel_enable(U32 flag)
{
	DBG_FUNC_NAME("SPRD_FPGA_dma_channel_enable");
    U32 reg_data =0;

    return 0;
}


int SPRD_FPGA_dma_read_reg(void)
{
    DBG_FUNC_NAME("SPRD_FPGA_dma_read_reg");
    U32 reg_data = 0;

    return  0;
}



int SPRD_FPGA_dma_read_reg_timeout(void)
{
    DBG_FUNC_NAME("SPRD_FPGA_dma_read_reg_timeout");
    U32 reg_data = 0;

    return  0;
}



/* enable dmac clock */
int DmacClockEnable(void)
{
        DBG_FUNC_NAME("DmacClockEnable");

            return  0;

}

/* disable dmac clock */
int DmacClockDisable(void)
{
	DBG_FUNC_NAME("DmacClockDisable");


    return  0;
}

void SPRD_FPGA_dma_isr(void)
{
	DBG_FUNC_NAME("SPRD_FPGA_dma_isr");
    U32 i  = 0;
    return;
}
#endif
#ifdef TRACE_ON
void DbgStatDMA(const char *aTitle)
{
	//DBG_FUNC_NAME("DbgStatDMA");
	//DBGMSG1("%s", aTitle);


}
#endif


