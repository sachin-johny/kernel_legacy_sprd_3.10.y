#include <linux/miscdevice.h>
#include <linux/sysfs.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/ioctl.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/ioport.h>
#include <asm/io.h>

#include "trout_fm_ctrl.h"
#include "trout_fm_audio.h"


int phy_write(u32 addr, u32 val)
{
	u32 *p_vaddr;

	p_vaddr = (u32*)ioremap_nocache(addr, 4);
	if(p_vaddr == NULL)
	{
		TROUT_PRINT("io_remap failed!");
		return -1;
	}

	writel(val, p_vaddr);

	iounmap(p_vaddr);
	
	return 0;
}

int phy_read(u32 addr, u32 *val)
{
	u32 *p_vaddr;

	p_vaddr = (u32*)ioremap_nocache(addr, 4);
	if(p_vaddr == NULL)
	{
		TROUT_PRINT("io_remap failed!");
		return -1;
	}

	*val = readl(p_vaddr);

	iounmap(p_vaddr);
	
	return 0;
}


void i2s_pin_cfg(void)
{
	phy_write(0x8c000358, 0x106);
	phy_write(0x8c00035c, 0x10a);
	phy_write(0x8c000360, 0x10a);
	phy_write(0x8c000364, 0x10a);
	phy_write(0x8c000368, 0x106);
}

void i2s_gr_ctrl(void)
{
	u32 addr = 0x8B00000C;	//GR_PCTL
	u32 val;

	phy_read(addr, &val);
	phy_write(addr, (val & (~BIT_31)));
}

void i2s_gr_en(void)
{
	u32 addr = 0x8B000008;	//GR_GEN0
	u32 val;

	phy_read(addr, &val);
	phy_write(addr, (val | BIT_12));
}

int fm_audio_play(void)
{
	i2s_pin_cfg();
	i2s_gr_ctrl();
	i2s_gr_en();

/*
	i2s_phy_fun_tbl.init(i2s_id);
	i2s_phy_fun_tbl.open(i2s_id, &fm_i2s_setting, 0);

	i2s_phy_fun_tbl.ioctl(i2s_id, DMA_CTRL, SCI_TRUE);
	i2s_phy_fun_tbl.dma_read(i2s_id, (u32)fm_rx_fifo, &data_len, fm_i2s_dma_callback);
	
	vbc_from_iis_play(fm_rx_fifo, 640);	
	*/
	
	return 0;   
}

