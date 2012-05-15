#include "audio_dev_phy.h"
#include "vbc-codec.h"
#include "sc88xx-asoc.h"

#define VBPMR2_RLGOD_SHIFT 4
#define VBPMR2_RLGOD_MASK  (0x3)

#define VBCGR8_LRGO_SHIFT 6
#define VBCGR8_LRGO_MASK    (0x3)

#define VBCGR1_GODL_SHIFT 0
#define VBCGR1_GODL_MASK    (0xf)

#define VBCGR8_GOL_SHIFT 0
#define VBCGR8_GOL_MASK    (0x1f)

#define VBCGR1_GODR_SHIFT 4
#define VBCGR1_GODR_MASK    (0xf)

#define VBCGR9_GOR_SHIFT 0
#define VBCGR9_GOR_MASK    (0x1f)

PUBLIC void CODEC_PHY_SetDACPGA(uint32_t uichannel,CODEC_DAC_OUTPUT_PGA_T pga)
{
	vbc_reg_write(VBPMR2,0,0,VBPMR2_RLGOD_MASK);
	vbc_reg_write(VBCGR8,0,0,VBCGR8_LRGO_MASK);	
	if(0 == uichannel)   //left
	{
		vbc_reg_write(VBCGR1,VBCGR1_GODL_SHIFT,pga.dac_pga_l,VBCGR1_GODL_MASK);
		vbc_reg_write(VBCGR8,VBCGR8_GOL_SHIFT,pga.hp_pga_l,VBCGR8_GOL_MASK);
		
	}
	else if(1 == uichannel)
	{
		vbc_reg_write(VBCGR1,VBCGR1_GODR_SHIFT,pga.dac_pga_r,VBCGR1_GODR_MASK);
		vbc_reg_write(VBCGR9,VBCGR9_GOR_SHIFT,pga.hp_pga_r,VBCGR9_GOR_MASK);
	}
	else
		{
			printk("CODEC_PHY_SetDACPGA error!\n");
		}
}
