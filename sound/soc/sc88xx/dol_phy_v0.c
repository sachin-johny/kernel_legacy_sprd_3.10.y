#include "audio_dev_phy.h"
#include "vbc-codec.h"

#define VBPMR2_RLGOD_SHIFT 4
#define VBPMR2_RLGOD_MASK  (0x3<<VBPMR2_RLGOD_SHIFT)

#define VBCGR8_RLGO_SHIFT 6
#define VBCGR8_RLGO_MASK    (0x3 << VBCGR8_RLGO_SHIFT)

#define VBCGR1_GODL_SHIFT 0
#define VBCGR1_GODL_MASK    (0xf << VBCGR1_GODL_SHIFT)

#define VBCGR8_GODL_SHIFT 0
#define VBCGR8_GODL_MASK    (0x1f << VBCGR8_GODL_SHIFT)

#define VBCGR1_GODR_SHIFT 4
#define VBCGR1_GODR_MASK    (0xf << VBCGR1_GODR_SHIFT)

#define VBCGR9_GODR_SHIFT 0
#define VBCGR9_GODR_MASK    (0x1f << VBCGR9_GODR_SHIFT)





PUBLIC void CODEC_PHY_SetDACPGA(uint32_t uichannel,CODEC_DAC_OUTPUT_PGA_T pga)
{
	vbc_reg_write(VBPMR2,0,0,VBPMR2_RLGOD_MASK);
	vbc_reg_write(VBCGR8,0,0,VBCGR8_RLGO_MASK);	
	if(0 == uichannel)
	{
		vbc_reg_write(VBCGR1,VBCGR1_GODL_SHIFT,pga.dac_pga_l,VBCGR1_GODL_MASK);
		vbc_reg_write(VBCGR8,VBCGR8_GODL_SHIFT,pga.hp_pga_l,VBCGR8_GODL_MASK);
	}
	else if(1 == uichannel)
	{
		vbc_reg_write(VBCGR1,VBCGR1_GODR_SHIFT,pga.dac_pga_r,VBCGR1_GODR_MASK);
		vbc_reg_write(VBCGR9,VBCGR9_GODR_SHIFT,pga.hp_pga_r,VBCGR9_GODR_MASK);
	}
	else
		{
			printk("CODEC_PHY_SetDACPGA error!\n");
		}
}
