/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __ASM_ARCH_AUDIO_GLB_SC8830_H
#define __ASM_ARCH_AUDIO_GLB_SC8830_H

#ifndef __ASM_ARCH_AUDIO_GLB_H
#error  "Don't include this file directly, include <mach/sprd-audio.h>"
#endif

#include <linux/delay.h>
#include <mach/hardware.h>
#include <mach/sci.h>
#include <mach/adi.h>
#include <mach/dma.h>
#include <mach/irqs.h>
#include <mach/regs_sc8830_aon_apb.h>
#include <mach/regs_sc8830_ap_apb.h>
#include <mach/regs_sc8830_ana_glb.h>

/* OKAY, this is for other else owner
   if you do not care the audio config
   you can set FIXED_AUDIO  to 0
   for compile happy.
*/
/* FIXME */
#define FIXED_AUDIO 1

enum {
	AUDIO_NO_CHANGE,
	AUDIO_TO_CP0_DSP_CTRL,
	AUDIO_TO_CP1_DSP_CTRL,
	AUDIO_TO_AP_ARM_CTRL,
	AUDIO_TO_CP0_ARM_CTRL,
	AUDIO_TO_CP1_ARM_CTRL,
	AUDIO_TO_CP2_ARM_CTRL,
};

#if FIXED_AUDIO
#define VBC_BASE		SPRD_VBC_BASE
#define CODEC_DP_BASE 		SPRD_AUDIO_BASE
#define CODEC_AP_BASE		(SPRD_ADI_BASE + 0x8600)
#define VBC_PHY_BASE		SPRD_VBC_PHYS
#define CODEC_DP_PHY_BASE	SPRD_AUDIO_PHYS
#define CODEC_AP_PHY_BASE	(SPRD_ADI_PHYS + 0x8600)
#define CODEC_AP_IRQ		(IRQ_ANA_INT)
#define CODEC_DP_IRQ		(IRQ_REQ_AUD_INT)

#ifdef CONFIG_SPRD_AUDIO_BUFFER_USE_IRAM
#define SPRD_IRAM_ALL_PHYS	0X00000000
#define SPRD_IRAM_ALL_SIZE	SZ_32K
#endif

#define CLASS_G_LDO_ID			"vddamp"
#endif

/* ------------------------------------------------------------------------- */

/* NOTE: all function maybe will call by atomic funtion
         don NOT any complex oprations. Just register.
return
   0:  	unchanged
   1:	changed
   ohter error
*/
/* vbc setting */

static inline int arch_audio_vbc_reg_enable(void)
{
	int ret = 0;

#if FIXED_AUDIO
	sci_glb_set(REG_AON_APB_APB_EB0, BIT_VBC_EB);
#endif

	return ret;
}

static inline int arch_audio_vbc_reg_disable(void)
{
	int ret = 0;

#if FIXED_AUDIO
	sci_glb_clr(REG_AON_APB_APB_EB0, BIT_VBC_EB);
#endif

	return ret;
}

static inline int arch_audio_vbc_enable(void)
{
	int ret = 0;

#if FIXED_AUDIO
	//sci_glb_set(REG_GLB_BUSCLK, BIT_ARM_VBC_ANAON);
#endif

	return ret;
}

static inline int arch_audio_vbc_disable(void)
{
	int ret = 0;

#if FIXED_AUDIO
	//sci_glb_clr(REG_GLB_BUSCLK, BIT_ARM_VBC_ANAON);
#endif

	return ret;
}

static inline int arch_audio_vbc_switch(int master)
{
	int ret = 0;
#if FIXED_AUDIO
	int val = 0;
	int mask =
	    BITS_VBC_AFIFO_INT_SYS_SEL(3) | BITS_VBC_DA01_INT_SYS_SEL(3) |
	    BITS_VBC_AD01_INT_SYS_SEL(3)
	    | BITS_VBC_AD23_INT_SYS_SEL(3) | BITS_VBC_DA01_DMA_SYS_SEL(3) |
	    BITS_VBC_AD01_DMA_SYS_SEL(3)
	    | BITS_VBC_AD23_DMA_SYS_SEL(3);

	switch (master) {
	case AUDIO_TO_AP_ARM_CTRL:
		val =
		    BITS_VBC_AFIFO_INT_SYS_SEL(0) | BITS_VBC_DA01_INT_SYS_SEL(0)
		    | BITS_VBC_AD01_INT_SYS_SEL(0)
		    | BITS_VBC_AD23_INT_SYS_SEL(0) |
		    BITS_VBC_DA01_DMA_SYS_SEL(0) | BITS_VBC_AD01_DMA_SYS_SEL(0)
		    | BITS_VBC_AD23_DMA_SYS_SEL(0);
		sci_glb_write(REG_AON_APB_VBC_CTRL, val, mask);
		break;
	case AUDIO_TO_CP0_DSP_CTRL:
		val =
		    BITS_VBC_AFIFO_INT_SYS_SEL(1) | BITS_VBC_DA01_INT_SYS_SEL(1)
		    | BITS_VBC_AD01_INT_SYS_SEL(1)
		    | BITS_VBC_AD23_INT_SYS_SEL(1) |
		    BITS_VBC_DA01_DMA_SYS_SEL(1) | BITS_VBC_AD01_DMA_SYS_SEL(1)
		    | BITS_VBC_AD23_DMA_SYS_SEL(1);
		sci_glb_write(REG_AON_APB_VBC_CTRL, val, mask);
		sci_glb_write(REG_AON_APB_VBC_CTRL, 0,
			(BIT_VBC_DMA_CP0_ARM_SEL |
			       BIT_VBC_DMA_CP0_ARM_SEL));
		break;
	case AUDIO_TO_CP1_DSP_CTRL:
		val =
		    BITS_VBC_AFIFO_INT_SYS_SEL(2) | BITS_VBC_DA01_INT_SYS_SEL(2)
		    | BITS_VBC_AD01_INT_SYS_SEL(2)
		    | BITS_VBC_AD23_INT_SYS_SEL(2) |
		    BITS_VBC_DA01_DMA_SYS_SEL(2) | BITS_VBC_AD01_DMA_SYS_SEL(2)
		    | BITS_VBC_AD23_DMA_SYS_SEL(2);
		sci_glb_write(REG_AON_APB_VBC_CTRL, val, mask);
		sci_glb_write(REG_AON_APB_VBC_CTRL, 0,
			      (BIT_VBC_DMA_CP1_ARM_SEL |
			       BIT_VBC_DMA_CP1_ARM_SEL));
		break;
	case AUDIO_TO_CP0_ARM_CTRL:
		val =
		    BITS_VBC_AFIFO_INT_SYS_SEL(1) | BITS_VBC_DA01_INT_SYS_SEL(1)
		    | BITS_VBC_AD01_INT_SYS_SEL(1)
		    | BITS_VBC_AD23_INT_SYS_SEL(1) |
		    BITS_VBC_DA01_DMA_SYS_SEL(1) | BITS_VBC_AD01_DMA_SYS_SEL(1)
		    | BITS_VBC_AD23_DMA_SYS_SEL(1);
		sci_glb_write(REG_AON_APB_VBC_CTRL,
			      (val | BIT_VBC_INT_CP0_ARM_SEL |
			       BIT_VBC_DMA_CP0_ARM_SEL),
			      (mask | BIT_VBC_INT_CP0_ARM_SEL |
			       BIT_VBC_DMA_CP0_ARM_SEL));
		break;
	case AUDIO_TO_CP1_ARM_CTRL:
		val =
		    BITS_VBC_AFIFO_INT_SYS_SEL(2) | BITS_VBC_DA01_INT_SYS_SEL(2)
		    | BITS_VBC_AD01_INT_SYS_SEL(2)
		    | BITS_VBC_AD23_INT_SYS_SEL(2) |
		    BITS_VBC_DA01_DMA_SYS_SEL(2) | BITS_VBC_AD01_DMA_SYS_SEL(2)
		    | BITS_VBC_AD23_DMA_SYS_SEL(2);
		sci_glb_write(REG_AON_APB_VBC_CTRL,
			      (val | BIT_VBC_INT_CP1_ARM_SEL |
			       BIT_VBC_DMA_CP1_ARM_SEL),
			      (mask | BIT_VBC_INT_CP1_ARM_SEL |
			       BIT_VBC_DMA_CP1_ARM_SEL));
		break;
	case AUDIO_TO_CP2_ARM_CTRL:
		val =
		    BITS_VBC_AFIFO_INT_SYS_SEL(3) | BITS_VBC_DA01_INT_SYS_SEL(3)
		    | BITS_VBC_AD01_INT_SYS_SEL(3)
		    | BITS_VBC_AD23_INT_SYS_SEL(3) |
		    BITS_VBC_DA01_DMA_SYS_SEL(3) | BITS_VBC_AD01_DMA_SYS_SEL(3)
		    | BITS_VBC_AD23_DMA_SYS_SEL(3);
		sci_glb_write(REG_AON_APB_VBC_CTRL, val, mask);
		break;
	case AUDIO_NO_CHANGE:
		ret =
		    sci_glb_read(REG_AON_APB_VBC_CTRL,
				 BITS_VBC_DA01_INT_SYS_SEL(3));
		if (ret == 0) {
			ret = AUDIO_TO_AP_ARM_CTRL;
		} else if (ret == 1) {
			ret =
			    sci_glb_read(REG_AON_APB_VBC_CTRL,
					 BIT_VBC_INT_CP0_ARM_SEL);
			if (ret)
				ret = AUDIO_TO_CP0_ARM_CTRL;
			else
				ret = AUDIO_TO_CP0_DSP_CTRL;
		} else if (ret == 2) {
			ret =
			    sci_glb_read(REG_AON_APB_VBC_CTRL,
					 BIT_VBC_INT_CP1_ARM_SEL);
			if (ret)
				ret = AUDIO_TO_CP1_ARM_CTRL;
			else
				ret = AUDIO_TO_CP1_DSP_CTRL;
		} else if (ret == 3) {
			ret = AUDIO_TO_CP2_ARM_CTRL;
		}
		break;
	default:
		ret = -ENODEV;
		break;
	}
#endif

	return ret;
}

static inline int arch_audio_vbc_da_dma_info(int chan)
{
	int ret = 0;

#if FIXED_AUDIO
	switch (chan) {
	case 0:
		ret = DMA_VB_DA0;
		break;
	case 1:
		ret = DMA_VB_DA1;
		break;
	default:
		ret = -ENODEV;
		break;
	}
#endif

	return ret;
}

static inline int arch_audio_vbc_ad_dma_info(int chan)
{
	int ret = 0;

#if FIXED_AUDIO
	switch (chan) {
	case 0:
		ret = DMA_VB_AD0;
		break;
	case 1:
		ret = DMA_VB_AD1;
		break;
	default:
		ret = -ENODEV;
		break;
	}
#endif

	return ret;
}

static inline int arch_audio_vbc_ad23_dma_info(int chan)
{
	int ret = 0;

#if FIXED_AUDIO
	switch (chan) {
	case 0:
		ret = DMA_VB_AD2;
		break;
	case 1:
		ret = DMA_VB_AD3;
		break;
	default:
		ret = -ENODEV;
		break;
	}
#endif

	return ret;
}

static inline int arch_audio_vbc_reset(void)
{
	int ret = 0;

#if FIXED_AUDIO
	sci_glb_set(REG_AON_APB_APB_RST0, BIT_VBC_SOFT_RST);
	udelay(10);
	sci_glb_clr(REG_AON_APB_APB_RST0, BIT_VBC_SOFT_RST);
#endif

	return ret;
}

#if 1
/* some SOC will move this into vbc module */
static inline int arch_audio_vbc_ad_int_clr(void)
{
	int ret = 0;

#if FIXED_AUDIO
	sci_glb_set((SPRD_INTC0_BASE + 0x0C), IRQ_REQ_AUD_VBC_AD01_INT);
#endif

	return ret;
}

static inline int arch_audio_vbc_ad23_int_clr(void)
{
	int ret = 0;

#if FIXED_AUDIO
	sci_glb_set((SPRD_INTC0_BASE + 0x0C), IRQ_REQ_AUD_VBC_AD23_INT);
#endif

	return ret;
}

/* some SOC will move this into vbc module */
static inline int arch_audio_vbc_da_int_clr(void)
{
	int ret = 0;

#if FIXED_AUDIO
	sci_glb_set((SPRD_INTC0_BASE + 0x0C), IRQ_REQ_AUD_VBC_DA_INT);
#endif
	return ret;
}

/* some SOC will move this into vbc module */
static inline int arch_audio_vbc_is_ad_int(void)
{
	int ret = 0;

#if FIXED_AUDIO
	sci_glb_read((SPRD_INTC0_BASE + 0x0), IRQ_REQ_AUD_VBC_AD01_INT);
#endif

	return ret;
}

/* some SOC will move this into vbc module */
static inline int arch_audio_vbc_is_ad23_int(void)
{
	int ret = 0;

#if FIXED_AUDIO
	sci_glb_read((SPRD_INTC0_BASE + 0x0), IRQ_REQ_AUD_VBC_AD23_INT);
#endif

	return ret;
}

/* some SOC will move this into vbc module */
static inline int arch_audio_vbc_is_da_int(void)
{
	int ret = 0;

#if FIXED_AUDIO
	sci_glb_read((SPRD_INTC0_BASE + 0x0), IRQ_REQ_AUD_VBC_DA_INT);
#endif

	return ret;
}
#endif
/* ------------------------------------------------------------------------- */

/* codec setting */
static inline int arch_audio_codec_write_mask(int reg, int val, int mask)
{
	int ret = 0;

#if FIXED_AUDIO
	ret = sci_adi_write(reg, val, mask);
#endif

	return ret;
}

static inline int arch_audio_codec_write(int reg, int val)
{
	int ret = 0;

#if FIXED_AUDIO
	ret = sci_adi_write(reg, val, 0xFFFF);
#endif

	return ret;
}

static inline int arch_audio_codec_read(int reg)
{
	int ret = 0;

#if FIXED_AUDIO
	ret = sci_adi_read(reg);
#endif

	return ret;
}

static inline int arch_audio_codec_audif_enable(int auto_clk)
{
	int ret = 0;

#if FIXED_AUDIO
	if (auto_clk) {
		sci_glb_clr(REG_AON_APB_APB_EB0, BIT_AUDIF_EB);
		sci_glb_set(REG_AON_APB_VBC_CTRL, BIT_AUDIF_CKG_AUTO_EN);
	} else {
		sci_glb_set(REG_AON_APB_APB_EB0, BIT_AUDIF_EB);
		sci_glb_clr(REG_AON_APB_VBC_CTRL, BIT_AUDIF_CKG_AUTO_EN);
	}
#endif

	return ret;
}

static inline int arch_audio_codec_audif_disable(void)
{
	int ret = 0;

#if FIXED_AUDIO
	sci_glb_clr(REG_AON_APB_APB_EB0, BIT_AUDIF_EB);
	sci_glb_clr(REG_AON_APB_VBC_CTRL, BIT_AUDIF_CKG_AUTO_EN);
#endif

	return ret;
}

static inline int arch_audio_codec_digital_reg_enable(void)
{
	int ret = 0;

#if FIXED_AUDIO
	sci_glb_set(REG_AON_APB_APB_EB0, BIT_AUD_EB);
	if (ret >= 0)
		arch_audio_codec_audif_enable(1);
#endif

	return ret;
}

static inline int arch_audio_codec_digital_reg_disable(void)
{
	int ret = 0;

#if FIXED_AUDIO
	arch_audio_codec_audif_disable();
	sci_glb_clr(REG_AON_APB_APB_EB0, BIT_AUD_EB);
#endif

	return ret;
}

static inline int arch_audio_codec_analog_reg_enable(void)
{
	int ret = 0;

#if FIXED_AUDIO
	ret =
		sci_adi_write(ANA_REG_GLB_ARM_MODULE_EN, BIT_ANA_AUD_EN,
						BIT_ANA_AUD_EN);
#endif

	return ret;
}

static inline int arch_audio_codec_analog_reg_disable(void)
{
	int ret = 0;

#if FIXED_AUDIO
	ret = sci_adi_write(ANA_REG_GLB_ARM_MODULE_EN, 0, BIT_ANA_AUD_EN);
#endif

	return ret;
}

static inline int arch_audio_codec_enable(void)
{
	int ret = 0;

#if FIXED_AUDIO
	int mask = BIT_CLK_AUD_6P5M_EN | BIT_CLK_AUDIF_EN;
	ret = sci_adi_write(ANA_REG_GLB_ARM_CLK_EN, mask, mask);

	ret = sci_adi_write(ANA_REG_GLB_AUDIO_CTRL, BIT_CLK_AUD_6P5M_TX_INV_EN, BIT_CLK_AUD_6P5M_TX_INV_EN);
#endif

	return ret;
}

static inline int arch_audio_codec_disable(void)
{
	int ret = 0;

#if FIXED_AUDIO
	int mask = BIT_CLK_AUD_6P5M_EN | BIT_CLK_AUDIF_EN;
	ret = sci_adi_write(ANA_REG_GLB_ARM_CLK_EN, 0, mask);

	ret = sci_adi_write(ANA_REG_GLB_AUDIO_CTRL, 0, BIT_CLK_AUD_6P5M_TX_INV_EN);
#endif

	return ret;
}

static inline int arch_audio_codec_switch(int master)
{
	int ret = 0;

#if FIXED_AUDIO
	switch (master) {
	case AUDIO_TO_AP_ARM_CTRL:
		sci_glb_write(REG_AON_APB_VBC_CTRL, BITS_AUD_INT_SYS_SEL(0),
			      BITS_AUD_INT_SYS_SEL(3));
		break;
	case AUDIO_TO_CP0_ARM_CTRL:
		sci_glb_write(REG_AON_APB_VBC_CTRL, BITS_AUD_INT_SYS_SEL(1),
			      BITS_AUD_INT_SYS_SEL(3));
		break;
	case AUDIO_TO_CP1_ARM_CTRL:
		sci_glb_write(REG_AON_APB_VBC_CTRL, BITS_AUD_INT_SYS_SEL(2),
			      BITS_AUD_INT_SYS_SEL(3));
		break;
	case AUDIO_TO_CP2_ARM_CTRL:
		sci_glb_write(REG_AON_APB_VBC_CTRL, BITS_AUD_INT_SYS_SEL(3),
			      BITS_AUD_INT_SYS_SEL(3));
		break;
	case AUDIO_NO_CHANGE:
		ret =
		    sci_glb_read(REG_AON_APB_VBC_CTRL, BITS_AUD_INT_SYS_SEL(3));
		if (ret == 0) {
			ret = AUDIO_TO_AP_ARM_CTRL;
		} else if (ret == 1) {
			ret = AUDIO_TO_CP0_ARM_CTRL;
		} else if (ret == 2) {
			ret = AUDIO_TO_CP1_ARM_CTRL;
		} else if (ret == 3) {
			ret = AUDIO_TO_CP2_ARM_CTRL;
		}
		break;
	default:
		ret = -ENODEV;
		break;
	}
#endif

	return ret;
}

static inline int arch_audio_codec_reset(void)
{
	int ret = 0;

#if FIXED_AUDIO
	int mask =
	    BIT_ANA_AUD_SOFT_RST | BIT_ANA_AUDTX_SOFT_RST |
	    BIT_ANA_AUDRX_SOFT_RST;
	sci_glb_set(REG_AON_APB_APB_RST0, BIT_AUD_SOFT_RST);
	sci_glb_set(REG_AON_APB_APB_RST0, BIT_AUDIF_SOFT_RST);
	ret = sci_adi_write(ANA_REG_GLB_ARM_RST, mask, mask);
	udelay(10);
	sci_glb_clr(REG_AON_APB_APB_RST0, BIT_AUD_SOFT_RST);
	sci_glb_clr(REG_AON_APB_APB_RST0, BIT_AUDIF_SOFT_RST);
	if (ret >= 0)
		ret = sci_adi_write(ANA_REG_GLB_ARM_RST, 0, mask);
#endif

	return ret;
}

/* ------------------------------------------------------------------------- */

/* i2s setting */

static inline int arch_audio_i2s_enable(int id)
{
	int ret = 0;

#if FIXED_AUDIO
	switch (id) {
	case 0:
		sci_glb_set(REG_AP_APB_APB_EB, BIT_IIS0_EB);
		break;
	case 1:
		sci_glb_set(REG_AP_APB_APB_EB, BIT_IIS1_EB);
		break;
	case 2:
		sci_glb_set(REG_AP_APB_APB_EB, BIT_IIS2_EB);
		break;
	case 3:
		sci_glb_set(REG_AP_APB_APB_EB, BIT_IIS3_EB);
		break;
	default:
		ret = -ENODEV;
		break;
	}
#endif

	return ret;
}

static inline int arch_audio_i2s_disable(int id)
{
	int ret = 0;

#if FIXED_AUDIO
	switch (id) {
	case 0:
		sci_glb_clr(REG_AP_APB_APB_EB, BIT_IIS0_EB);
		break;
	case 1:
		sci_glb_clr(REG_AP_APB_APB_EB, BIT_IIS1_EB);
		break;
	case 2:
		sci_glb_clr(REG_AP_APB_APB_EB, BIT_IIS2_EB);
		break;
	case 3:
		sci_glb_clr(REG_AP_APB_APB_EB, BIT_IIS3_EB);
		break;
	default:
		ret = -ENODEV;
		break;
	}
#endif

	return ret;
}

static inline int arch_audio_i2s_tx_dma_info(int id)
{
	int ret = 0;

#if FIXED_AUDIO
	switch (id) {
	case 0:
		ret = DMA_IIS0_TX;
		break;
	case 1:
		ret = DMA_IIS1_TX;
		break;
	case 2:
		ret = DMA_IIS2_TX;
		break;
	case 3:
		ret = DMA_IIS3_TX;
		break;
	default:
		ret = -ENODEV;
		break;
	}
#endif

	return ret;
}

static inline int arch_audio_i2s_rx_dma_info(int id)
{
	int ret = 0;

#if FIXED_AUDIO
	switch (id) {
	case 0:
		ret = DMA_IIS0_RX;
		break;
	case 1:
		ret = DMA_IIS1_RX;
		break;
	case 2:
		ret = DMA_IIS2_RX;
		break;
	case 3:
		ret = DMA_IIS3_RX;
		break;
	default:
		ret = -ENODEV;
		break;
	}

#endif

	return ret;
}

static inline int arch_audio_i2s_reset(int id)
{
	int ret = 0;

#if FIXED_AUDIO
	switch (id) {
	case 0:
		sci_glb_set(REG_AP_APB_APB_RST, BIT_IIS0_SOFT_RST);
		udelay(10);
		sci_glb_clr(REG_AP_APB_APB_RST, BIT_IIS0_SOFT_RST);
		break;
	case 1:
		sci_glb_set(REG_AP_APB_APB_RST, BIT_IIS1_SOFT_RST);
		udelay(10);
		sci_glb_clr(REG_AP_APB_APB_RST, BIT_IIS1_SOFT_RST);
		break;
	case 2:
		sci_glb_set(REG_AP_APB_APB_RST, BIT_IIS2_SOFT_RST);
		udelay(10);
		sci_glb_clr(REG_AP_APB_APB_RST, BIT_IIS2_SOFT_RST);
		break;
	case 3:
		sci_glb_set(REG_AP_APB_APB_RST, BIT_IIS3_SOFT_RST);
		udelay(10);
		sci_glb_clr(REG_AP_APB_APB_RST, BIT_IIS3_SOFT_RST);
		break;
	default:
		ret = -ENODEV;
		break;
	}
#endif

	return ret;
}

/*sc8830 AP IIS and CP IIS are different. AP IIS cann't switch to other master*/
static inline int arch_audio_i2s_switch(int id, int master)
{
	int ret = 0;

#if FIXED_AUDIO
	switch (master) {
	case AUDIO_TO_AP_ARM_CTRL:
		break;
	case AUDIO_NO_CHANGE:
		ret = AUDIO_TO_AP_ARM_CTRL;
		break;
	default:
		ret = -ENODEV;
		break;
	}
#endif
	return ret;
}
#endif
