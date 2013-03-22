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

#ifndef __AUDIO_GLB_REG_H
#define __AUDIO_GLB_REG_H

#include <linux/delay.h>
#include <mach/hardware.h>
#include <mach/globalregs.h>
#include <mach/adi.h>
#include <mach/dma.h>
#include <mach/irqs.h>

/* OKAY, this is for other else owner
   if you do not care the audio config
   you can set FIXED_AUDIO  to 0
   for compile happy.
*/
/* FIXME */
#define FIXED_AUDIO 1

enum {
	AUDIO_NO_CHANGE,
	AUDIO_TO_DSP_CTRL,
	AUDIO_TO_ARM_CTRL,
	AUDIO_TO_ARM_CP_CTRL,
};

#if FIXED_AUDIO
#define VBC_BASE		SPRD_VB_BASE
#define CODEC_DP_BASE 		SPRD_AUDTOP_BASE
#define CODEC_AP_BASE		(SPRD_MISC_BASE + 0x0700)
#define VBC_PHY_BASE		SPRD_VB_PHYS
#define CODEC_DP_PHY_BASE	SPRD_AUDTOP_PHYS
#define CODEC_AP_PHY_BASE	(SPRD_MISC_PHYS + 0x0700)
#define CODEC_AP_IRQ		(IRQ_ANA_AUD_PROTECT_AP_INT)
#define CODEC_DP_IRQ		(IRQ_AUD_INT)

#ifdef CONFIG_SPRD_AUDIO_BUFFER_USE_IRAM
#define SPRD_IRAM_ALL_PHYS	0XFFFF0000
#define SPRD_IRAM_ALL_SIZE	SZ_32K
#endif

#define	AUDIO_CTRL		(SPRD_MISC_BASE + 0x0888)
#define BIT_AUD_ARM_ACC                 ( BIT(15) )
#define BIT_AUDRX_ARM_SOFT_RST          ( BIT(10) )
#define BIT_AUDTX_ARM_SOFT_RST          ( BIT(9) )
#define BIT_AUD_ARM_SOFT_RST            ( BIT(8) )
#define BIT_AUD6M5_CLK_RX_INV_ARM_ENN   ( BIT(7) )
#define BIT_AUD6M5_CLK_TX_INV_ARM_EN    ( BIT(6) )
#define BIT_AUDIF_CLK_RX_INV_ARM_EN     ( BIT(5) )
#define BIT_AUDIF_CLK_TXT_INV_ARM_EN    ( BIT(4) )
#define BIT_CLK_AUD_6M5_ARM_EN          ( BIT(3) )
#define BIT_CLK_AUDIF_ARM_EN            ( BIT(2) )
#define BIT_RTC_AUD_ARM_EN              ( BIT(1) )
#define BIT_AUD_ARM_EN                  ( BIT(0) )

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
	sprd_greg_set_bits(REG_TYPE_GLOBAL, GEN0_VB_EN, GR_GEN0);
#endif

	return ret;
}

static inline int arch_audio_vbc_reg_disable(void)
{
	int ret = 0;

#if FIXED_AUDIO
	sprd_greg_clear_bits(REG_TYPE_GLOBAL, GEN0_VB_EN, GR_GEN0);
#endif

	return ret;
}

static inline int arch_audio_vbc_enable(void)
{
	int ret = 0;

#if FIXED_AUDIO
	sprd_greg_set_bits(REG_TYPE_GLOBAL, ARM_VB_ANAON, GR_BUSCLK);
#endif

	return ret;
}

static inline int arch_audio_vbc_disable(void)
{
	int ret = 0;

#if FIXED_AUDIO
	sprd_greg_clear_bits(REG_TYPE_GLOBAL, ARM_VB_ANAON, GR_BUSCLK);
#endif

	return ret;
}

static inline int arch_audio_vbc_switch(int master)
{
	int ret = 0;
/* sc7710g have two vbc modules, so just switch audif interface */
#if FIXED_AUDIO
	switch (master) {
	case AUDIO_TO_ARM_CTRL:
		sprd_greg_clear_bits(REG_TYPE_GLOBAL, BIT_AUD_IF_MUX, GR_AUD_CTRL);
		break;
	case AUDIO_TO_DSP_CTRL:
		sprd_greg_set_bits(REG_TYPE_GLOBAL, BIT_AUD_IF_MUX, GR_AUD_CTRL);
		break;
	case AUDIO_NO_CHANGE:
		ret = sprd_greg_read(REG_TYPE_GLOBAL, GR_AUD_CTRL) & BIT_AUD_IF_MUX;
		if (ret != 0)
			ret = AUDIO_TO_DSP_CTRL;
		else
			ret = AUDIO_TO_ARM_CTRL;
		break;
	default:
		ret = -ENODEV;
		break;
	}
#endif

	return ret;
}

static inline int arch_audio_vbc_ad_enable(int chan)
{
	int ret = 0;

#if FIXED_AUDIO
	switch (chan) {
	case 0:
		sprd_greg_set_bits(REG_TYPE_GLOBAL, ARM_VB_AD0ON, GR_BUSCLK);
		break;
	case 1:
		sprd_greg_set_bits(REG_TYPE_GLOBAL, ARM_VB_AD1ON, GR_BUSCLK);
		break;
	default:
		ret = -ENODEV;
		break;
	}
#endif

	return ret;
}

static inline int arch_audio_vbc_ad_disable(int chan)
{
	int ret = 0;

#if FIXED_AUDIO
	switch (chan) {
	case 0:
		sprd_greg_clear_bits(REG_TYPE_GLOBAL, ARM_VB_AD0ON, GR_BUSCLK);
		break;
	case 1:
		sprd_greg_clear_bits(REG_TYPE_GLOBAL, ARM_VB_AD1ON, GR_BUSCLK);
		break;
	default:
		ret = -ENODEV;
		break;
	}
#endif

	return ret;
}

static inline int arch_audio_vbc_da_enable(int chan)
{
	int ret = 0;

#if FIXED_AUDIO
	switch (chan) {
	case 0:
		sprd_greg_set_bits(REG_TYPE_GLOBAL, ARM_VB_DA0ON, GR_BUSCLK);
		break;
	case 1:
		sprd_greg_set_bits(REG_TYPE_GLOBAL, ARM_VB_DA1ON, GR_BUSCLK);
		break;
	default:
		ret = -ENODEV;
		break;
	}
#endif

	return ret;
}

static inline int arch_audio_vbc_da_disable(int chan)
{
	int ret = 0;

#if FIXED_AUDIO
	switch (chan) {
	case 0:
		sprd_greg_clear_bits(REG_TYPE_GLOBAL, ARM_VB_DA0ON, GR_BUSCLK);
		break;
	case 1:
		sprd_greg_clear_bits(REG_TYPE_GLOBAL, ARM_VB_DA1ON, GR_BUSCLK);
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

static inline int arch_audio_vbc_reset(void)
{
	int ret = 0;

#if FIXED_AUDIO
	sprd_greg_set_bits(REG_TYPE_GLOBAL, SWRST_VBC_RST, GR_SOFT_RST);
	udelay(10);
	sprd_greg_clear_bits(REG_TYPE_GLOBAL, SWRST_VBC_RST, GR_SOFT_RST);
#endif

	return ret;
}

/* some SOC will move this into vbc module */
static inline int arch_audio_vbc_ad_int_clr(void)
{
	int ret = 0;

#if FIXED_AUDIO
#endif

	return ret;
}

/* some SOC will move this into vbc module */
static inline int arch_audio_vbc_da_int_clr(void)
{
	int ret = 0;

#if FIXED_AUDIO
#endif

	return ret;
}

/* some SOC will move this into vbc module */
static inline int arch_audio_vbc_is_ad_int(void)
{
	int ret = 0;

#if FIXED_AUDIO
#endif

	return ret;
}

/* some SOC will move this into vbc module */
static inline int arch_audio_vbc_is_da_int(void)
{
	int ret = 0;

#if FIXED_AUDIO
#endif

	return ret;
}

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
	ret = sci_adi_raw_write(reg, val);
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
		sprd_greg_clear_bits(REG_TYPE_GLOBAL, BIT_AUD_IF_EB,
				     GR_CLK_GEN7);
		sprd_greg_set_bits(REG_TYPE_GLOBAL, BIT_AUDIF_AUTO_EN,
				   GR_AUD_CTRL);
	} else {
		sprd_greg_set_bits(REG_TYPE_GLOBAL, BIT_AUD_IF_EB, GR_CLK_GEN7);
		sprd_greg_clear_bits(REG_TYPE_GLOBAL, BIT_AUDIF_AUTO_EN,
				     GR_AUD_CTRL);
	}
#endif

	return ret;
}

static inline int arch_audio_codec_audif_disable(void)
{
	int ret = 0;

#if FIXED_AUDIO
	sprd_greg_clear_bits(REG_TYPE_GLOBAL, BIT_AUDIF_AUTO_EN, GR_AUD_CTRL);
	sprd_greg_clear_bits(REG_TYPE_GLOBAL, BIT_AUD_IF_EB, GR_CLK_GEN7);
#endif

	return ret;
}

static inline int arch_audio_codec_digital_reg_enable(void)
{
	int ret = 0;

#if FIXED_AUDIO
	sprd_greg_set_bits(REG_TYPE_GLOBAL, BIT_AUD_TOP_EB, GR_CLK_GEN7);
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
	sprd_greg_clear_bits(REG_TYPE_GLOBAL, BIT_AUD_TOP_EB, GR_CLK_GEN7);
#endif

	return ret;
}

static inline int arch_audio_codec_analog_reg_enable(void)
{
	int ret = 0;

#if FIXED_AUDIO
	ret = sci_adi_write(AUDIO_CTRL, BIT_AUD_ARM_EN, BIT_AUD_ARM_EN);
#endif

	return ret;
}

static inline int arch_audio_codec_analog_reg_disable(void)
{
	int ret = 0;

#if FIXED_AUDIO
	ret = sci_adi_write(AUDIO_CTRL, 0, BIT_AUD_ARM_EN);
#endif

	return ret;
}

static inline int arch_audio_codec_enable(void)
{
	int ret = 0;

#if FIXED_AUDIO
	int mask = BIT_AUD6M5_CLK_TX_INV_ARM_EN |
	    BIT_RTC_AUD_ARM_EN | BIT_CLK_AUD_6M5_ARM_EN | BIT_CLK_AUDIF_ARM_EN;
	ret = sci_adi_write(AUDIO_CTRL, mask, mask);
#endif

	return ret;
}

static inline int arch_audio_codec_disable(void)
{
	int ret = 0;

#if FIXED_AUDIO
	int mask =
	    BIT_RTC_AUD_ARM_EN | BIT_CLK_AUD_6M5_ARM_EN | BIT_CLK_AUDIF_ARM_EN;
	ret = sci_adi_write(AUDIO_CTRL, 0, mask);
#endif

	return ret;
}

static inline int arch_audio_codec_switch(int master)
{
	int ret = 0;

#if FIXED_AUDIO
	switch (master) {
	case AUDIO_TO_ARM_CTRL:
		sprd_greg_set_bits(REG_TYPE_GLOBAL,
				   BIT_AUD_CTL_SEL | BIT_AUD_CLK_SEL,
				   GR_AUD_CTRL);
		/* adi interface control from SC8810, OOPS */
		sprd_greg_set_bits(REG_TYPE_GLOBAL, ARM_VB_ACC, GR_BUSCLK);
		ret =
		    sci_adi_write(AUDIO_CTRL, BIT_AUD_ARM_ACC, BIT_AUD_ARM_ACC);
		break;
	case AUDIO_TO_DSP_CTRL:
		sprd_greg_clear_bits(REG_TYPE_GLOBAL,
				     BIT_AUD_CTL_SEL | BIT_AUD_CLK_SEL,
				     GR_AUD_CTRL);
		/* adi interface control from SC8810, OOPS */
		sprd_greg_clear_bits(REG_TYPE_GLOBAL, ARM_VB_ACC, GR_BUSCLK);
		ret = sci_adi_write(AUDIO_CTRL, 0, BIT_AUD_ARM_ACC);
		break;
	case AUDIO_NO_CHANGE:
		ret = sci_adi_read(AUDIO_CTRL) & BIT_AUD_ARM_ACC;
		if (ret == 0)
			ret = AUDIO_TO_DSP_CTRL;
		else
			ret = AUDIO_TO_ARM_CTRL;
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
	    BIT_AUD_ARM_SOFT_RST | BIT_AUDTX_ARM_SOFT_RST |
	    BIT_AUDRX_ARM_SOFT_RST;
	sprd_greg_set_bits(REG_TYPE_GLOBAL, BIT_AUDTOP_SOFT_RST,
			   GR_PERI_SOFT_RST2);
	sprd_greg_set_bits(REG_TYPE_GLOBAL, BIT_AUDIF_SOFT_RST,
			   GR_PERI_SOFT_RST2);
	ret = sci_adi_write(AUDIO_CTRL, mask, mask);
	udelay(10);
	sprd_greg_clear_bits(REG_TYPE_GLOBAL, BIT_AUDTOP_SOFT_RST,
			     GR_PERI_SOFT_RST2);
	sprd_greg_clear_bits(REG_TYPE_GLOBAL, BIT_AUDIF_SOFT_RST,
			     GR_PERI_SOFT_RST2);
	if (ret >= 0)
		ret = sci_adi_write(AUDIO_CTRL, 0, mask);
#endif

	return ret;
}

/* ------------------------------------------------------------------------- */

/* i2s setting */
static inline const char * arch_audio_i2s_clk_name(int id)
{
#if FIXED_AUDIO
	switch (id) {
	case 0:
		return "clk_iis";
		break;
	case 1:
		return "clk_iis1";
		break;
	default:
		break;
	}
	return NULL;
#endif
}

static inline int arch_audio_i2s_enable(int id)
{
	int ret = 0;

#if FIXED_AUDIO
	switch (id) {
	case 0:
		sprd_greg_set_bits(REG_TYPE_GLOBAL, GEN0_I2S0_EN, GR_GEN0);
		break;
	case 1:
		sprd_greg_set_bits(REG_TYPE_GLOBAL, GEN0_I2S1_EN, GR_GEN0);
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
		sprd_greg_clear_bits(REG_TYPE_GLOBAL, GEN0_I2S0_EN, GR_GEN0);
		break;
	case 1:
		sprd_greg_clear_bits(REG_TYPE_GLOBAL, GEN0_I2S1_EN, GR_GEN0);
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
		ret = DMA_IIS_TX;
		break;
	case 1:
		ret = DMA_IIS1_TX;
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
		ret = DMA_IIS_RX;
		break;
	case 1:
		ret = DMA_IIS1_RX;
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
		sprd_greg_set_bits(REG_TYPE_GLOBAL, SWRST_IIS_RST, GR_SOFT_RST);
		udelay(10);
		sprd_greg_clear_bits(REG_TYPE_GLOBAL, SWRST_IIS_RST,
				     GR_SOFT_RST);
		break;
	case 1:
		sprd_greg_set_bits(REG_TYPE_GLOBAL, SWRST_IIS1_RST,
				   GR_SOFT_RST);
		udelay(10);
		sprd_greg_clear_bits(REG_TYPE_GLOBAL, SWRST_IIS1_RST,
				     GR_SOFT_RST);
		break;
	default:
		ret = -ENODEV;
		break;
	}
#endif

	return ret;
}

static inline int arch_audio_i2s_switch(int id, int master)
{
	int ret = 0;

#if FIXED_AUDIO
	switch (id) {
	case 0:
		switch (master) {
		case AUDIO_TO_ARM_CTRL:
			sprd_greg_clear_bits(REG_TYPE_GLOBAL, IIS0_SEL, GR_PCTL);
			break;
		case AUDIO_TO_DSP_CTRL:
			sprd_greg_set_bits(REG_TYPE_GLOBAL, IIS0_SEL,
					     GR_PCTL);
			break;
		case AUDIO_NO_CHANGE:
			ret =
			    sprd_greg_read(REG_TYPE_GLOBAL, GR_PCTL) & IIS0_SEL;
			if (ret != 0)
				ret = AUDIO_TO_DSP_CTRL;
			else
				ret = AUDIO_TO_ARM_CTRL;
			break;
		default:
			ret = -ENODEV;
			break;
		}
		break;
	case 1:
		switch (master) {
		case AUDIO_TO_ARM_CTRL:
			sprd_greg_clear_bits(REG_TYPE_GLOBAL, IIS1_SEL, GR_PCTL);
			break;
		case AUDIO_TO_DSP_CTRL:
			sprd_greg_set_bits(REG_TYPE_GLOBAL, IIS1_SEL,
					     GR_PCTL);
			break;
		case AUDIO_NO_CHANGE:
			ret =
			    sprd_greg_read(REG_TYPE_GLOBAL, GR_PCTL) & IIS1_SEL;
			if (ret != 0)
				ret = AUDIO_TO_DSP_CTRL;
			else
				ret = AUDIO_TO_ARM_CTRL;
			break;
		default:
			ret = -ENODEV;
			break;
		}
		break;
	default:
		ret = -ENODEV;
		break;
	}
#endif

	return ret;
}

#endif
