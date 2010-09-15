/* 
 * sound/soc/sc88xx/vbc-codec.h
 *
 * VBC -- SpreadTrum sc88xx intergrated codec.
 *
 * Copyright (C) 2010 SpreadTrum Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY ork FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __VBC_CODEC_H
#define __VBC_CODEC_H

#define CHIP_VER_8800H5

#include <sound/soc-dai.h>
#include <mach/hardware.h>
#include <mach/regs_global.h>
#include <mach/regs_ahb.h>
#include <mach/regs_int.h>

#define VBC_FIFO_FRAME_NUM      160

#define VBC_RATE_8000   (10)
#define VBC_RATE_9600   ( 9)
#define VBC_RATE_11025  ( 8)
#define VBC_RATE_12000  ( 7)
#define VBC_RATE_16000  ( 6)
#define VBC_RATE_22050  ( 5)
#define VBC_RATE_24000  ( 4)
#define VBC_RATE_32000  ( 3)
#define VBC_RATE_44100  ( 2)
#define VBC_RATE_48000  ( 1)
#define VBC_RATE_96000  ( 0)

// VBADBUFFDTA
enum {
    VBISADCK_INV = 9,
    VBISDACK_INV,
    VBLSB_EB,
    VBIIS_DLOOP = 13,
    VBPCM_MODE,
    VBIIS_LRCK,
};
// VBDABUFFDTA
enum {
    RAMSW_NUMB = 9,
    RAMSW_EN,
    VBAD0DMA_EN,
    VBAD1DMA_EN,
    VBDA0DMA_EN,
    VBDA1DMA_EN,
    VBENABLE,
};
// VBAICR
#define VBCAICR_MODE_ADC_I2S    (1 << 0)
#define VBCAICR_MODE_DAC_I2S    (1 << 1)
#define VBCAICR_MODE_ADC_SERIAL (1 << 2)
#define VBCAICR_MODE_DAC_SERIAL (1 << 3)
// VBCR1
enum {
    BTL_MUTE = 1,
    BYPASS,
    DACSEL,
    HP_DIS,
    DAC_MUTE,
    MONO,
    SB_MICBIAS,
};
// VBCR2
#define DAC_DATA_WIDTH_16_bit   (0x00)
#define DAC_DATA_WIDTH_18_bit   (0x01)
#define DAC_DATA_WIDTH_20_bit   (0x02)
#define DAC_DATA_WIDTH_24_bit   (0x03)

#define ADC_DATA_WIDTH_16_bit   (0x00)
#define ADC_DATA_WIDTH_18_bit   (0x01)
#define ADC_DATA_WIDTH_20_bit   (0x02)
#define ADC_DATA_WIDTH_24_bit   (0x03)

#define MICROPHONE1              (0)
#define MICROPHONE2              (1)
enum {
    MICSEL = 1,
    ADC_HPF,
    ADC_ADWL,
    DAC_ADWL = 5,
    DAC_DEEMP = 7,
};
// VBPMR1
enum {
    SB_LOUT = 1,
    SB_BTL,
    SB_LIN,
    SB_ADC,
    SB_MIX,
    SB_OUT,
    SB_DAC,
};
// VBPMR2
enum {
    SB_SLEEP = 0,
    SB,
    SB_MC,
    GIM,
    RLGOD,
    LRGOD,
};
//--------------------------
#define ARM_VB_BASE SPRD_VB_BASE

#define VBDA0       (ARM_VB_BASE + 0x0000) // 0x0000  Voice band DAC0 data buffer
#define VBDA1       (ARM_VB_BASE + 0x0004) // 0x0002  Voice band DAC1 data buffer
#define VBAD0       (ARM_VB_BASE + 0x0008) // 0x0004  Voice band ADC0 data buffer
#define VBAD1       (ARM_VB_BASE + 0x000C) // 0x0006  Voice band ADC1 data buffer
#define VBBUFFSIZE  (ARM_VB_BASE + 0x0010) // 0x0008  Voice band buffer size
#define VBADBUFFDTA (ARM_VB_BASE + 0x0014) // 0x000A  Voice band AD buffer control
#define VBDABUFFDTA (ARM_VB_BASE + 0x0018) // 0x000C  Voice band DA buffer control
#define VBADCNT     (ARM_VB_BASE + 0x001C) // 0x000E  Voice band AD buffer counter
#define VBDACNT     (ARM_VB_BASE + 0x0020) // 0x0010  Voice band DA buffer counter
#define VBDAICTL    (ARM_VB_BASE + 0x0024) // 0x0012  Voice band DAI control
#define VBDAIIN     (ARM_VB_BASE + 0x0028) // 0x0014  Voice band DAI input
#define VBDAIOUT    (ARM_VB_BASE + 0x002C) // 0x0016  Voice band DAI output
#define VBAICR      (ARM_VB_BASE + 0x0100) // 0x0080  Voice band Codec AICR
#define VBCR1       (ARM_VB_BASE + 0x0104) // 0x0082  Voice band Codec CR1
#define VBCR2       (ARM_VB_BASE + 0x0108) // 0x0084  Voice band Codec CR2
#define VBCCR1      (ARM_VB_BASE + 0x010C) // 0x0086  Voice band Codec CCR1
#define VBCCR2      (ARM_VB_BASE + 0x0110) // 0x0088  Voice band Codec CCR2
#define VBPMR1      (ARM_VB_BASE + 0x0114) // 0x008A  Voice band Codec PMR1
#define VBPMR2      (ARM_VB_BASE + 0x0118) // 0x008C  Voice band Codec PMR2
#define VBCRR       (ARM_VB_BASE + 0x011C) // 0x008E  Voice band Codec CRR
#define VBICR       (ARM_VB_BASE + 0x0120) // 0x0090  Voice band Codec ICR
#define VBIFR       (ARM_VB_BASE + 0x0124) // 0x0092  Voice band Codec IFR
#define VBCGR1      (ARM_VB_BASE + 0x0128) // 0x0094  Voice band Codec CGR1
#define VBCGR2      (ARM_VB_BASE + 0x012C) // 0x0096  Voice band Codec CGR2
#define VBCGR3      (ARM_VB_BASE + 0x0130) // 0x0098  Voice band Codec CGR3
#define VBCGR8      (ARM_VB_BASE + 0x0144) // 0x00A2  Voice band Codec CGR8
#define VBCGR9      (ARM_VB_BASE + 0x0148) // 0x00A4  Voice band Codec CGR9
#define VBCGR10     (ARM_VB_BASE + 0x014C) // 0x00A6  Voice band Codec CGR10
#define VBTR1       (ARM_VB_BASE + 0x0150) // 0x00A8  Voice band Codec TR1
#define VBTR2       (ARM_VB_BASE + 0x0154) // 0x00AA  Voice band Codec TR2
//--------------------------
static inline void vbc_reg_write(u32 reg, u8 shift, u32 val, u32 mask)
{
    unsigned long flags;
    u32 tmp;
    raw_local_irq_save(flags);
    tmp = __raw_readl(reg);
    tmp &= ~(mask<<shift);
    tmp |= val << shift;
    __raw_writel(tmp, reg);
    raw_local_irq_restore(flags);
}
//--------------------------
extern struct snd_soc_codec_device vbc_codec;
extern struct snd_soc_dai vbc_dai[];

#endif
