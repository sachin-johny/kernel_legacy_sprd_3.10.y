/* 
 * sound/soc/sc88xx/vbc-codec.c
 *
 * VBC -- SpreadTrum sc88xx intergrated Dolphin codec.
 *
 * Copyright (C) 2010 SpreadTrum Ltd.
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
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/kernel.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/soc-dapm.h>
#include <linux/delay.h>
#include <sound/tlv.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/stat.h>
#include <linux/reboot.h>

#include "aud_enha.h"

#include "sc88xx-asoc.h"


#define VBC_DYNAMIC_POWER_MANAGEMENT    0
#define POWER_OFF_ON_STANDBY    0
#define VBC_CODEC_RESET    0xffff
#define VBC_CODEC_POWER    0xfffe
#define VBC_CODEC_POWER_ON_OUT  (1 << 0)
#define VBC_CODEC_POWER_ON_IN   (1 << 1)
#define VBC_CODEC_POWER_OFF_OUT (1 << 2)
#define VBC_CODEC_POWER_OFF_IN  (1 << 3)
#define VBC_CODEC_POWER_ON_OUT_MUTE_DAC (1 << 4)
#define VBC_CODEC_POWER_ON_FORCE        (1 << 29)
#define VBC_CODEC_POWER_DOWN_FORCE      (1 << 30)
#define VBC_CODEC_SPEAKER_PA 0xfffd
#define VBC_CODEC_DSP      0xfffc
#define VBC_CODEC_POWER2   0xfffb
/*
  ALSA SOC usually puts the device in standby mode when it's not used
  for sometime. If you define POWER_OFF_ON_STANDBY the driver will
  turn off the ADC/DAC when this callback is invoked and turn it back
  on when needed. Unfortunately this will result in a very light bump
  (it can be audible only with good earphones). If this bothers you
  just comment this line, you will have slightly higher power
  consumption . 
 */
static const unsigned int dac_tlv[] = {
    TLV_DB_RANGE_HEAD(1),
        0, 0xf, TLV_DB_LINEAR_ITEM(5, 450),
};

static const unsigned int adc_tlv[] = {
    TLV_DB_RANGE_HEAD(1),
        0, 0xf, TLV_DB_LINEAR_ITEM(0, 4500),
};

static const unsigned int dac_tlv_fm[] = {
    TLV_DB_RANGE_HEAD(1),
        0, 0x1f, TLV_DB_LINEAR_ITEM(5, 450),
};

static const char *vbc_mic_sel[] = {
    "1",
    "2",
};

static const struct soc_enum vbc_mic12_enum = 
    SOC_ENUM_SINGLE(VBCR2 & 0xffff,
        MICSEL,
        2,
        vbc_mic_sel);

static const char *vbc_codec_reset_enum_sel[] = {
    "false",
    "true",
};

static const struct soc_enum vbc_codec_reset_enum =
    SOC_ENUM_SINGLE(VBC_CODEC_RESET,
        0,
        2,
        vbc_codec_reset_enum_sel);

#define VBC_PCM_CTRL(name) \
    SOC_SINGLE(name" Playback Switch", VBCR1, DAC_MUTE, 1, 1), \
    SOC_DOUBLE_TLV(name" Playback Volume", VBCGR1, 0, 4, 0x0f, 1, dac_tlv), \
    SOC_SINGLE_TLV(name" Left Playback Volume", VBCGR1, 0, 0x0f, 1, dac_tlv), \
    SOC_SINGLE_TLV(name" Right Playback Volume",VBCGR1, 4, 0x0f, 1, dac_tlv)

static const struct snd_kcontrol_new vbc_snd_controls[] = {
    // Mic
    SOC_ENUM("Micphone", vbc_mic12_enum),
    // PCM
    SOC_SINGLE("PCM Playback Switch", VBCR1, DAC_MUTE, 1, 1),
 //   SOC_DOUBLE_TLV("PCM Playback Volume", VBCGR1, 0, 4, 0x0f, 1, dac_tlv),
 //   SOC_SINGLE_TLV("PCM Left Playback Volume", VBCGR1, 0, 0x0f, 1, dac_tlv),
 //   SOC_SINGLE_TLV("PCM Right Playback Volume",VBCGR1, 4, 0x0f, 1, dac_tlv),
    SOC_SINGLE("PCM2 Playback Switch", VBCR1, DAC_MUTE, 1, 1),
    SOC_DOUBLE_TLV("PCM2 Playback Volume", VBCGR1, 0, 4, 0x0f, 1, dac_tlv),
    SOC_SINGLE_TLV("PCM2 Left Playback Volume", VBCGR1, 0, 0x0f, 1, dac_tlv),
    SOC_SINGLE_TLV("PCM2 Right Playback Volume",VBCGR1, 4, 0x0f, 1, dac_tlv),
    // Speaker
    /* SOC_SINGLE("Speaker Playback Switch", VBCR1, xxx, 1, 1), */
    SOC_SINGLE("Speaker Playback Switch", VBC_CODEC_SPEAKER_PA, 0, 1, 0),
 //   SOC_DOUBLE_TLV("Speaker Playback Volume", VBCGR1, 0, 4, 0x0f, 1, dac_tlv),
 //   SOC_SINGLE_TLV("Speaker Left Playback Volume", VBCGR1, 0, 0x0f, 1, dac_tlv),
 //   SOC_SINGLE_TLV("Speaker Right Playback Volume",VBCGR1, 4, 0x0f, 1, dac_tlv),
    SOC_SINGLE("Speaker2 Playback Switch", VBC_CODEC_SPEAKER_PA, 0, 1, 0),
    SOC_DOUBLE_TLV("Speaker2 Playback Volume", VBCGR1, 0, 4, 0x0f, 1, dac_tlv),
    SOC_SINGLE_TLV("Speaker2 Left Playback Volume", VBCGR1, 0, 0x0f, 1, dac_tlv),
    SOC_SINGLE_TLV("Speaker2 Right Playback Volume",VBCGR1, 4, 0x0f, 1, dac_tlv),
    // Earpiece
    SOC_SINGLE("Earpiece Playback Switch", VBCR1, BTL_MUTE, 1, 1),
 //   SOC_DOUBLE_TLV("Earpiece Playback Volume", VBCGR1, 0, 4, 0x0f, 1, dac_tlv),
 //   SOC_SINGLE_TLV("Earpiece Left Playback Volume", VBCGR1, 0, 0x0f, 1, dac_tlv),
 //   SOC_SINGLE_TLV("Earpiece Right Playback Volume",VBCGR1, 4, 0x0f, 1, dac_tlv),
    SOC_SINGLE("Earpiece2 Playback Switch", VBCR1, BTL_MUTE, 1, 1),
    SOC_DOUBLE_TLV("Earpiece2 Playback Volume", VBCGR1, 0, 4, 0x0f, 1, dac_tlv),
    SOC_SINGLE_TLV("Earpiece2 Left Playback Volume", VBCGR1, 0, 0x0f, 1, dac_tlv),
    SOC_SINGLE_TLV("Earpiece2 Right Playback Volume",VBCGR1, 4, 0x0f, 1, dac_tlv),
    // Bypass
    SOC_SINGLE("BypassFM Playback Switch", VBCR1, BYPASS, 1, 0),
    SOC_DOUBLE_R_TLV("BypassFM Playback Volume", VBCGR2, VBCGR3, 0, 0x1f, 1, dac_tlv_fm),
    SOC_SINGLE_TLV("BypassFM Left Playback Volume", VBCGR2, 0, 0x1f, 1, dac_tlv_fm),
    SOC_SINGLE_TLV("BypassFM Right Playback Volume",VBCGR3, 0, 0x1f, 1, dac_tlv_fm),
    // Linein
    SOC_SINGLE("LineinFM", VBPMR1, SB_LIN, 1, 1),
#if defined(CONFIG_ARCH_SC8800G)
    SOC_SINGLE("LineinFM_Record", ANA_CHGR_CTL0, 15, 1, 0),
#elif defined(CONFIG_ARCH_SC8810)
    SOC_SINGLE("LineinFM_Record", ANA_AUDIO_CTRL, 3, 1, 0),
#endif
    // Headset
    SOC_SINGLE("Headset Playback Switch", VBCR1, HP_DIS, 1, 1),
 //   SOC_DOUBLE_TLV("Headset Playback Volume", VBCGR1, 0, 4, 0x0f, 1, dac_tlv),
 //   SOC_SINGLE_TLV("Headset Left Playback Volume", VBCGR1, 0, 0x0f, 1, dac_tlv),
 //   SOC_SINGLE_TLV("Headset Right Playback Volume",VBCGR1, 4, 0x0f, 1, dac_tlv),
    SOC_SINGLE("Headset2 Playback Switch", VBCR1, HP_DIS, 1, 1),
    SOC_DOUBLE_TLV("Headset2 Playback Volume", VBCGR1, 0, 4, 0x0f, 1, dac_tlv),
    SOC_SINGLE_TLV("Headset2 Left Playback Volume", VBCGR1, 0, 0x0f, 1, dac_tlv),
    SOC_SINGLE_TLV("Headset2 Right Playback Volume",VBCGR1, 4, 0x0f, 1, dac_tlv),
    // Capture
    SOC_SINGLE_TLV("Capture Capture Volume", VBCGR10, 4, 0x0f, 0, adc_tlv),
    // reset codec
    SOC_ENUM("Reset Codec", vbc_codec_reset_enum),
    // codec power
    //     poweron   poweroff
    // bit 0    1     2     3   4 ... 31
    //     out  in    out   in
    // value 1 is valid, value 0 is invalid
    SOC_SINGLE("Power Codec", VBC_CODEC_POWER, 0, 31, 0),
    SOC_SINGLE("Power Codec2",VBC_CODEC_POWER2,0, 31, 0),
    SOC_SINGLE("InCall", VBC_CODEC_DSP, 0, 1, 0),
};
static int32_t cur_sample_rate=44100;
static int32_t cur_internal_pa_gain = 0x8;   //default:1000----0db  added by jian

u32 vbc_reg_write(u32 reg, u8 shift, u32 val, u32 mask)
{
#ifdef CONFIG_ARCH_SC8800S
    unsigned long flags;
    u32 tmp, ret;
    raw_local_irq_save(flags);
    ret = tmp = __raw_readl(reg);
    tmp &= ~(mask << shift);
    tmp |= val << shift;
    __raw_writel(tmp, reg);
    raw_local_irq_restore(flags);
#elif defined(CONFIG_ARCH_SC8800G) || \
      defined(CONFIG_ARCH_SC8810)
    u32 tmp, ret;
    // raw_local_irq_save(flags);
    enter_critical();
    if (not_in_adi_range(reg)) tmp = __raw_readl(reg);
    else tmp = __raw_adi_read(reg);
    ret = tmp;
    tmp &= ~(mask << shift);
    tmp |= val << shift;
    if (not_in_adi_range(reg)) __raw_writel(tmp, reg);
    else __raw_adi_write(tmp, reg);
    // raw_local_irq_restore(flags);
    exit_critical();
#endif
    return ret & (mask << shift);
}
EXPORT_SYMBOL_GPL(vbc_reg_write);

u32 vbc_reg_read(u32 reg, u8 shift, u32 mask)
{
#ifdef CONFIG_ARCH_SC8800S
    unsigned long flags;
    u32 tmp;
    raw_local_irq_save(flags);
    tmp = __raw_readl(reg);
    // tmp &= mask << shift;
    // tmp |= val << shift;
    // __raw_writel(tmp, reg);
    raw_local_irq_restore(flags);
#elif defined(CONFIG_ARCH_SC8800G) || \
      defined(CONFIG_ARCH_SC8810)
    u32 tmp;
    enter_critical();
    if (not_in_adi_range(reg)) tmp = __raw_readl(reg);
    else tmp = __raw_adi_read(reg);
    // tmp &= ~(mask << shift);
    // tmp |= val << shift;
    // if (not_in_adi_range(reg)) __raw_writel(tmp, reg);
    // else __raw_adi_write(tmp, reg);
    exit_critical();
#endif
    return tmp & (mask << shift);
}
EXPORT_SYMBOL_GPL(vbc_reg_read);

int print_cpu_regs(u32 paddr, u32 offset, int nword, char *buf, int max, bool head, const char *prefix)
{
    int i;
    u32 ivaddr;
    u32 vaddr;
    char *bufb = buf;
    char *maxp = buf + max;
    paddr += offset;
    vaddr = (u32)ioremap(paddr, nword << 2);
    /* print a serial of reg in a formated way */
if (head) {
    if (buf) {
        if (prefix) buf += snprintf(buf, maxp - buf, "%s", prefix);
        else buf += snprintf(buf, maxp - buf, "%s",
                            "  ADDRESS  |   VALUE\n"
                            "-----------+-----------\n");
    } else {
        if (prefix) printk("%s", prefix);
        else printk("%s",
                    "  ADDRESS  |   VALUE\n"
                    "-----------+-----------\n");
    }
}
    for (i = 0; i < nword; i++) {
        if (paddr < (SPRD_ADI_PHYS + ANA_REG_ADDR_START - SPRD_ADI_BASE) ||
            paddr > (SPRD_ADI_PHYS + ANA_REG_ADDR_START - SPRD_ADI_BASE + ANA_REG_ADDR_END - ANA_REG_ADDR_START))
            ivaddr = vaddr;
        else ivaddr = ANA_REG_ADDR_START + paddr - (SPRD_ADI_PHYS + ANA_REG_ADDR_START - SPRD_ADI_BASE);
        if (buf) {
            buf += snprintf(buf, maxp - buf,
                            "0x%08x |     0x%04x\n",
                            paddr, vbc_reg_read(ivaddr, 0, UINT_MAX));
        } else       printk("0x%08x |     0x%04x\n",
                            paddr, vbc_reg_read(ivaddr, 0, UINT_MAX));
        paddr += 4;
        vaddr += 4;
    }

    return bufb - buf;
}
static void vbc_dump_regs(u32 cmd, char *buf, int max)
{
    char *bufb, *buft;
    static char buf2[1024*64];
    if (!buf) {
        buf = buf2;
        max = sizeof buf2;
    }
    bufb = buf;
    buft = bufb + max;
    switch (cmd) {
        case 0: {
            buf += print_cpu_regs(SPRD_VB_PHYS , 0, 107, 0, buft - buf, 1, "vbc_dump_regs VBDA0 - HPCOEF42\n");
            buf += print_cpu_regs(SPRD_ADI_PHYS, 0x0100, 22, 0, buft - buf, 1, "VBAICR - VBTR2\n");
            // printk("%s", bufb);
        }
        default: break;
    }
}

static void vbc_print_reg(uint32_t phy, int num)
{
    print_cpu_regs(phy, 0, num, NULL, 0, 0, NULL);
}

static void vbc_print_regs(int type)
{
#if 0
    printk("---------------- [ start %d ] ----------------\n", type);
    vbc_print_reg(0x82000104, 1); // VBCR1
    vbc_print_reg(0x82000674, 3); // ANA_AUDIO_CTRL
    vbc_print_reg(0x82000680, 1); // ANA_MIXED_CTRL
    vbc_print_reg(0x82000610, 1); // ANA_LDO_PD_CTL0
    vbc_print_reg(0x82000620, 1); // ANA_LDO_VCTL2
    printk("---------------- [ end %d ] ----------------\n", type);
#endif
}

static const struct snd_soc_dapm_widget vbc_dapm_widgets[] = {
    
};

/* vbc supported audio map */
static const struct snd_soc_dapm_route audio_map[] = {
    
};

static int vbc_add_widgets(struct snd_soc_codec *codec)
{
    snd_soc_dapm_new_controls(codec, vbc_dapm_widgets,
				  ARRAY_SIZE(vbc_dapm_widgets));

    snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

    // snd_soc_init_card will do this.
    // snd_soc_dapm_new_widgets(codec); 

    return 0;
}

static inline void vbc_reg_VBAICR_set(u8 mode)
{
    vbc_reg_write(VBAICR, 0, mode, 0x0F);
}

static inline int vbc_reg_VBCR1_set(u32 type, u32 val)
{
    return vbc_reg_write(VBCR1, type, val, 1);
}

static void vbc_reg_VBCR2_set(u32 type, u32 val)
{
    u32 mask;

    switch (type) {
        case ADC_ADWL:
        case DAC_ADWL:
            mask = 0x03; break;
        default: 
            mask = 1; break;
    }
    vbc_reg_write(VBCR2, type, val, mask);
}

static inline void vbc_reg_VBPMR2_set(u32 type, u32 val)
{
    vbc_reg_write(VBPMR2, type, val, 1);
}

static inline void vbc_reg_VBPMR1_set(u32 type, u32 val)
{
    vbc_reg_write(VBPMR1, type, val, 1);
}

static inline void vbc_set_AD_DA_fifo_frame_num(u8 da_fifo_fnum, u8 ad_fifo_fnum)
{
    vbc_reg_write(VBBUFFSIZE, 0, ((da_fifo_fnum-1)<<8) | ((ad_fifo_fnum-1)), 0xffff);
}

static inline void vbc_reg_VBDABUFFDTA_set(u32 type, u32 val)
{
    vbc_reg_write(VBDABUFFDTA, type, val, 1);
}

static inline void vbc_set_VBADBUFFDTA_set(u32 type, u32 val)
{
    vbc_reg_write(VBADBUFFDTA, type, val, 1);
}

static inline void vbc_access_buf(int enable)
{
    // Software access ping-pong buffer enable when VBENABE bit low
    vbc_reg_VBDABUFFDTA_set(RAMSW_EN, enable ? 1:0);
}

static void vbc_buffer_clear(int id)
{
    int i;
    vbc_reg_VBDABUFFDTA_set(RAMSW_NUMB, id ? 1:0);
    for (i = 0; i < VBC_FIFO_FRAME_NUM; i++) {
        __raw_writel(0, VBDA0);
        __raw_writel(0, VBDA1);
    }
}

static void vbc_buffer_clear_all(void)
{
    vbc_access_buf(true);
    vbc_buffer_clear(1); // clear data buffer 1
    vbc_buffer_clear(0); // clear data buffer 0
    vbc_access_buf(false);
}

static int vbc_codec_mute(void)
{
    return vbc_reg_VBCR1_set(DAC_MUTE, 1); // mute
}

static int vbc_codec_unmute(void)
{
    return vbc_reg_VBCR1_set(DAC_MUTE, 0); // don't mute
}

static void vbc_set_ctrl2arm(void)
{
    // Enable VB DAC0/DAC1 (ARM-side)
    __raw_bits_or(ARM_VB_MCLKON|ARM_VB_ACC|ARM_VB_DA0ON|ARM_VB_ANAON|ARM_VB_DA1ON|ARM_VB_ADCON, SPRD_VBC_ALSA_CTRL2ARM_REG);
    msleep(1);
}

#if defined(CONFIG_ARCH_SC8800G) || \
    defined(CONFIG_ARCH_SC8810)
u16 __raw_adi_read(u32 addr)
{
    u32 adi_data, phy_addr;
    check_range(addr);
    phy_addr = addr - SPRD_ADI_BASE + SPRD_ADI_PHYS;
    //enter_critical();
    __raw_writel(phy_addr, ADI_ARM_RD_CMD);
    do {
        adi_data = __raw_readl(ADI_RD_DATA);
    } while (adi_data & (1 << 31));
    //rd_data high part should be the address of the last read operation
    if ((adi_data & 0xFFFF0000) != ((phy_addr) <<16)) {
        panic("vbc ADI read error!");
    }
    //exit_critical();
//  lprintf("addr=0x%08x, phy=0x%08x, val=0x%04x\n", addr, phy_addr, adi_data & 0xffff);
    return adi_data & 0xffff;
}
EXPORT_SYMBOL_GPL(__raw_adi_read);

int __raw_adi_write(u32 data, u32 addr)
{
    check_range(addr);
    //enter_critical();
    while ((__raw_readl(ADI_FIFO_STS) & ADI_FIFO_EMPTY) == 0);
    __raw_writel(data, addr);
    //exit_critical();
    return 1;
}
EXPORT_SYMBOL_GPL(__raw_adi_write);

// adi_analogdie ANA_REG_MSK_OR
static void __raw_adi_and(u32 value, u32 addr)
{
    enter_critical();
    __raw_adi_write(__raw_adi_read(addr) & value, addr);
    exit_critical();
}

static void __raw_adi_or(u32 value, u32 addr)
{
    enter_critical();
    __raw_adi_write(__raw_adi_read(addr) | value, addr);
    exit_critical();
}
#endif

static void vbc_ldo_on(bool on)
{
    int do_on_off = 0;
    if (on) {
#ifdef CONFIG_ARCH_SC8800S
        if (!(__raw_readl(GR_LDO_CTL0) & (1 << 17))) {
            __raw_bits_or(1 << 17, GR_LDO_CTL0); // LDO_VB_PO
            do_on_off = 1;
        }
#elif defined(CONFIG_ARCH_SC8800G)
        if (!(adi_read(ANA_LDO_PD_CTL) & (1 << 15))) {
            __raw_adi_and(~(1 << 14), ANA_LDO_PD_CTL);
            __raw_adi_or(1 << 15, ANA_LDO_PD_CTL);
            do_on_off = 1;
        }
#elif defined(CONFIG_ARCH_SC8810)
        if (!(adi_read(ANA_LDO_PD_CTL0) & (1 << 15))) {
            __raw_adi_and(~(1 << 14), ANA_LDO_PD_CTL0);
            __raw_adi_or(1 << 15, ANA_LDO_PD_CTL0);
            do_on_off = 1;
        }
#endif
    } else {
#ifdef CONFIG_ARCH_SC8800S
        if ((__raw_readl(GR_LDO_CTL0) & (1 << 17))) {
            __raw_bits_and(~(1 << 17), GR_LDO_CTL0); // LDO_VB_PO
            do_on_off = 1;
        }
#elif defined(CONFIG_ARCH_SC8800G)
        if ((adi_read(ANA_LDO_PD_CTL) & (1 << 15))) {
            __raw_adi_and(~(1 << 15), ANA_LDO_PD_CTL);
            __raw_adi_or((1 << 14), ANA_LDO_PD_CTL);
            do_on_off = 1;
        }
#elif defined(CONFIG_ARCH_SC8810)
        if ((adi_read(ANA_LDO_PD_CTL0) & (1 << 15))) {
            __raw_adi_and(~(1 << 15), ANA_LDO_PD_CTL0);
            __raw_adi_or((1 << 14), ANA_LDO_PD_CTL0);
            do_on_off = 1;
        }
#endif
    }
    if (do_on_off) {
        printk("+++++++++++++ vbc set ldo to %s +++++++++++++\n", on ? "on" : "off");
    }
}

static void vbc_set_mainclk_to12M(void)
{
#ifdef CONFIG_ARCH_SC8800S
	u32 vpll_clk = CHIP_GetVPllClk();
	clk_12M_divider_set(vpll_clk);
    __raw_bits_or(1 << 17, GR_LDO_CTL0); // LDO_VB_PO
#elif defined(CONFIG_ARCH_SC8800G)
    __raw_bits_or(GEN0_VB_EN | GEN0_ADI_EN, GR_GEN0); // Enable voiceband module
    __raw_bits_or(1 << 29, GR_CLK_DLY); // CLK_ADI_EN_ARM and CLK_ADI_SEL=76.8MHZ
    __raw_adi_or(1 << 15, ANA_LDO_PD_CTL);
    __raw_adi_or(VBMCLK_ARM_EN | VBCTL_SEL, ANA_CLK_CTL);
#elif defined(CONFIG_ARCH_SC8810)
    __raw_bits_or(GEN0_VB_EN | GEN0_ADI_EN, GR_GEN0); // Enable voiceband module
    __raw_bits_or(1 << 29, GR_CLK_DLY); // CLK_ADI_EN_ARM and CLK_ADI_SEL=76.8MHZ
    __raw_adi_or(1 << 15, ANA_LDO_PD_CTL0);
	__raw_adi_or(0x1 | 0x2, ANA_AUDIO_CTRL);
#endif
}

static inline void vbc_ready2go(void)
{
    // Enable this bit then VBC interface starts working and software 
    // can receive voice band interrupt. 
    // Better set this bit after all other register bits are programmed. 
    vbc_reg_VBDABUFFDTA_set(VBENABLE, 1);
    msleep(2);
}

extern inline int vbc_amplifier_enabled(void);
extern inline void vbc_amplifier_enable(int enable, const char *prename);
static volatile int earpiece_muted = 1, headset_muted = 1, speaker_muted = 1;
void vbc_write_callback(unsigned int reg, unsigned int val)
{
    if (reg == VBCR1) {
       headset_muted  = !!(val & (1 << HP_DIS));
       earpiece_muted = !!(val & (1 << BTL_MUTE));
#if VBC_DYNAMIC_POWER_MANAGEMENT
       printk("[headset_muted =%d]\n"
              "[earpiece_muted=%d]\n"
              "[speaker_muted =%d]\n", headset_muted, earpiece_muted, speaker_muted);
#else
//      if (speaker_muted && earpiece_muted && headset_muted)
//          printk("---- vbc mute2 all pa ----\n");
//      else
//          printk("---- vbc unmute2 %s%s%spa ----\n", speaker_muted ? "":"Speaker ",
//                 earpiece_muted ? "":"Earpiece ", headset_muted ? "":"Headset ");
#endif
    }
}

void vbc_power_down(unsigned int value)
{
    int use_delay;
#if !VBC_DYNAMIC_POWER_MANAGEMENT
    int power_down_force = 0;
#endif
#if !VBC_DYNAMIC_POWER_MANAGEMENT
    if (value != -1) {
        power_down_force = !!(value & VBC_CODEC_POWER_DOWN_FORCE);
        if (power_down_force) value = -1;
        else value &= ~VBC_CODEC_POWER_DOWN_FORCE;
    } else value = -2;
#endif
    // printk("audio %s\n", __func__);
    {
        int do_sb_power = 0;
        use_delay = 1; // (in_irq() || irqs_disabled()) ? 0 : 1; // !headset_muted;
        // int VBCGR1_value;
        if ((vbc_reg_read(VBPMR1, SB_ADC, 1)
             && (value == SNDRV_PCM_STREAM_PLAYBACK && !vbc_reg_read(VBPMR1, SB_DAC, 1))) ||
            (vbc_reg_read(VBPMR1, SB_DAC, 1)
             && (value == SNDRV_PCM_STREAM_CAPTURE && !vbc_reg_read(VBPMR1, SB_ADC, 1))) ||
            (vbc_reg_read(VBPMR1, SB_ADC, 1) && vbc_reg_read(VBPMR1, SB_DAC, 1))) {
            do_sb_power = 1;
        }
#if !VBC_DYNAMIC_POWER_MANAGEMENT
        if (power_down_force) {
#endif
        if ((value == -1) ||
            (value == SNDRV_PCM_STREAM_PLAYBACK &&
            (!vbc_reg_read(VBPMR1, SB_DAC, 1) ||
            !earpiece_muted || !headset_muted || !speaker_muted))) {
#if !VBC_DYNAMIC_POWER_MANAGEMENT
            printk("---- vbc do power down ----\n");
#endif
            // VBCGR1_value = vbc_reg_write(VBCGR1, 0, 0xff, 0xff); // DAC Gain
            if (use_delay) msleep(100); // avoid quick switch from power on to off
            /*
            earpiece_muted= vbc_reg_read(VBCR1, BTL_MUTE, 1);
            headset_muted = vbc_reg_read(VBCR1, HP_DIS, 1);
            speaker_muted = vbc_amplifier_enabled();
            */
            vbc_reg_VBCR1_set(BTL_MUTE, 1); // Mute earpiece
            vbc_reg_VBCR1_set(HP_DIS, 1); // Mute headphone
            vbc_amplifier_enable(false, "vbc_power_down playback"); // Mute speaker
            vbc_codec_mute();
            if (use_delay) msleep(50);
            vbc_reg_VBCR1_set(DACSEL, 0); // not route DAC to mixer

            vbc_reg_VBPMR1_set(SB_OUT, 1); // Power down DAC OUT
            vbc_reg_VBCR1_set(MONO, 1); // mono DAC channel
            vbc_reg_VBPMR1_set(SB_BTL, 1); // power down earphone
            if (use_delay) msleep(100);

            vbc_reg_VBPMR1_set(SB_DAC, 1); // Power down DAC
            vbc_reg_VBPMR1_set(SB_LOUT, 1);
            vbc_reg_VBPMR1_set(SB_MIX, 1);
            // msleep(50);
            // vbc_reg_write(VBCGR1, 0, VBCGR1_value, 0xff); // DAC Gain
        }
#if !VBC_DYNAMIC_POWER_MANAGEMENT
        } else {
            if (value == SNDRV_PCM_STREAM_PLAYBACK) {
                vbc_reg_VBCR1_set(BTL_MUTE, 1); // Mute earpiece
                vbc_reg_VBCR1_set(HP_DIS, 1); // Mute headphone
                vbc_amplifier_enable(false, "vbc_power_down playback"); // Mute speaker
                vbc_codec_mute();
                printk("---- vbc mute all pa ----\n");
            }
            do_sb_power = 0;
        }
#endif
        if ((value == -1) ||
            (value == SNDRV_PCM_STREAM_CAPTURE &&
            !vbc_reg_read(VBPMR1, SB_ADC, 1))) {
            printk("vbc_power_down capture\n");
            vbc_reg_VBPMR1_set(SB_ADC, 1); // Power down ADC
            vbc_reg_VBCR1_set(SB_MICBIAS, 1); // power down mic
        }
        if ((value == -1) ||
            do_sb_power)/* vbc_reg_read(VBPMR1, SB_ADC, 1) && vbc_reg_read(VBPMR1, SB_DAC, 1) */ {
            vbc_reg_VBPMR2_set(SB_SLEEP, 1); // SB enter sleep mode
            vbc_reg_VBPMR2_set(SB, 1); // Power down sb
            if (use_delay) msleep(100); // avoid quick switch from power off to on
            vbc_ldo_on(0);
            printk("....................... vbc full power down [%d]-%d .......................\n", use_delay, in_irq() || irqs_disabled());
        }
    }
}
EXPORT_SYMBOL_GPL(vbc_power_down);

#if !VBC_DYNAMIC_POWER_MANAGEMENT
void vbc_power_on_playback(bool ldo)
{
    // Following code has risk [luther.ge]
    ldo = ldo;
#if 0
    if (!earpiece_muted) vbc_reg_VBCR1_set(BTL_MUTE, 0); // unMute earpiece
    if (!headset_muted) vbc_reg_VBCR1_set(HP_DIS, 0); // unMute headphone
    if (!speaker_muted) vbc_amplifier_enable(true, "vbc_power_on playback"); // unMute speaker
#endif
    //  if (speaker_muted && earpiece_muted && headset_muted)
    //      printk("---- vbc mute all pa ----\n");
    //  else
    //      printk("---- vbc unmute %s%s%spa ----\n", speaker_muted ? "":"Speaker ",
    //             earpiece_muted ? "":"Earpiece ",headset_muted ? "":"Headset ");
}
#endif

void vbc_power_on_capture(bool ldo)
{
    // Following code has risk [luther.ge]
    if (vbc_reg_read(VBPMR1, SB_ADC, 1)) {
        if (ldo) vbc_ldo_on(1);
        printk("vbc_power_on capture\n");
        vbc_reg_VBPMR2_set(SB, 0); // Power on sb
        vbc_reg_VBPMR2_set(SB_SLEEP, 0); // SB quit sleep mode

        vbc_reg_VBPMR2_set(GIM, 1); // 20db gain mic amplifier
        vbc_reg_write(VBCGR10, 4, 0xf, 0xf); // set GI to max
        vbc_reg_VBCR1_set(SB_MICBIAS, 0); // power on mic
        vbc_reg_VBPMR1_set(SB_ADC, 0); // Power on ADC
    }
}

void vbc_power_on(unsigned int value)
{
    int use_delay;
    int mute_dac;
    int power_on_force;
    mute_dac = !!(value & VBC_CODEC_POWER_ON_OUT_MUTE_DAC);
    value &= ~VBC_CODEC_POWER_ON_OUT_MUTE_DAC;
    power_on_force = !!(value & VBC_CODEC_POWER_ON_FORCE);
    value &= ~VBC_CODEC_POWER_ON_FORCE;
    vbc_ldo_on(1);
    // printk("audio %s\n", __func__);
    {
        use_delay = 1; // (in_irq() || irqs_disabled()) ? 0 : 1;; // !headset_muted;
        if (power_on_force ||
            (value == SNDRV_PCM_STREAM_PLAYBACK &&
            (vbc_reg_read(VBPMR1, SB_DAC, 1) ||
             vbc_reg_read(VBPMR1, SB_LOUT, 1)||
             vbc_reg_read(VBPMR1, SB_OUT, 1) ||
             vbc_reg_read(VBPMR1, SB_MIX, 1) ||
             vbc_reg_read(VBPMR2, SB, 1)     ||
             vbc_reg_read(VBPMR2, SB_SLEEP, 1)))) {
#if VBC_DYNAMIC_POWER_MANAGEMENT
            int forced = 0;
#endif
            // int VBCGR1_value;
#if !VBC_DYNAMIC_POWER_MANAGEMENT
            printk("---- vbc do power on ----\n");
#endif
            // VBCGR1_value = vbc_reg_write(VBCGR1, 0, 0xff, 0xff); // DAC Gain
            vbc_reg_VBPMR2_set(SB, 0); // Power on sb
            vbc_reg_VBPMR2_set(SB_SLEEP, 0); // SB quit sleep mode

            vbc_codec_mute();
            /* earpiece_muted = */ vbc_reg_VBCR1_set(BTL_MUTE, 1); // Mute earpiece
            /* headset_muted =  */ vbc_reg_VBCR1_set(HP_DIS, 1); // Mute headphone
            /* speaker_muted =  */ vbc_amplifier_enable(false, "vbc_power_on playback"); // Mute speaker
            if (use_delay) msleep(50);

            vbc_reg_VBCR1_set(DACSEL, 1); // route DAC to mixer
            vbc_reg_VBPMR1_set(SB_DAC, 0); // Power on DAC
            if (use_delay) msleep(50);
            vbc_reg_VBPMR1_set(SB_LOUT, 0);
            if (use_delay) msleep(50);
            vbc_reg_VBPMR1_set(SB_MIX, 0);
            if (use_delay) msleep(50);

            vbc_reg_VBPMR1_set(SB_OUT, 0); // Power on DAC OUT
            if (use_delay) msleep(50);
            vbc_reg_VBCR1_set(MONO, 0); // stereo DAC left & right channel
            vbc_reg_VBPMR1_set(SB_BTL, 0); // power on earphone
            if (use_delay) msleep(100);

            if (!mute_dac) vbc_codec_unmute();
#if VBC_DYNAMIC_POWER_MANAGEMENT
            if (!earpiece_muted || forced) vbc_reg_VBCR1_set(BTL_MUTE, 0); // unMute earpiece
            if (!headset_muted || forced) vbc_reg_VBCR1_set(HP_DIS, 0); // unMute headphone
            if (!speaker_muted || forced) vbc_amplifier_enable(true, "vbc_power_on playback"); // unMute speaker
            printk("....................... vbc power on playback [%d]-%d .......................\n", use_delay, in_irq() || irqs_disabled());
            // if (!use_delay) dump_stack();
#endif
            // vbc_reg_write(VBCGR1, 0, VBCGR1_value, 0xff); // DAC Gain
        }
#if !VBC_DYNAMIC_POWER_MANAGEMENT
        else {
            if (value == SNDRV_PCM_STREAM_PLAYBACK)
                vbc_power_on_playback(0);
        }
#endif
        if (value == SNDRV_PCM_STREAM_CAPTURE)
            vbc_power_on_capture(0);
    }
}
EXPORT_SYMBOL_GPL(vbc_power_on);

static inline int mode_incall(void)
{
    return !(__raw_readl(SPRD_VBC_ALSA_CTRL2ARM_REG) & ARM_VB_ACC);
}

static int vbc_reset(struct snd_soc_codec *codec, int poweron, int check_incall)
{
    int ret = 0;
    printk("vbc reset start...\n");
    // 1. dial phone number
    // 2. modem will set DSP control audio codec
    // 3. DSP control audio codec
    // 4. in call
    // 5. AT+ATH to quit call
    // 6. DSP will release audio chain
    // 7. modem will set ARM control audio codec
    // 8. android will reset audio codec to ARM & setting android alsa himself needed audio parameters
    //
    // The problem occures in step 7 & 8, if 8 first occures, pop sound will be created, and
    // alsa DMA can't be work, AudioFlinger will can't obtainBuffer from alsa driver [luther.ge]
    if (check_incall) {
        #define VBC_DSP_WAITING_MAX_COUNT   20
        int try_max = 0;
        // vbc_amplifier_enable(false, "vbc_init"); // Mute Speaker
        // fix above problem
        while (mode_incall() && try_max++ < VBC_DSP_WAITING_MAX_COUNT) {
            printk("vbc waiting DSP release audio codec ......\n");
            msleep(100);
        }
        if (try_max >= VBC_DSP_WAITING_MAX_COUNT) {
            printk("vbc say God God God God God God God oh my !!!!\n");
            ret = -1;
        }
        printk("vbc waiting modem stable setting audio codec ...... start ......\n");
        msleep(200);
        printk("vbc waiting modem stable setting audio codec ...... done ......\n");
        // atomic_read
    }
    vbc_set_mainclk_to12M();
    vbc_set_ctrl2arm();
    if (poweron) vbc_power_on((SNDRV_PCM_STREAM_LAST+1) | VBC_CODEC_POWER_ON_FORCE);
#if 0
    vbc_codec_mute();
#endif
    vbc_ready2go();

    vbc_set_AD_DA_fifo_frame_num(VBC_FIFO_FRAME_NUM, VBC_FIFO_FRAME_NUM);

    vbc_buffer_clear_all(); // must have this func, or first play will have noise

    // IIS timeing : High(right channel) for both DA1/AD1, Low(left channel) for both DA0/AD0
    // Active level of left/right channel for both ADC and DAC channel 
    vbc_set_VBADBUFFDTA_set(VBIIS_LRCK, 0);

    vbc_reg_VBAICR_set(VBCAICR_MODE_ADC_I2S    |
                       VBCAICR_MODE_DAC_I2S    |
                       VBCAICR_MODE_ADC_SERIAL |
                       VBCAICR_MODE_DAC_SERIAL);

    vbc_reg_VBCR2_set(DAC_ADWL, DAC_DATA_WIDTH_16_bit); // DAC data sample depth 16bits
    vbc_reg_VBCR2_set(ADC_ADWL, ADC_DATA_WIDTH_16_bit); // ADC data sample depth 16bits
    vbc_reg_VBCR2_set(MICSEL, MICROPHONE1); // route microphone 1 to ADC module

    vbc_reg_write(VBCCR2, 4, VBC_RATE_8000, 0xf); // 8K sample DAC
    vbc_reg_write(VBCCR2, 0, VBC_RATE_8000, 0xf); // 8K sample ADC

    vbc_reg_write(VBCGR2, 0, 0, 0x1f); // FM Gain to max by default
    vbc_reg_write(VBCGR3, 0, 0, 0x1f);
//    vbc_reg_write(VBCGR1, 0, 0x00, 0xff); // DAC Gain
//    vbc_reg_write(VBCGR8, 0, 0x00, 0x1f);
//    vbc_reg_write(VBCGR9, 0, 0x00, 0x1f);
#if 0
    msleep(1);

    vbc_reg_VBPMR2_set(SB, 0); // Power on sb
    // Deleay between SB and SB_SLEEP, for stablizing the speaker output wave
    msleep(1);

    vbc_reg_VBPMR2_set(SB_SLEEP, 0); // SB quit sleep mode
    msleep(1);
    vbc_reg_VBCR1_set(SB_MICBIAS, 0); // power on mic
#else
    // vbc_power_on(SNDRV_PCM_STREAM_PLAYBACK);
#endif
    vbc_reg_VBPMR2_set(GIM, 1); // 20db gain mic amplifier
    vbc_reg_write(VBCGR10, 4, 0xf, 0xf); // set GI to max
    vbc_reg_VBPMR1_set(SB_ADC, 1); // Power down ADC
    vbc_reg_VBCR1_set(SB_MICBIAS, 1); // power down mic
#if 0
    // vbc_reg_write(VBPMR1, 1, 0x02, 0x7f); // power on all units, except SB_BTL
    vbc_reg_VBPMR1_set(SB_DAC, 0); // Power on DAC
//  vbc_reg_VBPMR1_set(SB_ADC, 0); // Power on ADC
    vbc_reg_VBPMR1_set(SB_ADC, 1); // Power down ADC
    vbc_reg_VBPMR1_set(SB_MIX, 0);
    vbc_reg_VBPMR1_set(SB_LOUT, 0);
    vbc_reg_VBPMR1_set(SB_OUT, 0); // Power on DAC OUT
    msleep(5);

    // mono use DA0 left channel
#ifdef CONFIG_ARCH_SC8800S
    vbc_reg_VBCR1_set(MONO, 0); // stereo DAC left & right channel
    vbc_reg_VBCR1_set(HP_DIS, 1); // not route mixer audio data to headphone outputs
    vbc_reg_VBCR1_set(BYPASS, 0); // Analog bypass not route to mixer
    vbc_reg_VBCR1_set(DACSEL, 1); // route DAC to mixer
    vbc_reg_VBCR1_set(BTL_MUTE, 1); // Mute earpiece
#elif defined(CONFIG_ARCH_SC8800G) || \
      defined(CONFIG_ARCH_SC8810)
    vbc_reg_VBCR1_set(MONO, 0); // stereo DAC left & right channel
    vbc_reg_VBCR1_set(HP_DIS, 0); // route mixer audio data to headphone outputs
    vbc_reg_VBCR1_set(BYPASS, 0); // Analog bypass not route to mixer
    vbc_reg_VBCR1_set(DACSEL, 1); // route DAC to mixer
    vbc_reg_VBCR1_set(BTL_MUTE, 0); // not Mute earpiece

    vbc_reg_VBPMR1_set(SB_BTL, 0); // power on earphone
    vbc_reg_VBPMR1_set(SB_LIN, 0);

    vbc_reg_write(DAHPCTL, 8, 1, 1); // headphone 24bits output to sound more appealing
#endif
    vbc_codec_unmute(); // don't mute
#endif
    printk("vbc reset finish...\n");
    return ret;
}

static int vbc_soft_ctrl(struct snd_soc_codec *codec, unsigned int reg, unsigned int value, int dir)
{
    int ret = 0;
    // printk("vbc_soft_ctrl value[%d]=%04x\n", dir, reg);
    switch (reg) {
        case VBC_CODEC_RESET:
            // After phone call, we should reset all codec related registers
            // because in phone call state dsp will control codec, and set all registers
            // so we should reset all registers again in linux side,
            // otherwise android media will not work [luther.ge]
            // if (val & (1 << VBC_CODEC_SOFT_RESET))
            if (dir == 0) return 0; // dir 0 for read, we always return 0, so every set 1 value can reach here.
            // speaker_muted = true;
#if VBC_DYNAMIC_POWER_MANAGEMENT
            ret = vbc_reset(codec, 0, 1);
            vbc_power_down(-1);
#else
            ret = vbc_reset(codec, 1, 1);
#endif
            // ret = vbc_reset(codec);
            if (!earpiece_muted) vbc_reg_VBCR1_set(BTL_MUTE, 0); // unMute earpiece
            if (!headset_muted) vbc_reg_VBCR1_set(HP_DIS, 0); // unMute headphone            
            if (!speaker_muted) vbc_amplifier_enable(true, "vbc_soft_ctrl"); // unMute speaker
            return ret < 0 ? -2 : ret;
        case VBC_CODEC_POWER:
            if (dir == 0) return 0; // dir 0 for read, we always return 0, so every set 1 value can reach here.
            printk("vbc power to 0x%08x\n", value);
            if (value & VBC_CODEC_POWER_ON_OUT) {
                vbc_power_on(SNDRV_PCM_STREAM_PLAYBACK | (value & VBC_CODEC_POWER_ON_OUT_MUTE_DAC));
            }
            if (value & VBC_CODEC_POWER_ON_IN) {
                vbc_power_on(SNDRV_PCM_STREAM_CAPTURE);
            }
            if (value & VBC_CODEC_POWER_OFF_OUT) {
                vbc_power_down(SNDRV_PCM_STREAM_PLAYBACK);
            }
            if (value & VBC_CODEC_POWER_OFF_IN) {
                vbc_power_down(SNDRV_PCM_STREAM_CAPTURE);
            }
            return value;
        case VBC_CODEC_POWER2:
            if (dir == 0) return 0; // dir 0 for read, we always return 0, so every set 1 value can reach here.
            printk("vbc power2 to 0x%08x\n", value);
            vbc_power_down((SNDRV_PCM_STREAM_LAST+1) | VBC_CODEC_POWER_DOWN_FORCE);
            return value;
        case VBC_CODEC_DSP:
            return mode_incall();
        case VBC_CODEC_SPEAKER_PA:
            if (dir) {
                vbc_amplifier_enable(value & 0x01, "vbc_soft_ctrl2");
            }
            value = vbc_amplifier_enabled();
            speaker_muted = value ? 0:1;
            return value;
        default: return -1;
    }
}

static unsigned int vbc_read(struct snd_soc_codec *codec, unsigned int reg)
{
    int ret = vbc_soft_ctrl(codec, reg, 0, 0);
    if (ret != -1) return ret;
    // Because snd_soc_update_bits reg is 16 bits short type, so muse do following convert
    reg |= ARM_VB_BASE2;
#ifdef CONFIG_ARCH_SC8800S
    return __raw_readl(reg);
#elif defined(CONFIG_ARCH_SC8800G) || \
      defined(CONFIG_ARCH_SC8810)
    return adi_read(reg);
#endif
}

static int vbc_write(struct snd_soc_codec *codec, unsigned int reg, unsigned int val)
{
    int ret = vbc_soft_ctrl(codec, reg, val, 1);
    if (ret != -1) return ret;
    // Because snd_soc_update_bits reg is 16 bits short type, so muse do following convert
    reg |= ARM_VB_BASE2;
    vbc_write_callback(reg, val);
#ifdef CONFIG_ARCH_SC8800S
    __raw_writel(val, reg);
#elif defined(CONFIG_ARCH_SC8800G) || \
      defined(CONFIG_ARCH_SC8810)
    adi_write(val, reg);
#endif
    return 0;
}

static void vbc_dma_control(int chs, bool on)
{
    if (chs & AUDIO_VBDA0)
        vbc_reg_VBDABUFFDTA_set(VBDA0DMA_EN, on); // DMA write DAC0 data buffer enable/disable
    if (chs & AUDIO_VBDA1)
        vbc_reg_VBDABUFFDTA_set(VBDA1DMA_EN, on); // DMA write DAC1 data buffer enable/disable
    if (chs & AUDIO_VBAD0)
        vbc_reg_VBDABUFFDTA_set(VBAD0DMA_EN, on); // DMA read ADC0 data buffer enable/disable
    if (chs & AUDIO_VBAD1)
        vbc_reg_VBDABUFFDTA_set(VBAD1DMA_EN, on); // DAM read ADC1 data buffer enable/disable
}

inline void vbc_dma_start(struct snd_pcm_substream *substream)
{
    vbc_dma_control(audio_playback_capture_channel(substream), 1);
}
EXPORT_SYMBOL_GPL(vbc_dma_start);

static inline void vbc_dma_stop(struct snd_pcm_substream *substream)
{
    vbc_dma_control(audio_playback_capture_channel(substream), 0);
}

void flush_vbc_cache(struct snd_pcm_substream *substream)
{
#if 1
    struct snd_pcm_runtime *runtime = substream->runtime;
    // we could not stop vbc_dma_buffer immediately, because audio data still in cache
    if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
        return;
//  vbc_codec_mute();
    /* clear dma cache buffer */
    memset((void*)runtime->dma_area, GAP_DATA_CHAR, runtime->dma_bytes);
    printk("vbc flush cache buffer...\n");
    if (cpu_codec_dma_chain_operate_ready(substream)) {
        vbc_dma_start(substream); // we must restart dma
        start_cpu_dma(substream);
        /* must wait all dma cache chain filled by 0 data */
//      lprintf("Filling all dma chain cache audio data to 0\n");
        msleep(5);// msleep(20);
//      lprintf("done!\n");
        stop_cpu_dma(substream);
        vbc_dma_stop(substream);
    }
#endif
}
EXPORT_SYMBOL_GPL(flush_vbc_cache);

static int vbc_startup(struct snd_pcm_substream *substream,
    struct snd_soc_dai *dai)
{
    if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
        vbc_buffer_clear_all();
    }
//  vbc_codec_unmute();
    return 0;
}

static int vbc_prepare(struct snd_pcm_substream *substream,
    struct snd_soc_dai *dai)
{
    // printk("vbc_prepare......[start]\n");
    // From __pxa2xx_pcm_prepare clear DMA or power down
    if (cpu_codec_dma_chain_operate_ready(substream)) {
    //    printk("--------- vbc_prepare->vbc_dma_stop ---------\n");
        vbc_dma_stop(substream);
    } else {
    //    printk("--------- vbc_prepare->nop ---------\n");
    }
    // printk("vbc_prepare......[done]\n");
    return 0;
}

static void vbc_shutdown(struct snd_pcm_substream *substream,
    struct snd_soc_dai *dai)
{
    printk("vbc_shutdown......\n");
    // vbc_power_down(substream->stream);
#if 0
    if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
        vbc_amplifier_enable(false, "sprdphone_shutdown");
#endif
}

#if POWER_OFF_ON_STANDBY
static int vbc_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level)
{
    return 0;
}
#endif

static int vbc_set_dai_tristate(struct snd_soc_dai *codec_dai, int tristate)
{
    return 0;
}

static int vbc_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
    return 0;
}

static int vbc_set_dai_clkdiv(struct snd_soc_dai *codec_dai, int div_id, int div)
{
    return 0;
}

static int vbc_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *dai)
{
	int ret = 0;
#if VBC_NOSIE_CURRENT_SOUND_HARDWARE_BUG_FIX
    static int dac_muted = 0;
#endif

	switch (cmd) {
        case SNDRV_PCM_TRIGGER_START:
#if VBC_NOSIE_CURRENT_SOUND_HARDWARE_BUG_FIX
            if (!dac_muted && substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
                printk("vbc hardware nosie current unmute dac now!\n");
                vbc_codec_unmute();
            }
#endif
            #if VBC_DYNAMIC_POWER_MANAGEMENT
            vbc_power_on(substream->stream);
            #else
            if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
                vbc_power_on_playback(1);
            else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
                vbc_power_on_capture(1);
            #endif
#if !VBC_NOSIE_CURRENT_SOUND_HARDWARE_BUG_FIX
            vbc_dma_start(substream);
#endif
            break;
        case SNDRV_PCM_TRIGGER_STOP:
            // vbc_power_down(substream->stream);
        case SNDRV_PCM_TRIGGER_SUSPEND:
        case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
#if VBC_NOSIE_CURRENT_SOUND_HARDWARE_BUG_FIX
            if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
                // must mute here, otherwise noise current sound will appear
                dac_muted = !!vbc_reg_read(VBCR1, DAC_MUTE, 1);
                if (!dac_muted) {
                    vbc_codec_mute();
                    printk("vbc hardware nosie current mute dac now!\n");
                }
            }
#endif
            vbc_dma_stop(substream); // Stop DMA transfer
            break;
        case SNDRV_PCM_TRIGGER_RESUME:
        case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
#if VBC_NOSIE_CURRENT_SOUND_HARDWARE_BUG_FIX
            if (!dac_muted && substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
                vbc_codec_unmute();
            }
#endif
            vbc_dma_start(substream);
            break;
        default:
            ret = -EINVAL;
	}

	return ret;
}

static int vbc_pcm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
    int idx;

    if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
        idx = 1;
    else idx = 0;

    switch (params_format(params)) {
        case SNDRV_PCM_FORMAT_S16_LE:
        case SNDRV_PCM_FORMAT_S16_BE:
        case SNDRV_PCM_FORMAT_U16_LE:
        case SNDRV_PCM_FORMAT_U16_BE: break;
        default: 
            printk(KERN_EMERG "vbc codec only supports format 16bits");
            break;
    }
	cur_sample_rate = params_rate(params);
    switch (params_rate(params)) {
        case  8000: vbc_reg_write(VBCCR2, idx * 4, VBC_RATE_8000 , 0xf); break;
        case 11025: vbc_reg_write(VBCCR2, idx * 4, VBC_RATE_11025, 0xf); break;
        case 16000: vbc_reg_write(VBCCR2, idx * 4, VBC_RATE_16000, 0xf); break;
        case 22050: vbc_reg_write(VBCCR2, idx * 4, VBC_RATE_22050, 0xf); break;
        case 32000: vbc_reg_write(VBCCR2, idx * 4, VBC_RATE_32000, 0xf); break;
        case 44100: vbc_reg_write(VBCCR2, idx * 4, VBC_RATE_44100, 0xf); break;
        case 48000: vbc_reg_write(VBCCR2, idx * 4, VBC_RATE_48000, 0xf); break;
        case 96000: vbc_reg_write(VBCCR2, idx * 4, VBC_RATE_96000, 0xf); break;
        default:
            printk(KERN_EMERG "vbc codec not supports rate %d\n", params_rate(params));
			cur_sample_rate = 44100;
            break;
    }

    printk("vbc Sample Rate is [%d]\n", params_rate(params));

    return 0;
}

int32_t get_cur_sample_rate(void)
{
	return cur_sample_rate;
}

int vbc_hw_free(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
    flush_vbc_cache(substream);
    return 0;
}

static struct snd_soc_dai_ops vbc_dai_ops = {
    .startup    = vbc_startup,
    .prepare    = vbc_prepare,
    .trigger    = vbc_trigger,
	.hw_params  = vbc_pcm_hw_params,
    .hw_free    = vbc_hw_free,
    .shutdown   = vbc_shutdown,
	.set_clkdiv = vbc_set_dai_clkdiv,
	.set_fmt    = vbc_set_dai_fmt,
	.set_tristate = vbc_set_dai_tristate,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
static struct early_suspend early_suspend;
static void learly_suspend(struct early_suspend *es)
{
    // vbc_power_down();
}

static void learly_resume(struct early_suspend *es)
{
    // vbc_power_on();
}

static void android_pm_init(void)
{
    early_suspend.suspend = learly_suspend;
    early_suspend.resume = learly_resume;
    early_suspend.level = INT_MAX;
    register_early_suspend(&early_suspend);
}

static void android_pm_exit(void)
{
    unregister_early_suspend(&early_suspend);
}
#else
static void android_pm_init(void) {}
static void android_pm_exit(void) {}
#endif

#if 1
#include <mach/pm_devices.h>
static struct sprd_pm_suspend sprd_suspend;
static int lsprd_suspend(struct device *pdev, pm_message_t state)
{
    // vbc_power_down();
    return 0;
}

static int lsprd_resume(struct device *pdev)
{
    // vbc_power_on();
    return 0;
}

static void android_sprd_pm_init(void)
{
    sprd_suspend.suspend = lsprd_suspend;
    sprd_suspend.resume  = lsprd_resume;
    sprd_suspend.level   = INT_MAX;
    register_sprd_pm_suspend(&sprd_suspend);
}

static void android_sprd_pm_exit(void)
{
    unregister_sprd_pm_suspend(&sprd_suspend);
}
#else
static void android_sprd_pm_init(void) {}
static void android_sprd_pm_exit(void) {}
#endif

#ifdef CONFIG_PM
int vbc_suspend(struct platform_device *pdev, pm_message_t state)
{
    struct snd_soc_device *socdev = platform_get_drvdata(pdev);
    struct snd_soc_codec *codec = socdev->card->codec;
    mutex_lock(&codec->mutex);
    printk("vbc xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n");
    vbc_print_regs(0);
    vbc_power_down((SNDRV_PCM_STREAM_LAST+1) | VBC_CODEC_POWER_DOWN_FORCE);
    vbc_print_regs(0);
    mutex_unlock(&codec->mutex);
    return 0;
}

int vbc_resume(struct platform_device *pdev)
{
    struct snd_soc_device *socdev = platform_get_drvdata(pdev);
    struct snd_soc_codec *codec = socdev->card->codec;
    mutex_lock(&codec->mutex);
    printk("vbc yyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyy\n");
    vbc_print_regs(1);
#if 1
    vbc_reset(codec, 1, 0);
#else
    if (codec) vbc_reset(codec, 0, 0);
    vbc_power_on((SNDRV_PCM_STREAM_LAST+1) | VBC_CODEC_POWER_ON_FORCE);
#endif
    mutex_unlock(&codec->mutex);
    vbc_print_regs(1);
    return 0;
}
#else
#define vbc_suspend NULL
#define vbc_resume  NULL
#endif
#if defined(CONFIG_ARCH_SC8800G)
#define local_cpu_pa_control(x) \
do { \
    if (x) { \
        /* ADI_Analogdie_reg_write(ANA_PA_CTL, 0x1aa9); //classAb */ \
        ADI_Analogdie_reg_write(ANA_PA_CTL, 0x5A5A); /* classD */ \
    } else { \
        ADI_Analogdie_reg_write(ANA_PA_CTL, 0x1555); \
    } \
} while (0)
#elif defined(CONFIG_ARCH_SC8810)
#define local_cpu_pa_control(x) \
do { \
    if (x) { \	
        ADI_Analogdie_reg_write(ANA_AUDIO_PA_CTRL0, (0x1101 |( (cur_internal_pa_gain <<4) & 0x00F0)));  \
        ADI_Analogdie_reg_write(ANA_AUDIO_PA_CTRL1, 0x1e41); \
    } else { \    
        ADI_Analogdie_reg_write(ANA_AUDIO_PA_CTRL0, 0x182); \
        ADI_Analogdie_reg_write(ANA_AUDIO_PA_CTRL1, 0x1242); \
    } \
} while (0)
#endif

#if     defined(CONFIG_ARCH_SC8800S)             || \
        defined(CONFIG_MACH_SP6810A)
#if     defined(CONFIG_ARCH_SC8800S)
static u32 speaker_gpio = 102; // mfp_to_gpio(MFP_CFG_TO_PIN(gpio_amplifier));
#elif   defined(CONFIG_MACH_SP6810A)
static u32 speaker_gpio = 96;  // GPIO_PROD_SPEAKER_PA_EN_ID
#endif
static inline void local_amplifier_init(void)
{
    if (gpio_request(speaker_gpio, "speaker amplifier")) {
        printk(KERN_ERR "speaker amplifier gpio request fail!\n");
    }
}

static inline void local_amplifier_enable(int enable)
{
    local_cpu_pa_control(enable);
    gpio_direction_output(speaker_gpio, !!enable); msleep(1);
}

static inline int local_amplifier_enabled(void)
{
    if (gpio_get_value(speaker_gpio)) {
        return 1;
    } else {
        return 0;
    }
}
#else

#if defined(CONFIG_MACH_SP8805GA)
extern int sprd_local_audio_pa_mode_detect_gpio;
#elif defined(CONFIG_MACH_SP6820A)
static uint32_t SPRD_BOARD_VERSION;
static int32_t speaker_gpio = -1;
static int32_t speaker_gpio_enabled_level;
#endif
static int sprd_local_audio_pa_mode = 0; // 1 -- internal PA; 0 -- outside PA

static inline void local_amplifier_init(void)
{
#if defined(CONFIG_MACH_SP8805GA)
    if (gpio_get_value(sprd_local_audio_pa_mode_detect_gpio)) {
        sprd_local_audio_pa_mode = 0;
    } else sprd_local_audio_pa_mode = 1;
#elif defined(CONFIG_MACH_SP6820A)
    SPRD_BOARD_VERSION = system_rev & 0xffff;
    if (SPRD_BOARD_VERSION == 0x100) {
        sprd_local_audio_pa_mode = 1;
    } else if (SPRD_BOARD_VERSION == 0x101) {
        sprd_local_audio_pa_mode = 0;
        speaker_gpio = 91;
        speaker_gpio_enabled_level = 1;
        if (gpio_request(speaker_gpio, "speaker amplifier")) {
            printk(KERN_ERR "speaker amplifier gpio request fail!\n");
        }
    }
#else    //8810 is controlled by audio_para
//    sprd_local_audio_pa_mode = 1;
#endif
    printk("vbc sprd_local_audio_pa_mode = %d\n", sprd_local_audio_pa_mode);
}

static inline void local_amplifier_enable(int enable)
{
    if (sprd_local_audio_pa_mode) {
        local_cpu_pa_control(enable);
    } else {
#if defined(CONFIG_MACH_SP6820A)
        if (speaker_gpio >= 0)
            gpio_direction_output(speaker_gpio, enable ? speaker_gpio_enabled_level:!speaker_gpio_enabled_level);
        else printk("board 0x%03x not support outside PA\n", SPRD_BOARD_VERSION);
#else
        printk("vbc not support outside PA\n");
#endif
    }
}

static inline int local_amplifier_enabled(void)
{
    if (sprd_local_audio_pa_mode) {
#if defined(CONFIG_ARCH_SC8800G)
        u32 value = ADI_Analogdie_reg_read(ANA_PA_CTL);
        switch (value) {
            case 0x5A5A: return 1;
            default : return 0;
        }
#elif defined(CONFIG_ARCH_SC8810)
        u32 value = ADI_Analogdie_reg_read(ANA_AUDIO_PA_CTRL0);
		value &= 0x1101;
        switch (value) {
            case 0x1101: return 1;
            default : return 0;
        }
#endif
    } else {
#if defined(CONFIG_MACH_SP6820A)
        if (speaker_gpio >= 0) {
            if (!!gpio_get_value(speaker_gpio) == speaker_gpio_enabled_level)
                return 1;
            else return 0;
        } else printk("board 0x%03x not support outside PA\n", SPRD_BOARD_VERSION);
#else
        printk("vbc not support outside PA\n");
#endif
        return 0;
    }
}
#endif
inline void vbc_amplifier_enable(int enable, const char *prename)
{
#if VBC_DYNAMIC_POWER_MANAGEMENT
    printk("vbc %s ==> trun %s PA\n", prename, enable ? "on":"off");
    printk("[headset_muted =%d]\n"
           "[earpiece_muted=%d]\n"
           "[speaker_muted =%d]\n", headset_muted, earpiece_muted, speaker_muted);
#else
//  if (speaker_muted && earpiece_muted && headset_muted)
//      printk("---- vbc mute3 all pa ----\n");
//  else printk("---- vbc unmute3 %s%s%spa ----\n", speaker_muted ? "":"Speaker ",
//              earpiece_muted ? "":"Earpiece ", headset_muted ? "":"Headset ");
#endif
    local_amplifier_enable(enable);

}
EXPORT_SYMBOL_GPL(vbc_amplifier_enable);
inline int vbc_amplifier_enabled(void)
{
    return local_amplifier_enabled();
}
EXPORT_SYMBOL_GPL(vbc_amplifier_enabled);

void obj_vbc_param_release(struct kobject *kobject);               //added by jian
ssize_t kobj_vbc_param_show(struct kobject *kobject, struct attribute *attr,char *buf);       //added by jian
ssize_t kobj_vbc_param_store(struct kobject *kobject,struct attribute *attr,const char *buf, size_t count);  //added by jian
ssize_t modem_status_show(struct class *class, struct class_attribute *attr, char *buf);
ssize_t modem_status_store(struct class *class, struct class_attribute *attr, const char *buf, size_t count);
ssize_t android_mode_show(struct class *class, struct class_attribute *attr, char *buf);
ssize_t android_mode_store(struct class *class, struct class_attribute *attr, const char *buf, size_t count);
ssize_t android_sim_show(struct class *class, struct class_attribute *attr, char *buf);
ssize_t android_sim_store(struct class *class, struct class_attribute *attr, const char *buf, size_t count);
ssize_t vbc_regs_show(struct class *class, struct class_attribute *attr, char *buf);
ssize_t vbc_regs_store(struct class *class, struct class_attribute *attr, const char *buf, size_t count);
// /sys/class/modem/*
static struct class_attribute modem_class_attrs[] = { // drivers/gpio/gpiolib.c
	__ATTR(status, 0766, modem_status_show, modem_status_store),
    __ATTR(mode, 0766, android_mode_show, android_mode_store),
    __ATTR(sim, 0766, android_sim_show, android_sim_store),
    __ATTR(regs, 0766, vbc_regs_show, vbc_regs_store),
	// __ATTR(unexport, 0200, NULL, unexport_store),
	__ATTR_NULL,
};

struct class modem_class = {
    .name           = "modem",
    .owner          = THIS_MODULE,
    .class_attrs    = modem_class_attrs,
};

struct attribute vbc_param_attr = {            //added by jian
        .name = "vbc_param_config",
        .mode = S_IRWXUGO,
};
static struct attribute *def_vbc_param_attrs[] = {	//added by jian
        &vbc_param_attr,
        NULL,
};
struct sysfs_ops obj_vbc_param_sysops =			//added by jian
{
        .show = kobj_vbc_param_show,
        .store = kobj_vbc_param_store,
};
struct kobj_type ktype = 				//added by jian
{
        .release = obj_vbc_param_release,
        .sysfs_ops=&obj_vbc_param_sysops,
        .default_attrs=def_vbc_param_attrs,
};

void obj_vbc_param_release(struct kobject *kobject)	//added by jian
{
        printk("vbc_param_release: release!\n");
}

ssize_t kobj_vbc_param_show(struct kobject *kobject, struct attribute *attr,char *buf)	//added by jian
{
        printk("vbc_param have show!\n");
        printk("vbc_param attrname:%s !\n", attr->name);
        sprintf(buf,"%s\n",attr->name);
        return strlen(attr->name)+2;
}

ssize_t kobj_vbc_param_store(struct kobject *kobject,struct attribute *attr,const char *buf, size_t count)	//added by jian
{	
	AUDIO_TOTAL_T *audio_param_ptr = PNULL;
	uint16_t pga_gain_god = 0;
	uint16_t pga_gain_go = 0;
	int16_t vol_index = 0;
	int16_t arm_vol = 0;
	uint32_t i;
	audio_param_ptr = (AUDIO_TOTAL_T *)buf;
	if(PNULL == audio_param_ptr)
	{
		printk("kobj_vbc_param_store audio_param_ptr is NULL!\n");
		return 0;
	}
	printk("vbc_param have store!\n");

//#if defined(CONFIG_ARCH_SC8810)
#if !defined(CONFIG_MACH_SP6820A)
	sprd_local_audio_pa_mode = audio_param_ptr->audio_nv_arm_mode_info.tAudioNvArmModeStruct.reserve[AUDIO_NV_INTPA_SWITCH_INDEX];
#endif
	cur_internal_pa_gain = audio_param_ptr->audio_nv_arm_mode_info.tAudioNvArmModeStruct.reserve[AUDIO_NV_INTPA_GAIN_INDEX] & 0xF;
//	printk("chj kobj_vbc_param_store sprd_local_audio_pa_mode:%d cur_internal_pa_gain:0x%x\n",sprd_local_audio_pa_mode,cur_internal_pa_gain);
//#endif
	if(AUDIO_NO_ERROR != AUDENHA_SetPara(audio_param_ptr))
	{
		printk("kobj_vbc_param_store AUDENHA_SetPara error!\n");
		return 0;
	}
	vol_index = audio_param_ptr->audio_nv_arm_mode_info.tAudioNvArmModeStruct.app_config_info_set.app_config_info[0].valid_volume_level_count;
	arm_vol=audio_param_ptr->audio_nv_arm_mode_info.tAudioNvArmModeStruct.app_config_info_set.app_config_info[0].arm_volume[vol_index];
	AUDDEV_SetPGA(0,arm_vol);
	AUDDEV_SetPGA(1,arm_vol);
	
	return count;
}

ssize_t modem_status_show(struct class *class, struct class_attribute *attr, char *buf)
{
    char *base = buf;
    buf += sprintf(buf, "incall:");
    buf += sprintf(buf, "%d\n", mode_incall());
    return buf - base;
}

ssize_t modem_status_store(struct class *class, struct class_attribute *attr, const char *buf, size_t count)
{
    return count;
}

static int android_mode;
enum {
    MODE_NORMAL = 0,
    MODE_RINGTONE,
    MODE_IN_CALL,
    MODE_WAITING,
    MODE_MAX
};
ssize_t android_mode_show(struct class *class, struct class_attribute *attr, char *buf)
{
    const char *mode_name;

    local_fiq_disable();
    switch (android_mode) {
        case MODE_RINGTONE: mode_name = "ringtone"; break;
        case MODE_IN_CALL:  mode_name = "incall"; break;
        case MODE_WAITING:  mode_name = "waiting"; break;
        default: android_mode = MODE_NORMAL; mode_name = "normal"; break;
    }
    local_fiq_enable();

    return sprintf(buf, "%s\n", mode_name);
}

ssize_t android_mode_store(struct class *class, struct class_attribute *attr, const char *buf, size_t count)
{
    int value;
    sscanf(buf, "%d", &value);
    local_fiq_disable();
    android_mode = value;
    if (android_mode < 0 || android_mode >= MODE_MAX)
        android_mode = MODE_NORMAL;
    local_fiq_enable();
    return count;
}


static int sim_num = 0;
ssize_t android_sim_show(struct class *class, struct class_attribute *attr, char *buf)
{
    char *base = buf;
    buf += sprintf(buf, "%d\n", sim_num);
    return buf - base;
}

ssize_t android_sim_store(struct class *class, struct class_attribute *attr, const char *buf, size_t count)
{
    int value;
	sscanf(buf, "%d", &value);
    local_fiq_disable();
    sim_num = value;
    local_fiq_enable();
    return count;
}

ssize_t vbc_regs_show(struct class *class, struct class_attribute *attr, char *buf)
{
    vbc_dump_regs(0, 0, 0);
    return 0;
}

ssize_t vbc_regs_store(struct class *class, struct class_attribute *attr, const char *buf, size_t count)
{
    return count;
}

static int vbc_reboot_notify(struct notifier_block *nb,
			       unsigned long code, void *unused)
{
    pr_warn("vbc power off\n");
    vbc_power_down((SNDRV_PCM_STREAM_LAST+1) | VBC_CODEC_POWER_DOWN_FORCE);
    return 0;
}

static struct notifier_block vbc_reboot_notifier = {
	.notifier_call = vbc_reboot_notify,
};

#define VBC_PCM_RATES (SNDRV_PCM_RATE_8000  |	\
			  SNDRV_PCM_RATE_11025 |	\
			  SNDRV_PCM_RATE_16000 |	\
			  SNDRV_PCM_RATE_22050 |	\
              SNDRV_PCM_RATE_32000 |    \
			  SNDRV_PCM_RATE_44100 |	\
			  SNDRV_PCM_RATE_48000 |    \
              SNDRV_PCM_RATE_96000)

// PCM Playing and Recording default in full duplex mode
struct snd_soc_dai vbc_dai[] = {
{
    .name = "VBC",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2, // now we only want to support stereo mode [luther.ge]
		.rates = VBC_PCM_RATES,
		.formats = VBC_PCM_FORMATS,},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 1, // now we only support mono capture
		.rates = VBC_PCM_RATES,
		.formats = VBC_PCM_FORMATS,},
	.ops = &vbc_dai_ops,
	.symmetric_rates = 1,
},
};
EXPORT_SYMBOL_GPL(vbc_dai);

struct kobject vbc_kobj;  	//added by jian

static int vbc_probe(struct platform_device *pdev)
{
    struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
    int ret = 0;

	socdev->card->codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (socdev->card->codec == NULL)
		return -ENOMEM;
	codec = socdev->card->codec;
	mutex_init(&codec->mutex);

	codec->name = "VBC";
	codec->owner = THIS_MODULE;
	codec->dai = vbc_dai;
	codec->num_dai = ARRAY_SIZE(vbc_dai);
	codec->write = vbc_write;
	codec->read = vbc_read;
#if POWER_OFF_ON_STANDBY
	codec->set_bias_level = vbc_set_bias_level;
#endif
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0)
		goto pcm_err;

	vbc_reset(codec, 0, 0);
    local_cpu_pa_control(false); // Turn off classD cpu PA
    vbc_amplifier_enable(false, "vbc_init"); // Mute Speaker
    vbc_reg_VBCR1_set(BTL_MUTE, 1); // Mute earpiece
    vbc_reg_VBCR1_set(HP_DIS, 1); // Mute headphone
#if VBC_DYNAMIC_POWER_MANAGEMENT
    vbc_power_down((SNDRV_PCM_STREAM_LAST+1) | VBC_CODEC_POWER_DOWN_FORCE);
#else
    vbc_power_on((SNDRV_PCM_STREAM_LAST+1) | VBC_CODEC_POWER_ON_FORCE);
#endif
#if 0
    /* vbc_reset() must be initialized twice, or the noise when playing audio */
    vbc_reset(codec);
#endif

#if POWER_OFF_ON_STANDBY
	vbc_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
#endif
	snd_soc_add_controls(codec, vbc_snd_controls,
				ARRAY_SIZE(vbc_snd_controls));
	vbc_add_widgets(codec);
    android_pm_init();
    android_sprd_pm_init();
    class_register(&modem_class);
	printk("kboject vbc_param init!\n");					//added by jian
	kobject_init_and_add(&vbc_kobj,&ktype,NULL,"kobject_vbc_param");	//added by jian
	return 0;

// card_err:
// 	snd_soc_free_pcms(socdev);
pcm_err:
	kfree(socdev->card->codec);
	socdev->card->codec = NULL;
	return ret;
}

static int vbc_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	if (codec == NULL)
		return 0;
	printk("kobject vbc_param exit!\n");		//added by jian
	kobject_del(&vbc_kobj);				//added by jian

    class_unregister(&modem_class);
#if POWER_OFF_ON_STANDBY
    vbc_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	vbc_set_bias_level(codec, SND_SOC_BIAS_OFF);
#endif
	snd_soc_dapm_free(socdev);
	snd_soc_free_pcms(socdev);
	kfree(codec);
    android_pm_exit();
    android_sprd_pm_exit();
	return 0;
}

struct snd_soc_codec_device vbc_codec= {
    .probe   =  vbc_probe,
    .remove  =  vbc_remove,
    .suspend =  vbc_suspend,
    .resume  =  vbc_resume,
};
EXPORT_SYMBOL_GPL(vbc_codec);

#define VBC_VTIMER_TEST 0
#define VBC_VTIMER_ROUND_JIFFIES (msecs_to_jiffies(20))
#if VBC_VTIMER_TEST
#include <linux/timer.h>
static struct timer_list lutimer;
static void lutimer_handler(unsigned long data)
{
    printk("vbc timer enter\n");
    udelay(2*1000); udelay(2*1000); udelay(2*1000); udelay(2*1000); udelay(2*1000);
    printk("vbc timer exit\n");
    mod_timer(&lutimer, jiffies + VBC_VTIMER_ROUND_JIFFIES);
}

static int lutimer_init(void)
{
    init_timer(&lutimer);
    lutimer.expires = round_jiffies(jiffies + VBC_VTIMER_ROUND_JIFFIES);
    lutimer.data = 0;
    lutimer.function = &lutimer_handler;
    add_timer(&lutimer);
    return 0;
}

static void lutimer_exit(void)
{
    del_timer_sync(&lutimer);
}
#endif

static int vbc_init(void)
{
    #if VBC_VTIMER_TEST
    lutimer_init();
    #endif
    local_amplifier_init();
    register_reboot_notifier(&vbc_reboot_notifier);
    return snd_soc_register_dais(vbc_dai, ARRAY_SIZE(vbc_dai));
}

static void vbc_exit(void)
{
    #if VBC_VTIMER_TEST
    lutimer_exit();
    #endif
    snd_soc_unregister_dais(vbc_dai, ARRAY_SIZE(vbc_dai));
    unregister_reboot_notifier(&vbc_reboot_notifier);
}

module_init(vbc_init);
module_exit(vbc_exit);

MODULE_DESCRIPTION("ALSA SoC SpreadTrum VBC codec");
MODULE_AUTHOR("Luther Ge <luther.ge@spreadtrum.com>");
MODULE_LICENSE("GPL");
