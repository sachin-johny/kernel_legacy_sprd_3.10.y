/*
 * sound/soc/sprd/vbc-codec.c
 *
 * VBC -- SpreadTrum sc88xx intergrated Dolphin codec.
 *
 * Copyright (C) 2012 SpreadTrum Ltd.
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
#include "sc88xx-asoc.h"
#include <sound/audio_pa.h>
#include <mach/adi.h>
#include <mach/globalregs.h>
#include "aud_enha.h"

#define VBC_EQ_MODULE_SUPPORT			1
#define VBC_DYNAMIC_POWER_MANAGEMENT		0
#define POWER_OFF_ON_STANDBY			0
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

#define VBC_CODEC_RESET				0xffff
#define VBC_CODEC_POWER				0xfffe
#define VBC_CODEC_POWER_ON_OUT			(1 << 0)
#define VBC_CODEC_POWER_ON_IN			(1 << 1)
#define VBC_CODEC_POWER_OFF_OUT			(1 << 2)
#define VBC_CODEC_POWER_OFF_IN			(1 << 3)
#define VBC_CODEC_POWER_ON_OUT_MUTE_DAC		(1 << 4)
#define VBC_CODEC_POWER_ON_FORCE		(1 << 29)
#define VBC_CODEC_POWER_DOWN_FORCE		(1 << 30)
#define VBC_CODEC_SPEAKER_PA			0xfffd
#define VBC_CODEC_DSP				0xfffc
#define VBC_CODEC_POWER2			0xfffb

static volatile int earpiece_muted = 1, headset_muted = 1, speaker_muted = 1;
static void audio_speaker_enable(int enable, const char *prename);
static int audio_speaker_enabled(void);
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

#define SOC_VBC_ROUTING_SOC_SINGLE(xname, reg, shift, max, invert) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = snd_soc_info_volsw, .get = snd_soc_get_volsw,\
	.put = vbc_routing_put_volsw, \
	.private_value =  SOC_SINGLE_VALUE(reg, shift, max, invert) }

static int vbc_routing_put_volsw(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	int reg = mc->reg;
	int shift = mc->shift;
	int ret;
	u16 val;

	ret = snd_soc_put_volsw(kcontrol, ucontrol);
	if (ret < 0)
		return ret;

	if (reg == VBCR1) {
		switch (shift) {
		case BTL_MUTE:
			if (audio_pa_amplifier && audio_pa_amplifier->earpiece.control)
				audio_pa_amplifier->earpiece.control(!earpiece_muted, NULL);
			break;
		case HP_DIS:
			if (audio_pa_amplifier && audio_pa_amplifier->headset.control)
				audio_pa_amplifier->headset.control(!headset_muted, NULL);
			break;
		}
	}
	return ret;
}

static const struct snd_kcontrol_new vbc_snd_controls[] = {
	SOC_ENUM("Micphone", vbc_mic12_enum),
	SOC_SINGLE("PCM Playback Switch", VBCR1, DAC_MUTE, 1, 1),
#if VBC_EQ_MODULE_SUPPORT
	SOC_SINGLE("PCM2 Playback Switch", VBCR1, DAC_MUTE, 1, 1),
	SOC_DOUBLE_TLV("PCM2 Playback Volume", VBCGR1, 0, 4, 0x0f, 1, dac_tlv),
	SOC_SINGLE_TLV("PCM2 Left Playback Volume", VBCGR1, 0, 0x0f, 1, dac_tlv),
	SOC_SINGLE_TLV("PCM2 Right Playback Volume",VBCGR1, 4, 0x0f, 1, dac_tlv),
#endif
	SOC_VBC_ROUTING_SOC_SINGLE("Speaker Playback Switch", VBC_CODEC_SPEAKER_PA, 0, 1, 0),
	SOC_VBC_ROUTING_SOC_SINGLE("Earpiece Playback Switch", VBCR1, BTL_MUTE, 1, 1),
	SOC_VBC_ROUTING_SOC_SINGLE("Headset Playback Switch", VBCR1, HP_DIS, 1, 1),
#if !VBC_EQ_MODULE_SUPPORT
	SOC_DOUBLE_TLV("Speaker Playback Volume", VBCGR1, 0, 4, 0x0f, 1, dac_tlv),
	SOC_SINGLE_TLV("Speaker Left Playback Volume", VBCGR1, 0, 0x0f, 1, dac_tlv),
	SOC_SINGLE_TLV("Speaker Right Playback Volume",VBCGR1, 4, 0x0f, 1, dac_tlv),
	SOC_DOUBLE_TLV("Earpiece Playback Volume", VBCGR1, 0, 4, 0x0f, 1, dac_tlv),
	SOC_SINGLE_TLV("Earpiece Left Playback Volume", VBCGR1, 0, 0x0f, 1, dac_tlv),
	SOC_SINGLE_TLV("Earpiece Right Playback Volume",VBCGR1, 4, 0x0f, 1, dac_tlv),
	SOC_DOUBLE_TLV("Headset Playback Volume", VBCGR1, 0, 4, 0x0f, 1, dac_tlv),
	SOC_SINGLE_TLV("Headset Left Playback Volume", VBCGR1, 0, 0x0f, 1, dac_tlv),
	SOC_SINGLE_TLV("Headset Right Playback Volume",VBCGR1, 4, 0x0f, 1, dac_tlv),
#endif
	SOC_SINGLE("BypassFM Playback Switch", VBCR1, BYPASS, 1, 0),
	SOC_DOUBLE_R_TLV("BypassFM Playback Volume", VBCGR2, VBCGR3, 0, 0x1f, 1, dac_tlv_fm),
	SOC_SINGLE_TLV("BypassFM Left Playback Volume", VBCGR2, 0, 0x1f, 1, dac_tlv_fm),
	SOC_SINGLE_TLV("BypassFM Right Playback Volume",VBCGR3, 0, 0x1f, 1, dac_tlv_fm),
	SOC_SINGLE("LineinFM", VBPMR1, SB_LIN, 1, 1),
	SOC_SINGLE("LineinFM_Record", ANA_AUDIO_CTRL, 3, 1, 0),
	SOC_SINGLE_TLV("Capture Capture Volume", VBCGR10, 4, 0x0f, 0, adc_tlv),
	SOC_ENUM("Reset Codec", vbc_codec_reset_enum),
	SOC_SINGLE("Power Codec", VBC_CODEC_POWER, 0, 31, 0),
	SOC_SINGLE("Power Codec2",VBC_CODEC_POWER2,0, 31, 0),
	SOC_SINGLE("InCall", VBC_CODEC_DSP, 0, 1, 0),
};
static int32_t cur_sample_rate=44100;
//static int32_t sprd_local_audio_pa_mode=0;
static int32_t cur_internal_pa_gain = 0x8;   //default:1000----0db  added by jian
static int32_t b_internal_pa_open = 0;
static int32_t cur_fm_gain_l = 0x0;
static int32_t cur_fm_gain_r = 0x0;			//0 FM Gain to max by default added by jian
static int32_t cur_capture_gain = 0xf;		//default:set GI to max added by jian
static int32_t cur_captre_gim_gain = 0x1;	//default:set GIM to max added by jian
static int32_t b_fm_headset_on = 0;
static int32_t b_fm_handsfree_on = 0;


u32 vbc_reg_write(u32 reg, u8 shift, u32 val, u32 mask)
{
	u32 tmp, ret;
	enter_critical();
	if (not_in_adi_range(reg)) tmp = __raw_readl(reg);
	else tmp = sci_adi_read(reg);
	ret = tmp;
	tmp &= ~(mask << shift);
	tmp |= val << shift;
	if (not_in_adi_range(reg)) __raw_writel(tmp, reg);
	else sci_adi_raw_write(reg, tmp);
	exit_critical();
	return ret & (mask << shift);
}

u32 vbc_reg_read(u32 reg, u8 shift, u32 mask)
{
	u32 tmp;
	enter_critical();
	if (not_in_adi_range(reg)) tmp = __raw_readl(reg);
	else tmp = sci_adi_read(reg);
	exit_critical();
	return tmp & (mask << shift);
}

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
		if (prefix) pr_info("%s", prefix);
		else pr_info("%s",
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
							"0x%08x |	 0x%04x\n",
							paddr, vbc_reg_read(ivaddr, 0, UINT_MAX));
		} else pr_info("0x%08x |	 0x%04x\n",
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
		case 0:
			buf += print_cpu_regs(SPRD_VB_PHYS , 0, 107, 0,
					buft - buf, 1, "vbc_dump_regs VBDA0 - HPCOEF42\n");
			buf += print_cpu_regs(SPRD_ADI_PHYS, 0x0100, 22, 0,
					buft - buf, 1, "VBAICR - VBTR2\n");
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
	pr_info("---------------- [ start %d ] ----------------\n", type);
	vbc_print_reg(0x82000104, 1); /* VBCR1 */
	vbc_print_reg(0x82000674, 3); /* ANA_AUDIO_CTRL */
	vbc_print_reg(0x82000680, 1); /* ANA_MIXED_CTRL */
	vbc_print_reg(0x82000610, 1); /* ANA_LDO_PD_CTL0 */
	vbc_print_reg(0x82000620, 1); /* ANA_LDO_VCTL2 */
	pr_info("---------------- [ end %d ] ----------------\n", type);
#endif
}

static const struct snd_soc_dapm_widget vbc_dapm_widgets[] = {
	
};

/* vbc supported audio map */
static const struct snd_soc_dapm_route audio_map[] = {
	
};

static int vbc_add_widgets(struct snd_soc_codec *codec)
{
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	snd_soc_dapm_new_controls(dapm, vbc_dapm_widgets,
				ARRAY_SIZE(vbc_dapm_widgets));

	snd_soc_dapm_add_routes(dapm, audio_map, ARRAY_SIZE(audio_map));

	return 0;
}

static inline void vbc_reg_VBAICR_set(u8 mode)
{
	vbc_reg_write(VBAICR, 0, mode, 0x0F);
}

static inline int vbc_reg_VBCR1_set(u32 type, u32 val)
{
	switch (type) {
	case BTL_MUTE:
		if (audio_pa_amplifier && audio_pa_amplifier->earpiece.control)
			audio_pa_amplifier->earpiece.control(!val, NULL);
		break;
	case HP_DIS:
		if (audio_pa_amplifier && audio_pa_amplifier->headset.control)
			audio_pa_amplifier->headset.control(!val, NULL);
		break;
	}
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
	/* Software access ping-pong buffer enable when VBENABE bit low */
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
	vbc_buffer_clear(1); /* clear data buffer 1 */
	vbc_buffer_clear(0); /* clear data buffer 0 */
	vbc_access_buf(false);
}

static int vbc_codec_mute(void)
{
	return vbc_reg_VBCR1_set(DAC_MUTE, 1);
}

static int vbc_codec_unmute(void)
{
	return vbc_reg_VBCR1_set(DAC_MUTE, 0);
}

static inline void __raw_bits_or(unsigned int v, unsigned int a)
{
	unsigned long flags;
	local_irq_save(flags);
	__raw_writel((__raw_readl(a) | v), a);
	local_irq_restore(flags);
}

static void vbc_set_ctrl2arm(void)
{
	/* Enable VB DAC0/DAC1 (ARM-side) */
	__raw_writel(__raw_readl(SPRD_VBC_ALSA_CTRL2ARM_REG) |
			ARM_VB_MCLKON | ARM_VB_ACC | ARM_VB_DA0ON |
			ARM_VB_ANAON | ARM_VB_DA1ON | ARM_VB_ADCON,
			SPRD_VBC_ALSA_CTRL2ARM_REG);
	msleep(1);
}

/* FIXME: use regulator interface for power on/off */
static void vbc_ldo_on(int on)
{
	if (on) {
		sci_adi_set(ANA_LDO_PD_CTL0, (1 << 15));
		sci_adi_clr(ANA_LDO_PD_CTL0, (1 << 14));
	} else {
		sci_adi_clr(ANA_LDO_PD_CTL0, (1 << 15));
		sci_adi_set(ANA_LDO_PD_CTL0, (1 << 14));
	}
	pr_debug("vbc set ldo to %s \n", on ? "on" : "off");
}

/* FIXME: use clk interfaces */
static void vbc_set_mainclk_to12M(void)
{
	/* Enable voiceband module */
	sprd_greg_set_bits(REG_TYPE_GLOBAL, GEN0_VB_EN, GR_GEN0);
	/* CLK_ADI_EN_ARM and CLK_ADI_SEL=76.8MHZ */
	sprd_greg_set_bits(REG_TYPE_GLOBAL, 1 << 29, GR_CLK_DLY);

	vbc_ldo_on(1);
	sci_adi_set(ANA_AUDIO_CTRL, 0x1 | 0x2);
}

static inline void vbc_ready2go(void)
{
	/*
	 * Enable this bit then VBC interface starts working and software
	 * can receive voice band interrupt.
	 * Better set this bit after all other register bits are programmed.
	 */
	vbc_reg_VBDABUFFDTA_set(VBENABLE, 1);
	msleep(2);
}

void vbc_write_callback(unsigned int reg, unsigned int val)
{
	if (reg == VBCR1) {
		headset_muted  = !!(val & (1 << HP_DIS));
		earpiece_muted = !!(val & (1 << BTL_MUTE));
#if VBC_DYNAMIC_POWER_MANAGEMENT
		pr_debug("[headset_muted =%d]\n"
				"[earpiece_muted=%d]\n"
				"[speaker_muted =%d]\n", headset_muted, earpiece_muted, speaker_muted);
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
	pr_debug("audio %s\n", __func__);
	{
		int do_sb_power = 0;
		use_delay = 1; /* (in_irq() || irqs_disabled()) ? 0 : 1; // !headset_muted; */
		/* int VBCGR1_value; */
		if ((vbc_reg_read(VBPMR1, SB_ADC, 1) &&
			(value == SNDRV_PCM_STREAM_PLAYBACK && !vbc_reg_read(VBPMR1, SB_DAC, 1))) ||
			(vbc_reg_read(VBPMR1, SB_DAC, 1) &&
			(value == SNDRV_PCM_STREAM_CAPTURE && !vbc_reg_read(VBPMR1, SB_ADC, 1))) ||
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
			pr_info("---- vbc do power down ----\n");
#endif
			/* VBCGR1_value = vbc_reg_write(VBCGR1, 0, 0xff, 0xff); // DAC Gain */
			if (use_delay) msleep(100); /* avoid quick switch from power on to off */
			vbc_reg_VBCR1_set(BTL_MUTE, 1); /* Mute earpiece */
			vbc_reg_VBCR1_set(HP_DIS, 1); /* Mute headphone */
			audio_speaker_enable(false, "vbc_power_down playback"); /* Mute speaker */
			vbc_codec_mute();
			if (use_delay) msleep(50);
			vbc_reg_VBCR1_set(DACSEL, 0); /* not route DAC to mixer */

			vbc_reg_VBPMR1_set(SB_OUT, 1); /* Power down DAC OUT */
			vbc_reg_VBCR1_set(MONO, 1); /* mono DAC channel */
			vbc_reg_VBPMR1_set(SB_BTL, 1); /* power down earphone */
			if (use_delay) msleep(100);

			vbc_reg_VBPMR1_set(SB_DAC, 1); /* Power down DAC */
			vbc_reg_VBPMR1_set(SB_LOUT, 1);
			vbc_reg_VBPMR1_set(SB_MIX, 1);
			/* vbc_reg_write(VBCGR1, 0, VBCGR1_value, 0xff); // DAC Gain */
		}
#if !VBC_DYNAMIC_POWER_MANAGEMENT
		} else {
			if (value == SNDRV_PCM_STREAM_PLAYBACK) {
				vbc_reg_VBCR1_set(BTL_MUTE, 1); /* Mute earpiece */
				vbc_reg_VBCR1_set(HP_DIS, 1); /* Mute headphone */
				audio_speaker_enable(false, "vbc_power_down playback"); /* Mute speaker */
				vbc_codec_mute();
				pr_info("---- vbc mute all pa ----\n");
			}
			do_sb_power = 0;
		}
#endif
		if ((value == -1) ||
			(value == SNDRV_PCM_STREAM_CAPTURE &&
			!vbc_reg_read(VBPMR1, SB_ADC, 1))) {
			pr_info("vbc_power_down capture\n");
			vbc_reg_VBPMR1_set(SB_ADC, 1); /* Power down ADC */
			vbc_reg_VBCR1_set(SB_MICBIAS, 1); /* power down mic */
		}
		if ((value == -1) ||
			do_sb_power)/* vbc_reg_read(VBPMR1, SB_ADC, 1) && vbc_reg_read(VBPMR1, SB_DAC, 1) */ {
			vbc_reg_VBPMR2_set(SB_SLEEP, 1); /* SB enter sleep mode */
			vbc_reg_VBPMR2_set(SB, 1); /* Power down sb */
			if (use_delay) msleep(100); /* avoid quick switch from power off to on */
			vbc_ldo_on(0);
			pr_info("....................... vbc full power down [%d]-%d .......................\n", use_delay, in_irq() || irqs_disabled());
		}
	}
}

#if !VBC_DYNAMIC_POWER_MANAGEMENT
void vbc_power_on_playback(bool ldo)
{
	ldo = ldo;
}
#endif

void vbc_power_on_capture(bool ldo)
{
	if (vbc_reg_read(VBPMR1, SB_ADC, 1)) {
		if (ldo) vbc_ldo_on(1);
		pr_info("vbc_power_on capture\n");
		vbc_reg_VBPMR2_set(SB, 0); /* Power on sb */
		vbc_reg_VBPMR2_set(SB_SLEEP, 0); /* SB quit sleep mode */

		vbc_reg_VBPMR2_set(GIM, cur_captre_gim_gain); /* 20db gain mic amplifier */
		vbc_reg_write(VBCGR10, 4, cur_capture_gain, 0xf); /* set GI to max */
		vbc_reg_VBCR1_set(SB_MICBIAS, 0); /* power on mic */
		vbc_reg_VBPMR1_set(SB_ADC, 0); /* Power on ADC */
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
	pr_debug("audio %s\n", __func__);
	{
		use_delay = 1; /* (in_irq() || irqs_disabled()) ? 0 : 1;; // !headset_muted; */
		if (power_on_force ||
			(value == SNDRV_PCM_STREAM_PLAYBACK &&
			(vbc_reg_read(VBPMR1, SB_DAC, 1)	||
			vbc_reg_read(VBPMR1, SB_LOUT, 1)	||
			vbc_reg_read(VBPMR1, SB_OUT, 1)		||
			vbc_reg_read(VBPMR1, SB_MIX, 1)		||
			vbc_reg_read(VBPMR2, SB, 1)			||
			vbc_reg_read(VBPMR2, SB_SLEEP, 1)))) {
#if VBC_DYNAMIC_POWER_MANAGEMENT
			int forced = 0;
#endif
			/* int VBCGR1_value; */
#if !VBC_DYNAMIC_POWER_MANAGEMENT
			pr_info("---- vbc do power on ----\n");
#endif
			/* VBCGR1_value = vbc_reg_write(VBCGR1, 0, 0xff, 0xff); // DAC Gain */
			vbc_reg_VBPMR2_set(SB, 0); /* Power on sb */
			vbc_reg_VBPMR2_set(SB_SLEEP, 0); /* SB quit sleep mode */

			vbc_codec_mute();

			vbc_buffer_clear_all(); /* must have this func, or first play will have noise */

			/* earpiece_muted = */ vbc_reg_VBCR1_set(BTL_MUTE, 1); /* Mute earpiece */
			/* headset_muted =  */ vbc_reg_VBCR1_set(HP_DIS, 1); /* Mute headphone */
			/* speaker_muted =  */ audio_speaker_enable(false, "vbc_power_on playback"); /* Mute speaker */
			if (use_delay) msleep(50);

			vbc_reg_VBCR1_set(DACSEL, 1); /* route DAC to mixer */
			vbc_reg_VBPMR1_set(SB_DAC, 0); /* Power on DAC */
			if (use_delay) msleep(50);
			vbc_reg_VBPMR1_set(SB_LOUT, 0);
			if (use_delay) msleep(50);
			vbc_reg_VBPMR1_set(SB_MIX, 0);
			if (use_delay) msleep(50);

			vbc_reg_VBPMR1_set(SB_OUT, 0); /* Power on DAC OUT */
			if (use_delay) msleep(50);
			vbc_reg_VBCR1_set(MONO, 0); /* stereo DAC left & right channel */
			vbc_reg_VBPMR1_set(SB_BTL, 0); /* power on earphone */
			if (use_delay) msleep(100);

			if (!mute_dac) vbc_codec_unmute();
#if VBC_DYNAMIC_POWER_MANAGEMENT
			if (!earpiece_muted || forced) vbc_reg_VBCR1_set(BTL_MUTE, 0); /* unMute earpiece */
			if (!headset_muted || forced) vbc_reg_VBCR1_set(HP_DIS, 0); /* unMute headphone */
			if (!speaker_muted || forced) audio_speaker_enable(true, "vbc_power_on playback"); /* unMute speaker */
			pr_info("....................... vbc power on playback [%d]-%d .......................\n", use_delay, in_irq() || irqs_disabled());
#endif
			/* vbc_reg_write(VBCGR1, 0, VBCGR1_value, 0xff); // DAC Gain */
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

static inline int mode_incall(void)
{
	return !(__raw_readl(SPRD_VBC_ALSA_CTRL2ARM_REG) & ARM_VB_ACC);
}

static int vbc_reset(struct snd_soc_codec *codec, int poweron, int check_incall)
{
	int ret = 0;
	pr_info("vbc reset start...\n");
	/*
	 * 1. dial phone number
	 * 2. modem will set DSP control audio codec
	 * 3. DSP control audio codec
	 * 4. in call
	 * 5. AT+ATH to quit call
	 * 6. DSP will release audio chain
	 * 7. modem will set ARM control audio codec
	 * 8. android will reset audio codec to ARM & setting android alsa himself needed audio parameters
	 *
	 * The problem occures in step 7 & 8, if 8 first occures, pop sound will be created, and
	 * alsa DMA can't be work, AudioFlinger will can't obtainBuffer from alsa driver [luther.ge]
	 */
	if (check_incall) {
		#define VBC_DSP_WAITING_MAX_COUNT   20
		int try_max = 0;
		/* audio_speaker_enable(false, "vbc_init"); // Mute Speaker */
		/* fix above problem */
		while (mode_incall() && try_max++ < VBC_DSP_WAITING_MAX_COUNT) {
			pr_warn("vbc waiting DSP release audio codec ......\n");
			msleep(100);
		}
		if (try_max >= VBC_DSP_WAITING_MAX_COUNT) {
			pr_err("vbc say God God God God God God God oh my !!!!\n");
			ret = -1;
		}
		pr_warn("vbc waiting modem stable setting audio codec ...... start ......\n");
		msleep(200);
		pr_warn("vbc waiting modem stable setting audio codec ...... done ......\n");
	}
	vbc_set_mainclk_to12M();
	vbc_set_ctrl2arm();
	if (poweron)
		vbc_power_on((SNDRV_PCM_STREAM_LAST+1) | VBC_CODEC_POWER_ON_FORCE | VBC_CODEC_POWER_ON_OUT_MUTE_DAC);
	vbc_ready2go();
	vbc_set_AD_DA_fifo_frame_num(VBC_FIFO_FRAME_NUM, VBC_FIFO_FRAME_NUM);
	vbc_buffer_clear_all(); /* must have this func, or first play will have noise */
	/*
	 * IIS timeing : High(right channel) for both DA1/AD1, Low(left channel) for both DA0/AD0
	 * Active level of left/right channel for both ADC and DAC channel
	 */
	vbc_set_VBADBUFFDTA_set(VBIIS_LRCK, 1);
	vbc_reg_VBAICR_set(VBCAICR_MODE_ADC_I2S	|
					VBCAICR_MODE_DAC_I2S	|
					VBCAICR_MODE_ADC_SERIAL |
					VBCAICR_MODE_DAC_SERIAL);

	vbc_reg_VBCR2_set(DAC_ADWL, DAC_DATA_WIDTH_16_bit); /* DAC data sample depth 16bits */
	vbc_reg_VBCR2_set(ADC_ADWL, ADC_DATA_WIDTH_16_bit); /* ADC data sample depth 16bits */
	vbc_reg_VBCR2_set(MICSEL, MICROPHONE1); /* route microphone 1 to ADC module */

	vbc_reg_write(VBCCR2, 4, VBC_RATE_8000, 0xf); /* 8K sample DAC */
	vbc_reg_write(VBCCR2, 0, VBC_RATE_8000, 0xf); /* 8K sample ADC */

	vbc_reg_write(VBCGR2, 0, cur_fm_gain_l, 0x1f); /* FM Gain to max by default */
	vbc_reg_write(VBCGR3, 0, cur_fm_gain_r, 0x1f);
#if !VBC_EQ_MODULE_SUPPORT
	vbc_reg_write(VBCGR1, 0, 0x00, 0xff); /* DAC Gain */
	vbc_reg_write(VBCGR8, 0, 0x00, 0x1f);
	vbc_reg_write(VBCGR9, 0, 0x00, 0x1f);
#endif

	vbc_reg_VBPMR2_set(GIM, cur_captre_gim_gain); /* 20db gain mic amplifier */
	vbc_reg_write(VBCGR10, 4, cur_capture_gain, 0xf); /* set GI to max */
	vbc_reg_VBPMR1_set(SB_ADC, 1); /* Power down ADC */
	vbc_reg_VBCR1_set(SB_MICBIAS, 1); /* power down mic */

	pr_info("vbc reset finish...\n");
	return ret;
}

static int vbc_soft_ctrl(struct snd_soc_codec *codec, unsigned int reg, unsigned int value, int dir)
{
	int ret = 0;
	pr_debug("vbc_soft_ctrl value[%d]=%04x\n", dir, reg);
	switch (reg) {
		case VBC_CODEC_RESET:
			/*
			 * After phone call, we should reset all codec related registers
			 * because in phone call state dsp will control codec, and set all registers
			 * so we should reset all registers again in linux side,
			 * otherwise android media will not work [luther.ge]
			 */
			/* if (val & (1 << VBC_CODEC_SOFT_RESET)) */
			if (dir == 0) return 0; /* dir 0 for read, we always return 0, so every set 1 value can reach here. */
#if VBC_DYNAMIC_POWER_MANAGEMENT
			ret = vbc_reset(codec, 0, 1);
			vbc_power_down(-1);
#else
			ret = vbc_reset(codec, 1, 1);
#endif
			/* ret = vbc_reset(codec); */
			if (!earpiece_muted) vbc_reg_VBCR1_set(BTL_MUTE, 0); /* unMute earpiece */
			if (!headset_muted) vbc_reg_VBCR1_set(HP_DIS, 0); /* unMute headphone */
			if (!speaker_muted) audio_speaker_enable(true, "vbc_soft_ctrl"); /* unMute speaker */
			return ret < 0 ? -2 : ret;
		case VBC_CODEC_POWER:
			if (dir == 0) return 0; /* dir 0 for read, we always return 0, so every set 1 value can reach here. */
			pr_info("vbc power to 0x%08x\n", value);
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
			if (dir == 0) return 0; /* dir 0 for read, we always return 0, so every set 1 value can reach here. */
			pr_info("vbc power2 to 0x%08x\n", value);
			vbc_power_down((SNDRV_PCM_STREAM_LAST+1) | VBC_CODEC_POWER_DOWN_FORCE);
			return value;
		case VBC_CODEC_DSP:
			return mode_incall();
		case VBC_CODEC_SPEAKER_PA:
			if (dir) {
				audio_speaker_enable(value & 0x01, "vbc_soft_ctrl2");
			}
			value = audio_speaker_enabled();
			speaker_muted = value ? 0:1;
			return value;
		default: return -1;
	}
}

static unsigned int vbc_read(struct snd_soc_codec *codec, unsigned int reg)
{
	int ret = vbc_soft_ctrl(codec, reg, 0, 0);
	if (ret != -1) return ret;
	/* Because snd_soc_update_bits reg is 16 bits short type, so muse do following convert */
	reg |= ARM_VB_BASE2;
	return sci_adi_read(reg);
}

static int vbc_write(struct snd_soc_codec *codec, unsigned int reg, unsigned int val)
{
	int ret = vbc_soft_ctrl(codec, reg, val, 1);
	if (ret != -1) return ret;
	/* Because snd_soc_update_bits reg is 16 bits short type, so muse do following convert */
	reg |= ARM_VB_BASE2;
	vbc_write_callback(reg, val);
	sci_adi_raw_write(reg, val);
	return 0;
}

static void vbc_dma_control(int chs, bool on)
{
	if (chs & DMA_VB_DA0_BIT)
		vbc_reg_VBDABUFFDTA_set(VBDA0DMA_EN, on); /* DMA write DAC0 data buffer enable/disable */
	if (chs & DMA_VB_DA1_BIT)
		vbc_reg_VBDABUFFDTA_set(VBDA1DMA_EN, on); /* DMA write DAC1 data buffer enable/disable */
	if (chs & DMA_VB_AD0_BIT)
		vbc_reg_VBDABUFFDTA_set(VBAD0DMA_EN, on); /* DMA read ADC0 data buffer enable/disable */
	if (chs & DMA_VB_AD1_BIT)
		vbc_reg_VBDABUFFDTA_set(VBAD1DMA_EN, on); /* DAM read ADC1 data buffer enable/disable */
}

extern u32 sc88xx_get_dma_channel(struct snd_pcm_substream *substream);
void vbc_dma_start(struct snd_pcm_substream *substream)
{
	vbc_dma_control(sc88xx_get_dma_channel(substream), 1);
}

static inline void vbc_dma_stop(struct snd_pcm_substream *substream)
{
	vbc_dma_control(sc88xx_get_dma_channel(substream), 0);
}

void flush_vbc_cache(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	/* we could not stop vbc_dma_buffer immediately, because audio data still in cache */
	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return;
	/* clear dma cache buffer */
	memset((void*)runtime->dma_area, 0, runtime->dma_bytes);
	pr_info("vbc flush cache buffer...\n");
	if (cpu_codec_dma_chain_operate_ready(substream)) {
		vbc_dma_start(substream); /* we must restart dma */
		start_cpu_dma(substream);
		/* must wait all dma cache chain filled by 0 data */
		msleep(5);
		stop_cpu_dma(substream);
		vbc_dma_stop(substream);
	}
}

static int vbc_startup(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		vbc_buffer_clear_all();
	}
	return 0;
}

static int vbc_prepare(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	if (cpu_codec_dma_chain_operate_ready(substream)) {
		vbc_dma_stop(substream);
	}
	return 0;
}

static void vbc_shutdown(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	pr_debug("vbc_shutdown......\n");
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
				pr_warn("vbc hardware nosie current unmute dac now!\n");
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
#else
			vbc_buffer_clear_all(); /* must have this func, or first play will have noise */
#endif
			break;
		case SNDRV_PCM_TRIGGER_STOP:
			/* vbc_power_down(substream->stream); */
		case SNDRV_PCM_TRIGGER_SUSPEND:
		case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
#if VBC_NOSIE_CURRENT_SOUND_HARDWARE_BUG_FIX
			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
				/* must mute here, otherwise noise current sound will appear */
				dac_muted = !!vbc_reg_read(VBCR1, DAC_MUTE, 1);
				if (!dac_muted) {
					vbc_codec_mute();
					pr_warn("vbc hardware nosie current mute dac now!\n");
				}
			}
#endif
			vbc_dma_stop(substream); /* Stop DMA transfer */
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
		default: pr_err("vbc codec only supports format 16bits"); break;
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
		default: pr_err("vbc codec not supports rate %d\n", params_rate(params)); break;
	}

	printk("vbc sample rate is [%d]\n", params_rate(params));

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
	.startup	= vbc_startup,
	.prepare	= vbc_prepare,
	.trigger	= vbc_trigger,
	.hw_params	= vbc_pcm_hw_params,
	.hw_free	= vbc_hw_free,
	.shutdown	= vbc_shutdown,
	.set_clkdiv	= vbc_set_dai_clkdiv,
	.set_fmt	= vbc_set_dai_fmt,
	.set_tristate = vbc_set_dai_tristate,
};

static int vbc_codec_full_power_down = 0;
static struct snd_soc_codec *pcodec;
int vbc_resume_late(struct snd_pcm_substream *substream, const char *prefix)
{
	int ret = 0;
	if (vbc_codec_full_power_down) {
		mutex_lock(&pcodec->mutex);
		if (vbc_codec_full_power_down) {
			vbc_reset(pcodec, 1, 0);
			vbc_codec_full_power_down = 0;
			pr_info("vbc yyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyy <%s>\n", prefix);
		}
		ret = 1;
		mutex_unlock(&pcodec->mutex);
	}
	return ret;
}
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
static struct early_suspend early_suspend;
static void learly_suspend(struct early_suspend *es) { }
static void learly_resume(struct early_suspend *es)
{
	vbc_resume_late(NULL, "vbc early_resume");
}

static void android_pm_init(void)
{
    early_suspend.suspend = learly_suspend;
    early_suspend.resume = learly_resume;
    early_suspend.level = 0; // we are the last one
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

#ifdef CONFIG_PM
int vbc_soc_suspend(struct snd_soc_codec *codec, pm_message_t state)
{
	if(!vbc_reg_read(VBPMR1, SB_LIN, 1))
	{    
		printk("vbc_suspend FM is still running !!!!!!!!!\n");
		return 0;
	} 
	mutex_lock(&codec->mutex);
	if (vbc_codec_full_power_down == 0) {
		pr_info("vbc xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n");
		vbc_print_regs(0);
		vbc_power_down((SNDRV_PCM_STREAM_LAST+1) | VBC_CODEC_POWER_DOWN_FORCE);
		vbc_print_regs(0);
		vbc_codec_full_power_down = 1;
	} else {
		pr_info("vbc xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx has done\n");
	}
	mutex_unlock(&codec->mutex);
	return 0;
}

int vbc_soc_resume(struct snd_soc_codec *codec)
{    
	if(!vbc_reg_read(VBPMR1, SB_LIN, 1))
	{
		printk("vbc_resume FM is still running !!!!!!!!!\n");
		return 0;
	}
#if 1
	pr_info("vbc yyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyy later do this\n");
#else
	mutex_lock(&codec->mutex);
	pr_info("vbc yyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyy\n");
	vbc_print_regs(1);
#if 1
	vbc_reset(codec, 1, 0);
#else
	if (codec) vbc_reset(codec, 0, 0);
	vbc_power_on((SNDRV_PCM_STREAM_LAST+1) | VBC_CODEC_POWER_ON_FORCE);
#endif
	mutex_unlock(&codec->mutex);
	vbc_print_regs(1);
#endif
	return 0;
}
#else
#define vbc_soc_suspend NULL
#define vbc_soc_resume  NULL
#endif

static inline void local_cpu_pa_control(bool enable)
{
	b_internal_pa_open = enable;
	if (enable) {
		sci_adi_raw_write(ANA_AUDIO_PA_CTRL0, (0x1101 |( (cur_internal_pa_gain <<4) & 0x00F0)));
		sci_adi_raw_write(ANA_AUDIO_PA_CTRL1, 0x5e41);
	} else {
		sci_adi_raw_write(ANA_AUDIO_PA_CTRL0, 0x182);
		sci_adi_raw_write(ANA_AUDIO_PA_CTRL1, 0x1242);
	}
}

static void audio_speaker_enable(int enable, const char *prename)
{
#if VBC_DYNAMIC_POWER_MANAGEMENT
	pr_debug("vbc %s ==> trun %s PA\n", prename, enable ? "on":"off");
	pr_debug("[headset_muted =%d]\n"
			"[earpiece_muted=%d]\n"
			"[speaker_muted =%d]\n", headset_muted, earpiece_muted, speaker_muted);
#endif
	if (audio_pa_amplifier && audio_pa_amplifier->speaker.control)
		audio_pa_amplifier->speaker.control(enable, NULL);
	else local_cpu_pa_control(enable);
}

static int audio_speaker_enabled(void)
{
	if (audio_pa_amplifier && audio_pa_amplifier->speaker.control) {
		return audio_pa_amplifier->speaker.control(-1, NULL);
	} else {
		u32 value = sci_adi_read(ANA_AUDIO_PA_CTRL0);
		value &= 0x1101;
		switch (value) {
			case 0x1101: return 1;
			default : return 0;
		}
	}
}

ssize_t kclass_vbc_param_show(struct class *class, struct class_attribute *attr,char *buf);       //modify by jian
ssize_t kclass_vbc_param_store(struct class *class,struct class_attribute *attr,const char *buf, size_t count);  //modify by jian
ssize_t kclass_fm_devstat_show(struct class *class, struct class_attribute *attr, char *buf);
ssize_t kclass_fm_devstat_store(struct class *class, struct class_attribute *attr, const char *buf, size_t count);
ssize_t modem_status_show(struct class *class, struct class_attribute *attr, char *buf);
ssize_t modem_status_store(struct class *class, struct class_attribute *attr, const char *buf, size_t count);
ssize_t android_mode_show(struct class *class, struct class_attribute *attr, char *buf);
ssize_t android_mode_store(struct class *class, struct class_attribute *attr, const char *buf, size_t count);
ssize_t android_sim_show(struct class *class, struct class_attribute *attr, char *buf);
ssize_t android_sim_store(struct class *class, struct class_attribute *attr, const char *buf, size_t count);
ssize_t vbc_regs_show(struct class *class, struct class_attribute *attr, char *buf);
ssize_t vbc_regs_store(struct class *class, struct class_attribute *attr, const char *buf, size_t count);
ssize_t android_switch_show(struct class *class, struct class_attribute *attr, char *buf);
ssize_t android_switch_store(struct class *class, struct class_attribute *attr, const char *buf, size_t count);
ssize_t android_codec_show(struct class *class, struct class_attribute *attr, char *buf);
ssize_t android_codec_store(struct class *class, struct class_attribute *attr, const char *buf, size_t count);

/* /sys/class/modem/xxx */
static struct class_attribute modem_class_attrs[] = {
	__ATTR(status, 0664, modem_status_show, modem_status_store),
	__ATTR(mode, 0664, android_mode_show, android_mode_store),
	__ATTR(sim, 0664, android_sim_show, android_sim_store),
	__ATTR(regs, 0664, vbc_regs_show, vbc_regs_store),
	__ATTR(switch, 0664, android_switch_show, android_switch_store),
	__ATTR(codec, 0664, android_codec_show, android_codec_store),
	__ATTR_NULL,
};
/* /sys/class/vbc_param_config/vbc_param_store */
static struct class_attribute vbc_param_class_attrs[] = { //modify by jian
	__ATTR(vbc_param_store, 0664, kclass_vbc_param_show, kclass_vbc_param_store),
	// __ATTR(unexport, 0200, NULL, unexport_store),
	__ATTR_NULL,
};

/* /sys/class/fm_devstat_config/fm_devstat_store */
static struct class_attribute fm_devstat_class_attrs[] = { //modify by jian
	__ATTR(fm_devstat_store, 0664, kclass_fm_devstat_show, kclass_fm_devstat_store),
	// __ATTR(unexport, 0200, NULL, unexport_store),
	__ATTR_NULL,
};

struct class modem_class = {
	.name	= "modem",
	.owner	= THIS_MODULE,
	.class_attrs = modem_class_attrs,
};
struct class vbc_param_class = { //modify by jian
	.name           = "vbc_param_config",
	.owner          = THIS_MODULE,
	.class_attrs    = vbc_param_class_attrs,
};

struct class fm_devstat_class = { //modify by jian
	.name           = "fm_devstat_config",
	.owner          = THIS_MODULE,
	.class_attrs    = fm_devstat_class_attrs,
};

ssize_t kclass_fm_devstat_show(struct class *class, struct class_attribute *attr,char *buf)
{
//	printk("chj kclass_fm_devstat_show attrname:%s !\n", attr->attr.name);
	sprintf(buf,"%s\n",attr->attr.name);
	return strlen(attr->attr.name)+2;
}

ssize_t kclass_fm_devstat_store(struct class *class,struct class_attribute *attr,const char *buf, size_t count)
{
	AUDIO_FM_DEVSTAT_T *audio_fm_devstat = PNULL;
	audio_fm_devstat = (AUDIO_FM_DEVSTAT_T*)buf;
	b_fm_headset_on = audio_fm_devstat->fm_headset_stat;
	b_fm_handsfree_on = audio_fm_devstat->fm_handsfree_stat;
//	printk("chj kclass_fm_devstat_store b_fm_headset_on:%d b_fm_handsfree_on:%d \n",b_fm_headset_on,b_fm_handsfree_on);
	return count;
}

ssize_t kclass_vbc_param_show(struct class *class, struct class_attribute *attr,char *buf)	//modify by jian
{
	printk("kclass_vbc_param_show attrname:%s !\n", attr->attr.name);
	sprintf(buf,"%s\n",attr->attr.name);
	return strlen(attr->attr.name)+2;
}

ssize_t kclass_vbc_param_store(struct class *class,struct class_attribute *attr,const char *buf, size_t count)	//modify by jian
{	
	AUDIO_TOTAL_T *audio_param_ptr = PNULL;
	uint16_t pga_gain_god = 0;
	uint16_t pga_gain_go = 0;
	int16_t vol_index = 0;
	int16_t arm_vol = 0;
	int16_t fm_temp = 0;
	uint32_t i;
	audio_param_ptr = (AUDIO_TOTAL_T *)buf;
	if(PNULL == audio_param_ptr){
		printk("kclass_vbc_param_store audio_param_ptr is NULL!\n");
		return 0;
	}
	printk("kclass_vbc_param_store have store!\n");

//	sprd_local_audio_pa_mode = audio_param_ptr->audio_nv_arm_mode_info.tAudioNvArmModeStruct.reserve[AUDIO_NV_INTPA_SWITCH_INDEX];
	cur_internal_pa_gain = audio_param_ptr->audio_nv_arm_mode_info.tAudioNvArmModeStruct.reserve[AUDIO_NV_INTPA_GAIN_INDEX] & 0x00FF;
	cur_capture_gain = audio_param_ptr->audio_nv_arm_mode_info.tAudioNvArmModeStruct.reserve[AUDIO_NV_CAPTURE_GAIN_INDEX] & 0x00FF;
	cur_captre_gim_gain = (audio_param_ptr->audio_nv_arm_mode_info.tAudioNvArmModeStruct.reserve[AUDIO_NV_CAPTURE_GAIN_INDEX] & 0xFF00)>>8;
	cur_fm_gain_l = (audio_param_ptr->audio_nv_arm_mode_info.tAudioNvArmModeStruct.reserve[AUDIO_NV_FM_GAINL_INDEX]>>8) & 0x001F;	//GOBL
	cur_fm_gain_r = (audio_param_ptr->audio_nv_arm_mode_info.tAudioNvArmModeStruct.reserve[AUDIO_NV_FM_GAINR_INDEX]>>8) & 0x001F;	//GOBR
	if(AUDIO_NO_ERROR != AUDENHA_SetPara(audio_param_ptr)){
		printk("kclass_vbc_param_store AUDENHA_SetPara error!\n");
		return 0;
	}
	vol_index = audio_param_ptr->audio_nv_arm_mode_info.tAudioNvArmModeStruct.app_config_info_set.app_config_info[0].valid_volume_level_count;
	arm_vol=audio_param_ptr->audio_nv_arm_mode_info.tAudioNvArmModeStruct.app_config_info_set.app_config_info[0].arm_volume[vol_index];
	if(b_fm_headset_on){
		fm_temp = arm_vol&0xfe0f;
		arm_vol = (((audio_param_ptr->audio_nv_arm_mode_info.tAudioNvArmModeStruct.reserve[AUDIO_NV_FM_GAINL_INDEX]&0x00FF)<<4)&0x1F0)|fm_temp;
	}
	AUDDEV_SetPGA(0,arm_vol);
	AUDDEV_SetPGA(1,arm_vol);
	if(b_fm_handsfree_on){
		cur_internal_pa_gain = (audio_param_ptr->audio_nv_arm_mode_info.tAudioNvArmModeStruct.reserve[AUDIO_NV_INTPA_GAIN_INDEX] & 0xFF00)>>8;
	}
	if((!audio_pa_amplifier) || (!audio_pa_amplifier->speaker.control)){
		local_cpu_pa_control(b_internal_pa_open);
	}
	vbc_reg_VBPMR2_set(GIM, cur_captre_gim_gain);
	vbc_reg_write(VBCGR10, 4, cur_capture_gain, 0xf);
	vbc_reg_write(VBCGR2, 0, cur_fm_gain_l, 0x1f);
	vbc_reg_write(VBCGR3, 0, cur_fm_gain_r, 0x1f);
//	printk("chj kclass_vbc_param_store mode_name:%s b_fm_headset_on:%d b_fm_handsfree_on:%d cur_internal_pa_gain:0x%x cur_capture_gain:0x%x cur_captre_gim_gain:0x%x cur_fm_gain_l:0x%x cur_fm_gain_r:0x%x b_internal_pa_open:%d GOL:0x%x \n",audio_param_ptr->audio_nv_arm_mode_info.ucModeName,b_fm_headset_on,b_fm_handsfree_on,cur_internal_pa_gain,cur_capture_gain,cur_captre_gim_gain,cur_fm_gain_l,cur_fm_gain_r,b_internal_pa_open,(arm_vol>>4)&0x1f);
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
ssize_t android_mode_show(struct class *class, struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", android_mode);
}

ssize_t android_mode_store(struct class *class, struct class_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf, "%d", &android_mode);
	return count;
}

static int sim_num = 0;
ssize_t android_sim_show(struct class *class, struct class_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", sim_num);
}

ssize_t android_sim_store(struct class *class, struct class_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf, "%d", &sim_num);
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

ssize_t android_switch_show(struct class *class, struct class_attribute *attr, char *buf)
{
    return sprintf(buf, "%s|%s|%s\n",
			(audio_pa_amplifier && audio_pa_amplifier->earpiece.control) ? "0.earpiece" : "",
			(audio_pa_amplifier && audio_pa_amplifier->headset.control) ? "1.headset" : "",
			(audio_pa_amplifier && audio_pa_amplifier->speaker.control) ? "2.speaker" : "");
}

ssize_t android_switch_store(struct class *class, struct class_attribute *attr, const char *buf, size_t count)
{
	int type = buf[0];
	int cmd = buf[1];
	switch (type) {
	case 0:
		if (audio_pa_amplifier && audio_pa_amplifier->earpiece.control)
			audio_pa_amplifier->earpiece.control(cmd, NULL);
		else
			pr_warn("vbc switch not available on <earpiece>\n");
		break;
	case 1:
		if (audio_pa_amplifier && audio_pa_amplifier->headset.control)
			audio_pa_amplifier->headset.control(cmd, NULL);
		else
			pr_warn("vbc switch not available on <headset>\n");
		break;
	case 2:
		if (audio_pa_amplifier && audio_pa_amplifier->speaker.control)
			audio_pa_amplifier->speaker.control(cmd, NULL);
		else
			pr_warn("vbc switch not available on <speaker>\n");
		break;
	default:
		pr_err("vbc switch not support this type <0x%02x>\n", type);
		break;
	}
    return count;
}

ssize_t android_codec_show(struct class *class, struct class_attribute *attr, char *buf)
{
    return 0;
}

ssize_t android_codec_store(struct class *class, struct class_attribute *attr, const char *buf, size_t count)
{
	vbc_resume_late(NULL, "user space sys write or open");
    return count;
}

#define VBC_PCM_RATES (SNDRV_PCM_RATE_8000 | \
				SNDRV_PCM_RATE_11025 | \
				SNDRV_PCM_RATE_16000 | \
				SNDRV_PCM_RATE_22050 | \
				SNDRV_PCM_RATE_32000 | \
				SNDRV_PCM_RATE_44100 | \
				SNDRV_PCM_RATE_48000 | \
				SNDRV_PCM_RATE_96000)

/* PCM Playing and Recording default in full duplex mode */
struct snd_soc_dai_driver vbc_dai[] = {
{
	.name = "vbc-codec-embeded",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = VBC_PCM_RATES,
		.formats = VBC_PCM_FORMATS,},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 1,
		.rates = VBC_PCM_RATES,
		.formats = VBC_PCM_FORMATS,},
	.ops = &vbc_dai_ops,
	/* .symmetric_rates = 1, */
},
};

static int vbc_soc_probe(struct snd_soc_codec *codec)
{
	pcodec = codec;
	vbc_reset(codec, 0, 0);
	local_cpu_pa_control(false); /* Turn off classD cpu PA */
	audio_speaker_enable(false, "vbc_init"); /* Mute Speaker */
	vbc_reg_VBCR1_set(BTL_MUTE, 1); /* Mute earpiece */
	vbc_reg_VBCR1_set(HP_DIS, 1); /* Mute headphone */
#if VBC_DYNAMIC_POWER_MANAGEMENT
	vbc_power_down((SNDRV_PCM_STREAM_LAST+1) | VBC_CODEC_POWER_DOWN_FORCE);
#else
#if VBC_NOSIE_CURRENT_SOUND_HARDWARE_BUG_FIX
	vbc_codec_mute();
#endif
#endif
	android_pm_init();
	snd_soc_add_codec_controls(codec, vbc_snd_controls,
				ARRAY_SIZE(vbc_snd_controls));
	vbc_add_widgets(codec);
	class_register(&modem_class);
	class_register(&fm_devstat_class);
	printk("kclass vbc_param init!\n");		//added by jian
	class_register(&vbc_param_class);	//modify by jian
	return 0;
}

/* power down chip */
static int vbc_soc_remove(struct snd_soc_codec *codec)
{
	printk("kclass vbc_param exit!\n"); 	//added by jian
	class_unregister(&vbc_param_class); //modify by jian
	class_unregister(&fm_devstat_class);
	class_unregister(&modem_class);
#if POWER_OFF_ON_STANDBY
	vbc_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	vbc_set_bias_level(codec, SND_SOC_BIAS_OFF);
#endif
	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_vbc = {
	.probe = vbc_soc_probe,
	.remove = vbc_soc_remove,
	.suspend = vbc_soc_suspend,
	.resume = vbc_soc_resume,
	.read = vbc_read,
	.write = vbc_write,
#if POWER_OFF_ON_STANDBY
	.set_bias_level = vbc_set_bias_level,
#endif
	.reg_word_size = sizeof(u16),
	.reg_cache_step = 2,
	.dapm_widgets = vbc_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(vbc_dapm_widgets),
	.dapm_routes = audio_map,
	.num_dapm_routes = ARRAY_SIZE(audio_map),
};

static __devinit int vbc_probe(struct platform_device *pdev)
{
	return snd_soc_register_codec(&pdev->dev,
			&soc_codec_dev_vbc, vbc_dai, ARRAY_SIZE(vbc_dai));
}

static int __devexit vbc_remove(struct platform_device *pdev)
{
	android_pm_exit();
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

static struct platform_driver vbc_codec_driver = {
	.driver = {
		.name = "vbc-codec",
		.owner = THIS_MODULE,
	},
	.probe = vbc_probe,
	.remove = __devexit_p(vbc_remove),
};

static int vbc_init(void)
{
	return platform_driver_register(&vbc_codec_driver);
}

static void vbc_exit(void)
{
	platform_driver_unregister(&vbc_codec_driver);
}

module_init(vbc_init);
module_exit(vbc_exit);

MODULE_DESCRIPTION("ALSA SoC SpreadTrum VBC codec");
MODULE_AUTHOR("Luther Ge <luther.ge@spreadtrum.com>");
MODULE_LICENSE("GPL");
