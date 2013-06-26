/*
 * sound/soc/sprd/dai/vbc/vbc.c
 *
 * SPRD SoC CPU-DAI -- SpreadTrum SOC DAI with EQ&ALC and some loop.
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
#define pr_fmt(fmt) "[audio: vbc ] " fmt

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/stat.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/firmware.h>
#include <linux/workqueue.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm_params.h>
#include <sound/tlv.h>

#include "../sprd-pcm.h"
#include "vbc_r2p0.h"

#ifdef CONFIG_SPRD_AUDIO_DEBUG
#define vbc_dbg pr_debug
#else
#define vbc_dbg(...)
#endif

#define FUN_REG(f) ((unsigned short)(-((f) + 1)))
#define SOC_REG(r) ((unsigned short)(r))

#define SWITCH_FUN_ON    1
#define SWITCH_FUN_OFF   0

enum {
	VBC_LEFT = 0,
	VBC_RIGHT = 1,
};

#define VBC_DG_VAL_MAX (0x7F)

struct vbc_fw_header {
	char magic[VBC_EQ_FIRMWARE_MAGIC_LEN];
	u32 profile_version;
	u32 num_profile;	/*total num */
	u32 num_da;
	u32 num_ad01;
	u32 num_ad23;
};

struct vbc_da_eq_profile {
	char magic[VBC_EQ_FIRMWARE_MAGIC_LEN];
	char name[VBC_EQ_PROFILE_NAME_MAX];
	/* TODO */
	u32 effect_paras[VBC_DA_EFFECT_PARAS_LEN];
};

struct vbc_ad_eq_profile {
	char magic[VBC_EQ_FIRMWARE_MAGIC_LEN];
	char name[VBC_EQ_PROFILE_NAME_MAX];
	/* TODO */
	u32 effect_paras[VBC_AD_EFFECT_PARAS_LEN];
};

static const u32 vbc_da_eq_profile_default[VBC_DA_EFFECT_PARAS_LEN] = {
/* TODO the default register value */
	0x00000000,		/*  DAPATCHCTL      */
	0x0000007F,		/*  DAHPCTL         */
	0x00000000,		/*  DAALCCTL0       */
	0x00000000,		/*  DAALCCTL1       */
	0x00000000,		/*  DAALCCTL2       */
	0x00000000,		/*  DAALCCTL3       */
	0x00000000,		/*  DAALCCTL4       */
	0x00000000,		/*  DAALCCTL5       */
	0x00000000,		/*  DAALCCTL6       */
	0x00000000,		/*  DAALCCTL7       */
	0x00000000,		/*  DAALCCTL8       */
	0x00000000,		/*  DAALCCTL9       */
	0x00000000,		/*  DAALCCTL10      */
	0x00000183,		/*  STCTL0          */
	0x00000183,		/*  STCTL1          */
	0x00000000,		/*  DACSRCCTL       */
	0x00000000,		/*  MIXERCTL        */
	0x00000000,		/*  VBNGCVTHD       */
	0x00000000,		/*  VBNGCTTHD       */
	0x00000000,		/*  VBNGCTL         */
	0x00000000,		/*  HPCOEF0_H ~ HPCOEF71_H   */
	0x00000000,		/*  HPCOEF0_L  ~ HPCOEF71_L   */
};

static const u32 vbc_ad_eq_profile_default[VBC_AD_EFFECT_PARAS_LEN] = {
/* TODO the default register value */
	0x00000000,		/*  ADPATCHCTL      */
	0x00000000,		/*  ADHPCTL         */
	0x00000000,		/*  ADC01_HPCOEF0_H  or   ADC23_HPCOEF0_H   */
	0x00000000,		/*  ADC01_HPCOEF0_L  or ADC23_HPCOEF0_L     */
};

struct vbc_equ {
	struct device *dev;
	int is_active[VBC_CHAN_MAX];
	int is_loading;
	struct snd_soc_dai *codec_dai;
	int now_profile[VBC_CHAN_MAX];
	struct vbc_fw_header hdr;
	void *data[VBC_CHAN_MAX];
	void (*vbc_eq_apply) (struct snd_soc_dai * codec_dai, void *data,
			      enum vbc_chan chan);
	int vbc_idx;
};

typedef int (*vbc_dma_set) (int enable);
typedef int (*vbc_dg_set) (int enable, int dg);
static DEFINE_MUTEX(vbc_mutex);
struct vbc_priv {
	vbc_dma_set dma_set[2];

	int (*arch_enable) (int chan);
	int (*arch_disable) (int chan);
	int is_open;
	int is_active;
	int used_chan_count;
	int dg_switch[2];
	int dg_val[2];
	vbc_dg_set dg_set[2];
};

static DEFINE_MUTEX(load_mutex);
static struct vbc_equ vbc_eq_setting = { 0 };

static int vbc_control;
static int vbc_da_iis_port;
static int adc_dgmux_val[ADC_DGMUX_MAX];
static int fm_sample_rate = 32000;
struct st_hpf_dg {
	int dg_switch[2];
	int dg_val[2];
};

static struct st_hpf_dg st_dg = { {0, 0}, {0x18, 0x18} };

static void vbc_eq_try_apply(struct snd_soc_dai *codec_dai,
			     enum vbc_chan chan_id);
static struct vbc_priv vbc[3];
static struct clk *s_vbc_clk = 0;
static struct sprd_pcm_dma_params vbc_pcm_stereo_out = {
	.name = "VBC PCM Stereo out",
/*	.workmode = DMA_LINKLIST,	no use */
	.irq_type = BLK_DONE,
	.desc = {
		 .datawidth = SHORT_WIDTH,
		 .fragmens_len = VBC_FIFO_FRAME_NUM * 2,
		 .src_step = 2,
		 .des_step = 0,
		 },
#ifdef CONFIG_SPRD_VBC_LR_INVERT
	.dev_paddr = {PHYS_VBDA1, PHYS_VBDA0},
#else
	.dev_paddr = {PHYS_VBDA0, PHYS_VBDA1},
#endif
};

static struct sprd_pcm_dma_params vbc_pcm_stereo_in = {
	.name = "VBC PCM Stereo in",
/*	.workmode = DMA_LINKLIST,*/
	.irq_type = BLK_DONE,
	.desc = {
		 .datawidth = SHORT_WIDTH,
		 .fragmens_len = VBC_FIFO_FRAME_NUM * 2,
		 .src_step = 0,
		 .des_step = 2,
		 },
	.dev_paddr = {PHYS_VBAD0, PHYS_VBAD1},
};

static struct sprd_pcm_dma_params vbc_pcm23_stereo_in = {
	.name = "VBC PCM23 Stereo in",
/*	.workmode = DMA_LINKLIST,*/
	.irq_type = BLK_DONE,
	.desc = {
		 .datawidth = SHORT_WIDTH,
		 .fragmens_len = VBC_FIFO_FRAME_NUM * 2,
		 .src_step = 0,
		 .des_step = 2,
		 },
	.dev_paddr = {PHYS_VBAD2, PHYS_VBAD3},
};

/*vbc mux setting*/
enum {
	SPRD_VBC_MUX_START = 0,
	SPRD_VBC_ST0_CHAN_MUX = SPRD_VBC_MUX_START,
	SPRD_VBC_ST1_CHAN_MUX,
	SPRD_VBC_ST0_MUX,
	SPRD_VBC_ST1_MUX,
	SPRD_VBC_AD0_INMUX,
	SPRD_VBC_AD1_INMUX,
	SPRD_VBC_AD2_INMUX,
	SPRD_VBC_AD3_INMUX,
	SPRD_VBC_AD_IISMUX,
	SPRD_VBC_AD23_IISMUX,
	SPRD_VBC_MUX_MAX
};

const char *vbc_mux_debug_str[SPRD_VBC_MUX_MAX] = {
	"st0 chan mux",
	"st1 chan mux",
	"st0 mux",
	"st1 mux",
	"ad0 inmux",
	"ad1 inmux",
	"ad2 inmux",
	"ad3 inmux",
	"ad iis mux",
	"ad23 iis mux"
};

typedef int (*sprd_vbc_mux_set) (int sel);
struct sprd_vbc_mux_op {
	int val;
	sprd_vbc_mux_set set;
};
struct sprd_vbc_mux_op sprd_vbc_mux[SPRD_VBC_MUX_MAX];

/* vbc local power suppliy and chan on */
static struct vbc_refcount {
	atomic_t vbc_power_on;	/*vbc reg power enable */
	atomic_t vbc_on;	/*vbc enable */
	atomic_t da0_on;
	atomic_t da1_on;
	atomic_t ad0_on;
	atomic_t ad1_on;
	atomic_t ad2_on;
	atomic_t ad3_on;
} vbc_refcnt;

static char *vbc_get_chan_name(int chan_id)
{
	return ((chan_id == 0) ? "DAC" : ((chan_id == 1) ? "ADC01" : "ADC23"));
}

static int vbc_reg_write(int reg, int val, int mask);

static int vbc_st0_chnmux_set(int val)
{
	vbc_reg_write(STCTL1, val << VBST0_SEL_CHN, (1 << VBST0_SEL_CHN));
	return 0;
}

static int vbc_st1_chnmux_set(int val)
{
	vbc_reg_write(STCTL1, val << VBST1_SEL_CHN, (1 << VBST1_SEL_CHN));
	return 0;
}

static int vbc_st0_inmux_set(int val)
{
	vbc_reg_write(ADPATCHCTL, val << VBADPATH_ST0_INMUX_SHIFT,
		      VBADPATH_ST0_INMUX_MASK);
	return 0;
}

static int vbc_st1_inmux_set(int val)
{
	vbc_reg_write(ADPATCHCTL, val << VBADPATH_ST1_INMUX_SHIFT,
		      VBADPATH_ST1_INMUX_MASK);
	return 0;
}

static int vbc_ad0_inmux_set(int val)
{
	vbc_reg_write(ADPATCHCTL, val << VBADPATH_AD0_INMUX_SHIFT,
		      VBADPATH_AD0_INMUX_MASK);
	return 0;
}

static int vbc_ad1_inmux_set(int val)
{
	vbc_reg_write(ADPATCHCTL, val << VBADPATH_AD1_INMUX_SHIFT,
		      VBADPATH_AD1_INMUX_MASK);
	return 0;
}

static int vbc_ad2_inmux_set(int val)
{
	vbc_reg_write(ADPATCHCTL, val << VBADPATH_AD2_INMUX_SHIFT,
		      VBADPATH_AD2_INMUX_MASK);
	return 0;
}

static int vbc_ad3_inmux_set(int val)
{
	vbc_reg_write(ADPATCHCTL, val << VBADPATH_AD3_INMUX_SHIFT,
		      VBADPATH_AD3_INMUX_MASK);
	return 0;
}

static int vbc_ad0_dgmux_set(int val)
{
	vbc_reg_write(ADPATCHCTL, val << VBADPATH_AD0_DGMUX_SHIFT,
		      VBADPATH_AD0_DGMUX_MASK);
	return 0;
}

static int vbc_ad1_dgmux_set(int val)
{
	vbc_reg_write(ADPATCHCTL, val << VBADPATH_AD1_DGMUX_SHIFT,
		      VBADPATH_AD1_DGMUX_MASK);
	return 0;
}

static int vbc_ad2_dgmux_set(int val)
{
	vbc_reg_write(ADPATCHCTL, val << VBADPATH_AD2_DGMUX_SHIFT,
		      VBADPATH_AD2_DGMUX_MASK);
	return 0;
}

static int vbc_ad3_dgmux_set(int val)
{
	vbc_reg_write(ADPATCHCTL, val << VBADPATH_AD3_DGMUX_SHIFT,
		      VBADPATH_AD3_DGMUX_MASK);
	return 0;
}

static int vbc_adc_src_set(int rate, int is_ad23)
{
	int f1f2f3_bp;
	int f1_sel;
	int en_sel;
	int val;
	int mask;

	vbc_dbg("Entering %s\n", __func__);
	vbc_dbg("rate:%d, chan: ad%s", rate, (is_ad23) ? "23" : "01");

	/*src_clr */
	if (!is_ad23) {
		vbc_reg_write(ADCSRCCTL, (1 << VBADCSRC_CLR_01),
			      (1 << VBADCSRC_CLR_01));
		udelay(10);
		vbc_reg_write(ADCSRCCTL, 0, (1 << VBADCSRC_CLR_01));
	} else {
		vbc_reg_write(ADCSRCCTL, (1 << VBADCSRC_CLR_23),
			      (1 << VBADCSRC_CLR_23));
		udelay(10);
		vbc_reg_write(ADCSRCCTL, 0, (1 << VBADCSRC_CLR_23));

	}
	switch (rate) {
	case 32000:
		f1f2f3_bp = 0;
		f1_sel = 1;
		en_sel = 1;
		break;
	case 48000:
		f1f2f3_bp = 0;
		f1_sel = 0;
		en_sel = 1;
		break;
	case 44100:
		f1f2f3_bp = 1;
		f1_sel = 0;
		en_sel = 1;
		break;
	default:
		f1f2f3_bp = 0;
		f1_sel = 0;
		en_sel = 0;
		break;
	}
	/*src_set */
	if (!is_ad23) {
		mask =
		    (1 << VBADCSRC_F1F2F3_BP_01) | (1 << VBADCSRC_F1_SEL_01) |
		    (1 << VBADCSRC_EN_01);

		val =
		    (f1f2f3_bp << VBADCSRC_F1F2F3_BP_01) | (f1_sel <<
							    VBADCSRC_F1_SEL_01)
		    | (en_sel << VBADCSRC_EN_01);
	} else {
		mask =
		    (1 << VBADCSRC_F1F2F3_BP_23) | (1 << VBADCSRC_F1_SEL_23) |
		    (1 << VBADCSRC_EN_23);

		val =
		    (f1f2f3_bp << VBADCSRC_F1F2F3_BP_23) | (f1_sel <<
							    VBADCSRC_F1_SEL_23)
		    | (en_sel << VBADCSRC_EN_23);
	}

	vbc_reg_write(ADCSRCCTL, val, mask);

	vbc_dbg("Leaving %s\n", __func__);
	return 0;
}

static int vbc_ad_iismux_set(int port)
{
	vbc_reg_write(VBIISSEL, port << VBIISSEL_AD01_PORT_SHIFT,
		      VBIISSEL_AD01_PORT_MASK);
	/*ADC  SRC set */
	if (port == 1 || port == 2)	/*fm input */
		vbc_adc_src_set(fm_sample_rate, 0);
	return 0;
}

static int vbc_ad23_iismux_set(int port)
{
	vbc_reg_write(VBIISSEL, port << VBIISSEL_AD23_PORT_SHIFT,
		      VBIISSEL_AD23_PORT_MASK);
	/*ADC23  SRC set */
	if (port == 1 || port == 2)	/*fm input */
		vbc_adc_src_set(fm_sample_rate, 1);
	return 0;
}

static int vbc_da_iismux_set(int port)
{
	vbc_reg_write(VBIISSEL, port << VBIISSEL_DA_PORT_SHIFT,
		      VBIISSEL_DA_PORT_MASK);
	return 0;
}

sprd_vbc_mux_set vbc_mux_cfg[SPRD_VBC_MUX_MAX] = {
	vbc_st0_chnmux_set,
	vbc_st1_chnmux_set,
	vbc_st0_inmux_set,
	vbc_st1_inmux_set,
	vbc_ad0_inmux_set,
	vbc_ad1_inmux_set,
	vbc_ad2_inmux_set,
	vbc_ad3_inmux_set,
	vbc_ad_iismux_set,
	vbc_ad23_iismux_set
};

static inline void vbc_safe_mem_release(void **free)
{
	if (*free) {
		kfree(*free);
		*free = NULL;
	}
}

#define vbc_safe_kfree(p) vbc_safe_mem_release((void**)p)

static DEFINE_SPINLOCK(vbc_lock);
/* local register setting */
static int vbc_reg_write(int reg, int val, int mask)
{
	int tmp, ret;
	spin_lock(&vbc_lock);
	tmp = __raw_readl(reg - VBC_REG_OFFSET);
	ret = tmp;
	tmp &= ~(mask);
	tmp |= val & mask;
	__raw_writel(tmp, reg - VBC_REG_OFFSET);
	spin_unlock(&vbc_lock);
	return ret & (mask);
}

inline int vbc_reg_read(int reg)
{
	int tmp;
	tmp = __raw_readl(reg - VBC_REG_OFFSET);
	return tmp;
}

EXPORT_SYMBOL_GPL(vbc_reg_read);

inline int vbc_reg_write2(int reg, int val)
{
	int tmp;
	tmp = __raw_writel(val, reg - VBC_REG_OFFSET);
	return tmp;
}

EXPORT_SYMBOL_GPL(vbc_reg_write2);

static int vbc_set_buffer_size(int ad_buffer_size, int da_buffer_size,
			       int ad23_buffer_size)
{
	int val = vbc_reg_read(VBBUFFSIZE);
	WARN_ON(ad_buffer_size > VBC_FIFO_FRAME_NUM);
	WARN_ON(da_buffer_size > VBC_FIFO_FRAME_NUM);
	WARN_ON(ad23_buffer_size > VBC_FIFO_FRAME_NUM);
	if ((ad_buffer_size > 0)
	    && (ad_buffer_size <= VBC_FIFO_FRAME_NUM)) {
		val &= ~(VBADBUFFERSIZE_MASK);
		val |= (((ad_buffer_size - 1) << VBADBUFFERSIZE_SHIFT)
			& VBADBUFFERSIZE_MASK);
	}
	if ((da_buffer_size > 0)
	    && (da_buffer_size <= VBC_FIFO_FRAME_NUM)) {
		val &= ~(VBDABUFFERSIZE_MASK);
		val |= (((da_buffer_size - 1) << VBDABUFFERSIZE_SHIFT)
			& VBDABUFFERSIZE_MASK);
	}
	vbc_reg_write(VBBUFFSIZE, val,
		      (VBDABUFFERSIZE_MASK | VBADBUFFERSIZE_MASK));
	if ((ad23_buffer_size > 0)
	    && (ad23_buffer_size <= VBC_FIFO_FRAME_NUM)) {
		val &= ~(VBAD23BUFFERSIZE_MASK);
		val |= (((ad23_buffer_size - 1) << VBAD23BUFFERSIZE_SHIFT)
			& VBAD23BUFFERSIZE_MASK);
	}
	vbc_reg_write(VBBUFFAD23, val, VBAD23BUFFERSIZE_MASK);
	return 0;
}

static inline int vbc_sw_write_buffer(int enable)
{
	/* Software access ping-pong buffer enable when VBENABE bit low */
	vbc_reg_write(VBDABUFFDTA, ((enable ? 1 : 0) << RAMSW_EN),
		      (1 << RAMSW_EN));
	return 0;
}

static inline int vbc_da_enable(int enable, int chan)
{
	vbc_reg_write(VBCHNEN, ((enable ? 1 : 0) << (VBDACHEN_SHIFT + chan)),
		      (1 << (VBDACHEN_SHIFT + chan)));
	return 0;
}

static inline int vbc_ad_enable(int enable, int chan)
{
	vbc_reg_write(VBCHNEN, ((enable ? 1 : 0) << (VBADCHEN_SHIFT + chan)),
		      (1 << (VBADCHEN_SHIFT + chan)));
	return 0;
}

static inline int vbc_ad23_enable(int enable, int chan)
{
	vbc_reg_write(VBCHNEN, ((enable ? 1 : 0) << (VBAD23CHEN_SHIFT + chan)),
		      (1 << (VBAD23CHEN_SHIFT + chan)));
	return 0;
}

static inline int vbc_enable_set(int enable)
{
	vbc_reg_write(VBADBUFFDTA, (0 << VBIIS_LRCK), (1 << VBIIS_LRCK));
	vbc_reg_write(VBDABUFFDTA, ((enable ? 1 : 0) << VBENABLE),
		      (1 << VBENABLE));
	return 0;
}

static inline void vbc_reg_enable(void)
{
	if (s_vbc_clk) {
		clk_enable(s_vbc_clk);
	} else {
		arch_audio_vbc_reg_enable();
	}
}

static inline void vbc_reg_disable(void)
{
	if (s_vbc_clk) {
		clk_disable(s_vbc_clk);
	} else {
		arch_audio_vbc_reg_disable();
	}
}

static inline int vbc_da0_enable(int enable)
{
	if (enable) {
		atomic_inc(&vbc_refcnt.da0_on);
		if (atomic_read(&vbc_refcnt.da0_on) == 1) {
			vbc_da_enable(1, 0);
			pr_info("VBC DA0 ON\n");
		}
	} else {
		if (atomic_dec_and_test(&vbc_refcnt.da0_on)) {
			vbc_da_enable(0, 0);
			pr_info("VBC DA0 OFF\n");
		}
		if (atomic_read(&vbc_refcnt.da0_on) < 0) {
			atomic_set(&vbc_refcnt.da0_on, 0);
		}
	}
	vbc_dbg("------da0 ref: %d", atomic_read(&vbc_refcnt.da0_on));
	return 0;
}

static inline int vbc_da1_enable(int enable)
{
	if (enable) {
		atomic_inc(&vbc_refcnt.da1_on);
		if (atomic_read(&vbc_refcnt.da1_on) == 1) {
			vbc_da_enable(1, 1);
			pr_info("VBC DA1 ON\n");
		}
	} else {
		if (atomic_dec_and_test(&vbc_refcnt.da1_on)) {
			vbc_da_enable(0, 1);
			pr_info("VBC DA1 OFF\n");
		}
		if (atomic_read(&vbc_refcnt.da1_on) < 0) {
			atomic_set(&vbc_refcnt.da1_on, 0);
		}
	}

	vbc_dbg("------da1 ref: %d", atomic_read(&vbc_refcnt.da1_on));
	return 0;
}

static inline int vbc_ad0_enable(int enable)
{
	if (enable) {
		atomic_inc(&vbc_refcnt.ad0_on);
		if (atomic_read(&vbc_refcnt.ad0_on) == 1) {
			vbc_ad_enable(1, 0);
			pr_info("VBC AD0 ON\n");
		}
	} else {
		if (atomic_dec_and_test(&vbc_refcnt.ad0_on)) {
			vbc_ad_enable(0, 0);
			pr_info("VBC AD0 OFF\n");
		}
		if (atomic_read(&vbc_refcnt.ad0_on) < 0) {
			atomic_set(&vbc_refcnt.ad0_on, 0);
		}
	}

	vbc_dbg("------ad0 ref: %d", atomic_read(&vbc_refcnt.ad0_on));
	return 0;
}

static inline int vbc_ad1_enable(int enable)
{
	if (enable) {
		atomic_inc(&vbc_refcnt.ad1_on);
		if (atomic_read(&vbc_refcnt.ad1_on) == 1) {
			vbc_ad_enable(1, 1);
			pr_info("VBC AD1 ON\n");
		}
	} else {
		if (atomic_dec_and_test(&vbc_refcnt.ad1_on)) {
			vbc_ad_enable(0, 1);
			pr_info("VBC AD1 OFF\n");
		}
		if (atomic_read(&vbc_refcnt.ad1_on) < 0) {
			atomic_set(&vbc_refcnt.ad1_on, 0);
		}
	}

	vbc_dbg("------ad1 ref: %d", atomic_read(&vbc_refcnt.ad1_on));
	return 0;
}

static inline int vbc_ad2_enable(int enable)
{
	if (enable) {
		atomic_inc(&vbc_refcnt.ad2_on);
		if (atomic_read(&vbc_refcnt.ad2_on) == 1) {
			vbc_ad23_enable(1, 0);
			pr_info("VBC AD2 ON\n");
		}
	} else {
		if (atomic_dec_and_test(&vbc_refcnt.ad2_on)) {
			vbc_ad23_enable(0, 0);
			pr_info("VBC AD2 OFF\n");
		}
		if (atomic_read(&vbc_refcnt.ad2_on) < 0) {
			atomic_set(&vbc_refcnt.ad2_on, 0);
		}
	}
	vbc_dbg("------ad2 ref: %d", atomic_read(&vbc_refcnt.ad2_on));

	return 0;
}

static inline int vbc_ad3_enable(int enable)
{
	if (enable) {
		atomic_inc(&vbc_refcnt.ad3_on);
		if (atomic_read(&vbc_refcnt.ad3_on) == 1) {
			vbc_ad23_enable(1, 1);
			pr_info("VBC AD3 ON\n");
		}
	} else {
		if (atomic_dec_and_test(&vbc_refcnt.ad3_on)) {
			vbc_ad23_enable(0, 1);
			pr_info("VBC AD3 OFF\n");
		}
		if (atomic_read(&vbc_refcnt.ad3_on) < 0) {
			atomic_set(&vbc_refcnt.ad3_on, 0);
		}
	}
	vbc_dbg("------ad3 ref: %d", atomic_read(&vbc_refcnt.ad3_on));

	return 0;
}

static inline int vbc_enable(int enable)
{
	if (enable) {
		atomic_inc(&vbc_refcnt.vbc_on);
		if (atomic_read(&vbc_refcnt.vbc_on) == 1) {
			vbc_enable_set(1);
			pr_info("VBC Enable\n");
		}
	} else {
		if (atomic_dec_and_test(&vbc_refcnt.vbc_on)) {
			vbc_enable_set(0);
			pr_info("VBC Disablen");
		}
		if (atomic_read(&vbc_refcnt.vbc_on) < 0) {
			atomic_set(&vbc_refcnt.vbc_on, 0);
		}
	}

	vbc_dbg("------vbc en ref: %d", atomic_read(&vbc_refcnt.vbc_on));
	return 0;
}

static inline int vbc_power_enable(int enable)
{
	if (enable) {
		atomic_inc(&vbc_refcnt.vbc_power_on);
		if (atomic_read(&vbc_refcnt.vbc_power_on) == 1) {
			arch_audio_vbc_enable();
			vbc_reg_enable();
			pr_info("VBC Power ON\n");
		}
	} else {
		if (atomic_dec_and_test(&vbc_refcnt.vbc_power_on)) {
			arch_audio_vbc_reset();
			arch_audio_vbc_disable();
			vbc_reg_disable();
			pr_info("VBC Power Off\n");
		}
		if (atomic_read(&vbc_refcnt.vbc_power_on) < 0) {
			atomic_set(&vbc_refcnt.vbc_power_on, 0);
		}
	}
	vbc_dbg("------vbc power ref: %d",
		atomic_read(&vbc_refcnt.vbc_power_on));
	return 0;
}

static inline int vbc_ad0_dma_set(int enable)
{
	vbc_reg_write(VBDABUFFDTA, ((enable ? 1 : 0) << VBAD0DMA_EN),
		      (1 << VBAD0DMA_EN));
	return 0;
}

static inline int vbc_ad1_dma_set(int enable)
{
	vbc_reg_write(VBDABUFFDTA, ((enable ? 1 : 0) << VBAD1DMA_EN),
		      (1 << VBAD1DMA_EN));
	return 0;
}

static inline int vbc_ad2_dma_set(int enable)
{
	vbc_reg_write(VBADDMA, ((enable ? 1 : 0) << VBAD2DMA_EN),
		      (1 << VBAD2DMA_EN));
	return 0;
}

static inline int vbc_ad3_dma_set(int enable)
{
	vbc_reg_write(VBADDMA, ((enable ? 1 : 0) << VBAD3DMA_EN),
		      (1 << VBAD3DMA_EN));
	return 0;
}

static inline int vbc_da0_dma_set(int enable)
{
	vbc_reg_write(VBDABUFFDTA, ((enable ? 1 : 0) << VBDA0DMA_EN),
		      (1 << VBDA0DMA_EN));
	return 0;
}

static inline int vbc_da1_dma_set(int enable)
{
	vbc_reg_write(VBDABUFFDTA, ((enable ? 1 : 0) << VBDA1DMA_EN),
		      (1 << VBDA1DMA_EN));
	return 0;
}

static void vbc_da_buffer_clear(int id)
{
	int i;
	vbc_reg_write(VBDABUFFDTA, ((id ? 1 : 0) << RAMSW_NUMB),
		      (1 << RAMSW_NUMB));
	for (i = 0; i < VBC_FIFO_FRAME_NUM; i++) {
		vbc_reg_write2(VBDA0, 0);
		vbc_reg_write2(VBDA1, 0);
	}
}

static void vbc_da_buffer_clear_all(struct snd_soc_dai *dai)
{
	int ret;

	ret = vbc_da0_enable(1);
	if (ret < 0) {
		pr_err("Failed to enable VBC DA0\n");
	}
	ret = vbc_da1_enable(1);
	if (ret < 0) {
		pr_err("Failed to enable VBC DA1\n");
	}

	vbc_sw_write_buffer(true);
	vbc_set_buffer_size(0, VBC_FIFO_FRAME_NUM, 0);
	vbc_da_buffer_clear(1);	/* clear data buffer 1 */
	vbc_da_buffer_clear(0);	/* clear data buffer 0 */
	vbc_sw_write_buffer(false);

	vbc_da0_enable(0);
	vbc_da1_enable(0);
}

static inline int vbc_str_2_index(int stream);
static int vbc_da_arch_enable(int chan)
{
	int ret;
	if (chan == VBC_LEFT)
		ret = vbc_da0_enable(1);
	else
		ret = vbc_da1_enable(1);
	if (ret < 0) {
		pr_err("VBC da enable error:%i\n", ret);
		return ret;
	} else {
		vbc[vbc_str_2_index(SNDRV_PCM_STREAM_PLAYBACK)].is_active = 1;
	}
	return ret;
}

static int vbc_da_arch_disable(int chan)
{
	int ret;
	if (chan == VBC_LEFT)
		ret = vbc_da0_enable(0);
	else
		ret = vbc_da1_enable(0);
	if (ret < 0) {
		pr_err("VBC da disable error:%i\n", ret);
		return ret;
	} else {
		vbc[vbc_str_2_index(SNDRV_PCM_STREAM_PLAYBACK)].is_active = 0;
	}
	return ret;
}

static int vbc_ad_arch_enable(int chan)
{
	int ret;
	if (chan == VBC_LEFT)
		ret = vbc_ad0_enable(1);
	else
		ret = vbc_ad1_enable(1);
	if (ret < 0) {
		pr_err("VBC ad enable error:%i\n", ret);
		return ret;
	} else {
		vbc[vbc_str_2_index(SNDRV_PCM_STREAM_CAPTURE)].is_active = 1;
	}
	return ret;
}

static int vbc_ad_arch_disable(int chan)
{
	int ret;
	if (chan == VBC_LEFT)
		ret = vbc_ad0_enable(0);
	else
		ret = vbc_ad1_enable(0);
	if (ret < 0) {
		pr_err("VBC ad disable error:%i\n", ret);
		return ret;
	} else {
		vbc[vbc_str_2_index(SNDRV_PCM_STREAM_CAPTURE)].is_active = 0;
	}
	return ret;
}

static int vbc_ad23_arch_enable(int chan)
{
	int ret;
	if (chan == VBC_LEFT)
		ret = vbc_ad2_enable(1);
	else
		ret = vbc_ad3_enable(1);
	if (ret < 0) {
		pr_err("VBC ad enable error:%i\n", ret);
		return ret;
	} else {
		vbc[vbc_str_2_index(SNDRV_PCM_STREAM_CAPTURE) + 1].is_active =
		    1;
	}
	return ret;
}

static int vbc_ad23_arch_disable(int chan)
{
	int ret;
	if (chan == VBC_LEFT)
		ret = vbc_ad2_enable(0);
	else
		ret = vbc_ad3_enable(0);
	if (ret < 0) {
		pr_err("VBC ad23 disable error:%i\n", ret);
		return ret;
	} else {
		vbc[vbc_str_2_index(SNDRV_PCM_STREAM_CAPTURE) + 1].is_active =
		    0;
	}
	return ret;
}

static inline int vbc_da0_dg_set(int enable, int dg)
{
	if (enable) {
		vbc_reg_write(DADGCTL, 0x80 | (0xFF & dg), 0xFF);
	} else {
		vbc_reg_write(DADGCTL, 0, 0x80);
	}
	return 0;
}

static inline int vbc_da1_dg_set(int enable, int dg)
{
	if (enable) {
		vbc_reg_write(DADGCTL, (0x80 | (0xFF & dg)) << 8, 0xFF00);
	} else {
		vbc_reg_write(DADGCTL, 0, 0x8000);
	}
	return 0;
}

static inline int vbc_ad0_dg_set(int enable, int dg)
{
	if (enable) {
		vbc_reg_write(ADDG01CTL, 0x80 | (0xFF & dg), 0xFF);
	} else {
		vbc_reg_write(ADDG01CTL, 0, 0x80);
	}
	return 0;
}

static inline int vbc_ad1_dg_set(int enable, int dg)
{
	if (enable) {
		vbc_reg_write(ADDG01CTL, (0x80 | (0xFF & dg)) << 8, 0xFF00);
	} else {
		vbc_reg_write(ADDG01CTL, 0, 0x8000);
	}
	return 0;
}

static inline int vbc_ad2_dg_set(int enable, int dg)
{
	if (enable) {
		vbc_reg_write(ADDG23CTL, 0x80 | (0xFF & dg), 0xFF);
	} else {
		vbc_reg_write(ADDG23CTL, 0, 0x80);
	}
	return 0;
}

static inline int vbc_ad3_dg_set(int enable, int dg)
{
	if (enable) {
		vbc_reg_write(ADDG23CTL, (0x80 | (0xFF & dg)) << 8, 0xFF00);
	} else {
		vbc_reg_write(ADDG23CTL, 0, 0x8000);
	}
	return 0;
}

static inline int vbc_st0_dg_set(int enable, int dg)
{
	if (enable) {
		vbc_reg_write(STCTL0, (0x80 | (0xFF & dg)) << 4, 0xFF0);
	} else {
		vbc_reg_write(STCTL0, 0, 0x800);
	}
	return 0;
}

static inline int vbc_st1_dg_set(int enable, int dg)
{
	if (enable) {
		vbc_reg_write(STCTL1, (0x80 | (0xFF & dg)) << 8, 0xFF0);
	} else {
		vbc_reg_write(STCTL1, 0, 0x800);
	}
	return 0;
}

static int vbc_try_dg_set(int vbc_idx, int id)
{
	int dg = vbc[vbc_idx].dg_val[id];
	if (vbc[vbc_idx].dg_switch[id]) {
		vbc[vbc_idx].dg_set[id] (1, dg);
	} else {
		vbc[vbc_idx].dg_set[id] (0, dg);
	}
	return 0;
}

static int vbc_try_st_dg_set(int id)
{
	if (id == VBC_LEFT) {
		vbc_st0_dg_set(st_dg.dg_switch[id], st_dg.dg_val[id]);
	} else {
		vbc_st1_dg_set(st_dg.dg_switch[id], st_dg.dg_val[id]);
	}
	return 0;
}

static int vbc_try_da_iismux_set(void)
{
	vbc_da_iismux_set(vbc_da_iis_port == 0 ?
			  0 : ((vbc_da_iis_port == 1) ? 2 : 3));
	return 0;
}

static int vbc_try_ad_dgmux_set(int id)
{
	switch (id) {
	case ADC0_DGMUX:
		vbc_ad0_dgmux_set(adc_dgmux_val[ADC0_DGMUX]);
		break;
	case ADC1_DGMUX:
		vbc_ad1_dgmux_set(adc_dgmux_val[ADC1_DGMUX]);
		break;
	case ADC2_DGMUX:
		vbc_ad2_dgmux_set(adc_dgmux_val[ADC2_DGMUX]);
		break;
	case ADC3_DGMUX:
		vbc_ad3_dgmux_set(adc_dgmux_val[ADC3_DGMUX]);
		break;
	default:
		break;
	}
	return 0;
}

int dig_fm_event(struct snd_soc_dapm_widget *w,
		 struct snd_kcontrol *k, int event)
{
	vbc_dbg("Entering %s switch %s\n", __func__,
		SND_SOC_DAPM_EVENT_ON(event) ? "ON" : "OFF");
	vbc_try_st_dg_set(VBC_LEFT);
	vbc_try_st_dg_set(VBC_RIGHT);
	vbc_enable(! !SND_SOC_DAPM_EVENT_ON(event));
	vbc_dbg("Leaving %s\n", __func__);
	return 0;
}

EXPORT_SYMBOL_GPL(dig_fm_event);

static const char *get_event_name(int event)
{
	const char *ev_name;
	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		ev_name = "PRE_PMU";
		break;
	case SND_SOC_DAPM_POST_PMU:
		ev_name = "POST_PMU";
		break;
	case SND_SOC_DAPM_PRE_PMD:
		ev_name = "PRE_PMD";
		break;
	case SND_SOC_DAPM_POST_PMD:
		ev_name = "POST_PMD";
		break;
	default:
		BUG();
		return 0;
	}
	return ev_name;
}

static const char *st_chan_sel_txt[] = {
	"AD01", "AD23"
};

static const char *st0_sel_txt[] = {
	"AD0(2)ST0", "AD1(3)ST0", "NOINPUT",
};

static const char *st1_sel_txt[] = {
	"AD1(3)ST1", "AD0(2)ST1", "NOINPUT",
};

static const char *ad0_inmux_txt[] = {
	"IIS0AD0", "IIS1AD0", "NOINPUT",
};

static const char *ad1_inmux_txt[] = {
	"IIS1AD1", "IIS0AD1", "NOINPUT",
};

static const char *ad2_inmux_txt[] = {
	"IIS2AD2", "IIS3AD2", "NOINPUT",
};

static const char *ad3_inmux_txt[] = {
	"IIS3AD3", "IIS2AD3", "NOINPUT",
};

static const char *ad_iis_txt[] = {
	"AUDIIS0", "DIGFM", "EXTDIGFM", "EXTIIS6", "AUDIIS1",
};

static const char *ad23_iis_txt[] = {
	"AUDIIS1", "DIGFM", "EXTDIGFM", "EXTIIS6", "AUDIIS0",
};

#define SPRD_VBC_ENUM(xreg, xmax, xtexts)\
		  SOC_ENUM_SINGLE(FUN_REG(xreg), 0, xmax, xtexts)

static const struct soc_enum vbc_mux_sel_enum[SPRD_VBC_MUX_MAX] = {
	/*ST CHAN MUX */
	SPRD_VBC_ENUM(SPRD_VBC_ST0_CHAN_MUX, 2, st_chan_sel_txt),
	SPRD_VBC_ENUM(SPRD_VBC_ST1_CHAN_MUX, 2, st_chan_sel_txt),
	/*ST INMUX */
	SPRD_VBC_ENUM(SPRD_VBC_ST0_MUX, 4, st0_sel_txt),
	SPRD_VBC_ENUM(SPRD_VBC_ST1_MUX, 4, st1_sel_txt),
	/*AD INMUX */
	SPRD_VBC_ENUM(SPRD_VBC_AD0_INMUX, 4, ad0_inmux_txt),
	SPRD_VBC_ENUM(SPRD_VBC_AD1_INMUX, 4, ad1_inmux_txt),
	SPRD_VBC_ENUM(SPRD_VBC_AD2_INMUX, 4, ad2_inmux_txt),
	SPRD_VBC_ENUM(SPRD_VBC_AD3_INMUX, 4, ad3_inmux_txt),
	/*IIS INMUX */
	SPRD_VBC_ENUM(SPRD_VBC_AD_IISMUX, 5, ad_iis_txt),
	SPRD_VBC_ENUM(SPRD_VBC_AD23_IISMUX, 5, ad23_iis_txt)
};

static int vbc_da0_event(struct snd_soc_dapm_widget *w,
			 struct snd_kcontrol *kcontrol, int event)
{
	int ret = 0;

	vbc_dbg("Entering %s event is %s\n", __func__, get_event_name(event));

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		vbc_da0_enable(1);
		break;
	case SND_SOC_DAPM_POST_PMD:
		vbc_da0_enable(0);
		break;
	default:
		BUG();
		ret = -EINVAL;
		break;
	}
	vbc_dbg("Leaving %s\n", __func__);

	return ret;
}

static int vbc_da1_event(struct snd_soc_dapm_widget *w,
			 struct snd_kcontrol *kcontrol, int event)
{
	int ret = 0;

	vbc_dbg("Entering %s event is %s\n", __func__, get_event_name(event));

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		vbc_da1_enable(1);
		break;
	case SND_SOC_DAPM_POST_PMD:
		vbc_da1_enable(0);
		break;
	default:
		BUG();
		ret = -EINVAL;
		break;
	}

	vbc_dbg("Leaving %s\n", __func__);

	return ret;
}

static int vbc_ad0_event(struct snd_soc_dapm_widget *w,
			 struct snd_kcontrol *kcontrol, int event)
{
	int ret = 0;

	vbc_dbg("Entering %s event is %s\n", __func__, get_event_name(event));

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		vbc_ad0_enable(1);
		break;
	case SND_SOC_DAPM_POST_PMD:
		vbc_ad0_enable(0);
		break;
	default:
		BUG();
		ret = -EINVAL;
		break;
	}

	vbc_dbg("Leaving %s\n", __func__);

	return ret;
}

static int vbc_ad1_event(struct snd_soc_dapm_widget *w,
			 struct snd_kcontrol *kcontrol, int event)
{
	int ret = 0;

	vbc_dbg("Entering %s event is %s\n", __func__, get_event_name(event));

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		vbc_ad1_enable(1);
		break;
	case SND_SOC_DAPM_POST_PMD:
		vbc_ad1_enable(0);
		break;
	default:
		BUG();
		ret = -EINVAL;
		break;
	}

	vbc_dbg("Leaving %s\n", __func__);

	return ret;
}

static int vbc_ad2_event(struct snd_soc_dapm_widget *w,
			 struct snd_kcontrol *kcontrol, int event)
{
	int ret = 0;

	vbc_dbg("Entering %s event is %s\n", __func__, get_event_name(event));

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		vbc_ad2_enable(1);
		break;
	case SND_SOC_DAPM_POST_PMD:
		vbc_ad2_enable(0);
		break;
	default:
		BUG();
		ret = -EINVAL;
		break;
	}

	vbc_dbg("Leaving %s\n", __func__);

	return ret;
}

static int vbc_ad3_event(struct snd_soc_dapm_widget *w,
			 struct snd_kcontrol *kcontrol, int event)
{
	int ret = 0;

	vbc_dbg("Entering %s event is %s\n", __func__, get_event_name(event));

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		vbc_ad3_enable(1);
		break;
	case SND_SOC_DAPM_POST_PMD:
		vbc_ad3_enable(0);
		break;
	default:
		BUG();
		ret = -EINVAL;
		break;
	}

	vbc_dbg("Leaving %s\n", __func__);

	return ret;
}

static int vbc_power_event(struct snd_soc_dapm_widget *w,
			   struct snd_kcontrol *kcontrol, int event)
{
	int ret = 0;

	vbc_dbg("Entering %s event is %s\n", __func__, get_event_name(event));

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		vbc_power_enable(1);
		break;
	case SND_SOC_DAPM_POST_PMD:
		vbc_power_enable(0);
		break;
	default:
		BUG();
		ret = -EINVAL;
		break;
	}

	vbc_dbg("Leaving %s\n", __func__);
	return ret;
}

static int mux_event(struct snd_soc_dapm_widget *w,
		     struct snd_kcontrol *kcontrol, int event)
{
	unsigned int id = FUN_REG(w->reg);
	struct sprd_vbc_mux_op *mux = &(sprd_vbc_mux[id]);
	int ret = 0;

	vbc_dbg("Entering %s set %s(%d) event is %s\n", __func__,
		vbc_mux_debug_str[id], mux->val, get_event_name(event));

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		mux->set = vbc_mux_cfg[id];
		ret = mux->set(mux->val);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		mux->set = 0;
		ret = vbc_mux_cfg[id] (0);
		break;
	default:
		BUG();
		ret = -EINVAL;
	}

	vbc_dbg("Leaving %s\n", __func__);

	return ret;
}

int sprd_vbc_mux_get(struct snd_kcontrol *kcontrol,
		     struct snd_ctl_elem_value *ucontrol)
{
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int reg = FUN_REG(e->reg);
	struct sprd_vbc_mux_op *mux = &(sprd_vbc_mux[reg]);

	ucontrol->value.enumerated.item[0] = mux->val;

	return 0;
}

int sprd_vbc_mux_put(struct snd_kcontrol *kcontrol,
		     struct snd_ctl_elem_value *ucontrol)
{
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int reg = FUN_REG(e->reg);
	unsigned int max = e->max;
	unsigned int mask = (1 << fls(max)) - 1;
	struct sprd_vbc_mux_op *mux = &(sprd_vbc_mux[reg]);
	int ret = 0;

	pr_info("set MUX[%s] to %d\n", vbc_mux_debug_str[reg],
		ucontrol->value.enumerated.item[0]);

	if (mux->val == ucontrol->value.enumerated.item[0])
		return 0;
	if (ucontrol->value.enumerated.item[0] > e->max - 1)
		return -EINVAL;

	ret = snd_soc_dapm_put_enum_double(kcontrol, ucontrol);

	mux->val = (ucontrol->value.enumerated.item[0] & mask);
	if (mux->set) {
		ret = mux->set(mux->val);
	}
	vbc_dbg("Leaving %s\n", __func__);
	return ret;
}

#define SPRD_VBC_MUX(xname, xenum) \
	 SOC_DAPM_ENUM_EXT(xname, xenum, sprd_vbc_mux_get, sprd_vbc_mux_put)

static const struct snd_kcontrol_new vbc_mux[SPRD_VBC_MUX_MAX] = {
	SPRD_VBC_MUX("ST0 CHAN MUX", vbc_mux_sel_enum[SPRD_VBC_ST0_CHAN_MUX]),
	SPRD_VBC_MUX("ST1 CHAN MUX", vbc_mux_sel_enum[SPRD_VBC_ST1_CHAN_MUX]),
	SPRD_VBC_MUX("ST0 INMUX", vbc_mux_sel_enum[SPRD_VBC_ST0_MUX]),
	SPRD_VBC_MUX("ST1 INMUX", vbc_mux_sel_enum[SPRD_VBC_ST1_MUX]),
	SPRD_VBC_MUX("AD0 INMUX", vbc_mux_sel_enum[SPRD_VBC_AD0_INMUX]),
	SPRD_VBC_MUX("AD1 INMUX", vbc_mux_sel_enum[SPRD_VBC_AD1_INMUX]),
	SPRD_VBC_MUX("AD2 INMUX", vbc_mux_sel_enum[SPRD_VBC_AD2_INMUX]),
	SPRD_VBC_MUX("AD3 INMUX", vbc_mux_sel_enum[SPRD_VBC_AD3_INMUX]),
	SPRD_VBC_MUX("AD IISMUX", vbc_mux_sel_enum[SPRD_VBC_AD_IISMUX]),
	SPRD_VBC_MUX("AD23 IISMUX", vbc_mux_sel_enum[SPRD_VBC_AD23_IISMUX])
};

#define VBC_DAPM_MUX_E(wname, wreg) \
	SND_SOC_DAPM_MUX_E(wname, FUN_REG(wreg), 0, 0, &vbc_mux[wreg], mux_event, \
							SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD)

static const struct snd_soc_dapm_widget vbc_dapm_widgets[] = {
	/*power */
	SND_SOC_DAPM_SUPPLY("VBC Power", SND_SOC_NOPM, 0, 0,
			    vbc_power_event,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	/* STchan mux */
	VBC_DAPM_MUX_E("ST0 CHAN MUX", SPRD_VBC_ST0_CHAN_MUX),
	VBC_DAPM_MUX_E("ST1 CHAN MUX", SPRD_VBC_ST1_CHAN_MUX),

	/* ST inmux */
	VBC_DAPM_MUX_E("ST0 INMUX", SPRD_VBC_ST0_MUX),
	VBC_DAPM_MUX_E("ST1 INMUX", SPRD_VBC_ST1_MUX),

	/*ADC inmux */
	VBC_DAPM_MUX_E("AD0 INMUX", SPRD_VBC_AD0_INMUX),
	VBC_DAPM_MUX_E("AD1 INMUX", SPRD_VBC_AD1_INMUX),
	VBC_DAPM_MUX_E("AD2 INMUX", SPRD_VBC_AD2_INMUX),
	VBC_DAPM_MUX_E("AD3 INMUX", SPRD_VBC_AD3_INMUX),

	/*IIS MUX */
	VBC_DAPM_MUX_E("AD IISMUX", SPRD_VBC_AD_IISMUX),
	VBC_DAPM_MUX_E("AD23 IISMUX", SPRD_VBC_AD23_IISMUX),

	/*ST Switch */
	SND_SOC_DAPM_PGA_S("ST0 Switch", 4, SOC_REG(STCTL0), VBST_EN_0, 0, NULL,
			   0),
	SND_SOC_DAPM_PGA_S("ST1 Switch", 4, SOC_REG(STCTL1), VBST_EN_1, 0, NULL,
			   0),

	/*FM Mixer */
	SND_SOC_DAPM_PGA_S("DA0 FM Mixer", 4, SOC_REG(DAPATCHCTL),
			   VBDAPATH_DA0_ADDFM_SHIFT, 0, NULL, 0),
	SND_SOC_DAPM_PGA_S("DA1 FM Mixer", 4, SOC_REG(DAPATCHCTL),
			   VBDAPATH_DA1_ADDFM_SHIFT, 0, NULL, 0),
	SND_SOC_DAPM_PGA_S("DFM", 4, SND_SOC_NOPM, 0, 0, NULL, 0),

	/*VBC Chan Switch */
	SND_SOC_DAPM_PGA_S("DA0 Switch", 5, SND_SOC_NOPM, 0, 0,
			   vbc_da0_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_PGA_S("DA1 Switch", 5, SND_SOC_NOPM, 0, 0,
			   vbc_da1_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_PGA_S("AD0 Switch", 5, SND_SOC_NOPM, 0, 0,
			   vbc_ad0_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_PGA_S("AD1 Switch", 5, SND_SOC_NOPM, 0, 0,
			   vbc_ad1_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_PGA_S("AD2 Switch", 5, SND_SOC_NOPM, 0, 0,
			   vbc_ad2_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_PGA_S("AD3 Switch", 5, SND_SOC_NOPM, 0, 0,
			   vbc_ad3_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
};

/* sprd_vbc supported interconnection*/
static const struct snd_soc_dapm_route vbc_intercon[] = {
	/************************power********************************/
	/* digital fm playback need to open DA Clk and DA power */
	{"DFM", NULL, "VBC Power"},
	{"DFM", NULL, "DA Clk"},

	/********************** capture in  path in vbc ********************/
	/*AD input route */
	/*AD01 */
	{"AD IISMUX", "DIGFM", "Dig FM Jack"},
	{"AD IISMUX", "EXTDIGFM", "Dig FM Jack"},

	/*AD23 */
	{"AD23 IISMUX", "DIGFM", "Dig FM Jack"},
	{"AD23 IISMUX", "EXTDIGFM", "Dig FM Jack"},

	{"AD0 INMUX", "IIS0AD0", "AD IISMUX"},
	{"AD0 INMUX", "IIS1AD0", "AD IISMUX"},
	{"AD1 INMUX", "IIS1AD1", "AD IISMUX"},
	{"AD1 INMUX", "IIS0AD1", "AD IISMUX"},
	{"AD2 INMUX", "IIS2AD2", "AD23 IISMUX"},
	{"AD2 INMUX", "IIS3AD2", "AD23 IISMUX"},
	{"AD3 INMUX", "IIS3AD3", "AD23 IISMUX"},
	{"AD3 INMUX", "IIS2AD3", "AD23 IISMUX"},

	{"AD0 Switch", NULL, "AD0 INMUX"},
	{"AD1 Switch", NULL, "AD1 INMUX"},
	{"AD2 Switch", NULL, "AD2 INMUX"},
	{"AD3 Switch", NULL, "AD3 INMUX"},

	/********************** fm playback route ********************/
	/*ST route */
	{"ST0 CHAN MUX", "AD01", "AD0 Switch"},
	{"ST0 CHAN MUX", "AD01", "AD1 Switch"},
	{"ST0 CHAN MUX", "AD23", "AD2 Switch"},
	{"ST0 CHAN MUX", "AD23", "AD3 Switch"},
	{"ST1 CHAN MUX", "AD01", "AD0 Switch"},
	{"ST1 CHAN MUX", "AD01", "AD1 Switch"},
	{"ST1 CHAN MUX", "AD23", "AD2 Switch"},
	{"ST1 CHAN MUX", "AD23", "AD3 Switch"},

	{"ST0 INMUX", "AD0(2)ST0", "ST0 CHAN MUX"},
	{"ST0 INMUX", "AD1(3)ST0", "ST0 CHAN MUX"},
	{"ST1 INMUX", "AD1(3)ST1", "ST1 CHAN MUX"},
	{"ST1 INMUX", "AD0(2)ST1", "ST1 CHAN MUX"},

	{"ST0 Switch", NULL, "ST0 INMUX"},
	{"ST1 Switch", NULL, "ST1 INMUX"},
	{"DA0 Switch", NULL, "ST0 Switch"},
	{"DA1 Switch", NULL, "ST1 Switch"},
	{"DA0 FM Mixer", NULL, "DA0 Switch"},
	{"DA1 FM Mixer", NULL, "DA1 Switch"},
	{"DFM", NULL, "DA0 FM Mixer"},
	{"DFM", NULL, "DA1 FM Mixer"},
	{"Digital DACL Switch", NULL, "DFM"},
	{"Digital DACR Switch", NULL, "DFM"},
};

static struct vbc_priv vbc[3] = {
	{			/*PlayBack */
	 .dma_set = {vbc_da0_dma_set, vbc_da1_dma_set},
	 .arch_enable = vbc_da_arch_enable,
	 .arch_disable = vbc_da_arch_disable,
	 .is_active = 0,
	 .dg_switch = {0, 0},
	 .dg_val = {0x18, 0x18},
	 .dg_set = {vbc_da0_dg_set, vbc_da1_dg_set},
	 },
	{			/*Capture for ad01 */
	 .dma_set = {vbc_ad0_dma_set, vbc_ad1_dma_set},
	 .arch_enable = vbc_ad_arch_enable,
	 .arch_disable = vbc_ad_arch_disable,
	 .is_active = 0,
	 .dg_switch = {0, 0},
	 .dg_val = {0x18, 0x18},
	 .dg_set = {vbc_ad0_dg_set, vbc_ad1_dg_set},
	 },
	{			/*Capture for ad23 */
	 .dma_set = {vbc_ad2_dma_set, vbc_ad3_dma_set},
	 .arch_enable = vbc_ad23_arch_enable,
	 .arch_disable = vbc_ad23_arch_disable,
	 .is_active = 0,
	 .dg_switch = {0, 0},
	 .dg_val = {0x18, 0x18},
	 .dg_set = {vbc_ad2_dg_set, vbc_ad3_dg_set},
	 },
};

/* NOTE:
   this index need use for the [struct vbc_priv] vbc[2] index
   default MUST return 0.
 */
static inline int vbc_str_2_index(int stream)
{
	if (stream == SNDRV_PCM_STREAM_CAPTURE) {
		return 1;
	}
	return 0;
}

static int vbc_startup(struct snd_pcm_substream *substream,
		       struct snd_soc_dai *dai)
{
	int vbc_idx;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;

	vbc_dbg("Entering %s\n", __func__);
	vbc_idx = vbc_str_2_index(substream->stream);
	/*check SNDRV_PCM_STREAM_CAPTURE :ad01 or ad23  */
	if (!strcmp(dai->name, "vbc-ad23"))
		vbc_idx += 1;
	vbc_dbg("vbc_idx:%d\n", vbc_idx);
	if (vbc[vbc_idx].is_open || vbc[vbc_idx].is_active) {
		pr_err("vbc is actived:%d\n", substream->stream);
	}

	mutex_lock(&vbc_mutex);
	vbc[vbc_idx].is_open = 1;
	mutex_unlock(&vbc_mutex);
	vbc_power_enable(1);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		vbc_da_buffer_clear_all(dai);
		vbc_set_buffer_size(0, VBC_FIFO_FRAME_NUM, 0);
		vbc_try_da_iismux_set();
	} else if (vbc_idx == 1) {
		vbc_set_buffer_size(VBC_FIFO_FRAME_NUM, 0, 0);
		vbc_try_ad_dgmux_set(0);
		vbc_try_ad_dgmux_set(1);
	} else {
		vbc_set_buffer_size(0, 0, VBC_FIFO_FRAME_NUM);
		vbc_try_ad_dgmux_set(2);
		vbc_try_ad_dgmux_set(3);
	}

	vbc_eq_setting.codec_dai = codec_dai;
	if (vbc_eq_setting.is_active[vbc_idx])
		vbc_eq_try_apply(codec_dai, vbc_eq_setting.vbc_idx);

	vbc_try_dg_set(vbc_idx, VBC_LEFT);
	vbc_try_dg_set(vbc_idx, VBC_RIGHT);

	WARN_ON(!vbc[vbc_idx].arch_enable);
	WARN_ON(!vbc[vbc_idx].arch_disable);
	WARN_ON(!vbc[vbc_idx].dma_set[0]);
	WARN_ON(!vbc[vbc_idx].dma_set[1]);

	vbc_dbg("Leaving %s\n", __func__);
	return 0;
}

static inline int vbc_can_close(void)
{
	return !(vbc[vbc_str_2_index(SNDRV_PCM_STREAM_PLAYBACK)].is_open
		 || vbc[vbc_str_2_index(SNDRV_PCM_STREAM_CAPTURE)].is_open
		 ||
		 vbc[(vbc_str_2_index(SNDRV_PCM_STREAM_CAPTURE) + 1)].is_open);
}

static void vbc_shutdown(struct snd_pcm_substream *substream,
			 struct snd_soc_dai *dai)
{
	int vbc_idx;

	vbc_dbg("Entering %s\n", __func__);

	/* vbc da close MUST clear da buffer */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		vbc_da_buffer_clear_all(dai);
		vbc_eq_setting.codec_dai = 0;
	}

	vbc_idx = vbc_str_2_index(substream->stream);
	/*check SNDRV_PCM_STREAM_CAPTURE :ad01 or ad23 */
	if (!strcmp(dai->name, "vbc-ad23"))
		vbc_idx += 1;
	vbc_dbg("vbc_idx:%d\n", vbc_idx);

	mutex_lock(&vbc_mutex);
	vbc[vbc_idx].is_open = 0;
	vbc_power_enable(0);
	mutex_unlock(&vbc_mutex);

	vbc_dbg("Leaving %s\n", __func__);
}

static int vbc_hw_params(struct snd_pcm_substream *substream,
			 struct snd_pcm_hw_params *params,
			 struct snd_soc_dai *dai)
{
	int vbc_idx;
	struct sprd_pcm_dma_params *dma_data;

	vbc_dbg("Entering %s\n", __func__);

	vbc_idx = vbc_str_2_index(substream->stream);
	/*check SNDRV_PCM_STREAM_CAPTURE :ad01 or ad23 */
	if (!strcmp(dai->name, "vbc-ad23"))
		vbc_idx += 1;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		dma_data = &vbc_pcm_stereo_out;
	else
		dma_data =
		    ((vbc_idx ==
		      2) ? (&vbc_pcm23_stereo_in) : (&vbc_pcm_stereo_in));

	snd_soc_dai_set_dma_data(dai, substream, dma_data);

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	default:
		pr_err("vbc only supports format SNDRV_PCM_FORMAT_S16_LE\n");
		break;
	}

	vbc[vbc_idx].used_chan_count = params_channels(params);
	if (vbc[vbc_idx].used_chan_count > 2) {
		pr_err("vbc can not supports grate 2 channels\n");
	}

	if (vbc_idx != 0) {
		if (params_rate(params) == 44100) {
			if ((vbc_idx == 1
			     && sprd_vbc_mux[SPRD_VBC_AD_IISMUX].val == 0)
			    || (vbc_idx == 2
				&& sprd_vbc_mux[SPRD_VBC_AD23_IISMUX].val == 0))
				vbc_adc_src_set(CONFIG_CODEC_SRC_SAMPLE_RATE,
						vbc_idx - 1);
		} else
			vbc_adc_src_set(0, vbc_idx - 1);	/*close adc src */
	}
	vbc_dbg("Leaving %s\n", __func__);
	return 0;
}

static int vbc_trigger(struct snd_pcm_substream *substream, int cmd,
		       struct snd_soc_dai *dai)
{
	int vbc_idx;
	int ret = 0;
	int i;

#if 0
	vbc_dbg("Entering %s\n", __func__);
#endif

	vbc_idx = vbc_str_2_index(substream->stream);
	if (!strcmp(dai->name, "vbc-ad23"))
		vbc_idx += 1;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		for (i = 0; i < vbc[vbc_idx].used_chan_count; i++) {
			vbc[vbc_idx].arch_enable(i);
			vbc[vbc_idx].dma_set[i] (1);
		}
		vbc_enable(1);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		for (i = 0; i < vbc[vbc_idx].used_chan_count; i++) {
			vbc[vbc_idx].arch_disable(i);
			vbc[vbc_idx].dma_set[i] (0);
		}
		vbc_enable(0);
		break;
	default:
		ret = -EINVAL;
		break;
	}

#if 0
	vbc_dbg("Leaving %s\n", __func__);
#endif
	return ret;
}

static struct snd_soc_dai_ops vbc_dai_ops = {
	.startup = vbc_startup,
	.shutdown = vbc_shutdown,
	.hw_params = vbc_hw_params,
	.trigger = vbc_trigger,
};

struct snd_soc_dai_driver vbc_dai[] = {
	{
	 .name = "vbc",
	 .playback = {
		      .channels_min = 1,
		      .channels_max = 2,
		      .rates = SNDRV_PCM_RATE_CONTINUOUS,
		      .rate_max = 96000,
		      .formats = SNDRV_PCM_FMTBIT_S16_LE,
		      },
	 .capture = {
		     .channels_min = 1,
		     .channels_max = 2,	/*ad01 */
		     .rates = SNDRV_PCM_RATE_CONTINUOUS,
		     .rate_max = 96000,
		     .formats = SNDRV_PCM_FMTBIT_S16_LE,
		     },
	 .ops = &vbc_dai_ops,
	 },
	{
	 .name = "vbc-ad23",
	 .capture = {
		     .channels_min = 1,
		     .channels_max = 2,	/*ad23 */
		     .rates = SNDRV_PCM_RATE_CONTINUOUS,
		     .rate_max = 96000,
		     .formats = SNDRV_PCM_FMTBIT_S16_LE,
		     },
	 .ops = &vbc_dai_ops,
	 },
};

static int vbc_drv_probe(struct platform_device *pdev)
{
	int i;
	int ret;
	struct clk *vbc_clk;

	vbc_dbg("Entering %s\n", __func__);

	for (i = 0; i < 2; i++) {
#ifdef CONFIG_SPRD_VBC_LR_INVERT
		vbc_pcm_stereo_out.channels[i] =
		    arch_audio_vbc_da_dma_info(1 - i);
#else
		vbc_pcm_stereo_out.channels[i] = arch_audio_vbc_da_dma_info(i);
#endif
		vbc_pcm_stereo_in.channels[i] = arch_audio_vbc_ad_dma_info(i);
		vbc_pcm23_stereo_in.channels[i] =
		    arch_audio_vbc_ad23_dma_info(i);
	}

	vbc_eq_setting.dev = &pdev->dev;

	ret = snd_soc_register_dais(&pdev->dev, vbc_dai, ARRAY_SIZE(vbc_dai));

	if (ret < 0) {
		pr_err("%s err!\n", __func__);
		goto probe_err;
	}

	vbc_clk = clk_get(&pdev->dev, "clk_vbc");
	if (IS_ERR(vbc_clk)) {
		pr_err("Cannot request clk_vbc\n");
	} else {
		s_vbc_clk = vbc_clk;
	}

probe_err:
	vbc_dbg("return %i\n", ret);
	vbc_dbg("Leaving %s\n", __func__);

	return ret;
}

static int __devexit vbc_drv_remove(struct platform_device *pdev)
{
	snd_soc_unregister_dais(&pdev->dev, ARRAY_SIZE(vbc_dai));
	vbc_eq_setting.dev = 0;

	vbc_safe_kfree(&vbc_eq_setting.data[VBC_CHAN_DA]);
	vbc_safe_kfree(&vbc_eq_setting.data[VBC_CHAN_AD01]);
	vbc_safe_kfree(&vbc_eq_setting.data[VBC_CHAN_AD23]);

	if (s_vbc_clk) {
		clk_put(s_vbc_clk);
		s_vbc_clk = 0;
	}
	return 0;
}

static struct platform_driver vbc_driver = {
	.probe = vbc_drv_probe,
	.remove = __devexit_p(vbc_drv_remove),

	.driver = {
		   .name = "vbc",
		   .owner = THIS_MODULE,
		   },
};

/*
 * proc interface
 */

#ifdef CONFIG_PROC_FS
static void vbc_proc_read(struct snd_info_entry *entry,
			  struct snd_info_buffer *buffer)
{
	int reg;

	snd_iprintf(buffer, "vbc register dump\n");
	for (reg = ARM_VB_BASE + 0x10; reg < ARM_VB_END; reg += 0x10) {
		snd_iprintf(buffer, "0x%04x | 0x%04x 0x%04x 0x%04x 0x%04x\n",
			    (reg - ARM_VB_BASE)
			    , vbc_reg_read(reg + 0x00)
			    , vbc_reg_read(reg + 0x04)
			    , vbc_reg_read(reg + 0x08)
			    , vbc_reg_read(reg + 0x0C)
		    );
	}
}

static void vbc_proc_init(struct snd_soc_codec *codec)
{
	struct snd_info_entry *entry;

	if (!snd_card_proc_new(codec->card->snd_card, "vbc", &entry))
		snd_info_set_text_ops(entry, NULL, vbc_proc_read);
}
#else /* !CONFIG_PROC_FS */
static inline void vbc_proc_init(struct snd_soc_codec *codec)
{
}
#endif

static int vbc_da_eq_reg_offset(u32 reg)
{
	int i = 0;
	if (reg == DAPATCHCTL) {
		i = 0;
	} else if ((reg >= DAHPCTL) && (reg <= STCTL1)) {
		i = ((reg - DAHPCTL) >> 2) + 1;
	} else if ((reg >= DACSRCCTL) && (reg <= MIXERCTL)) {
		i = ((reg - DACSRCCTL) >> 2) + ((STCTL1 - DAHPCTL) >> 2) + 2;
	} else if ((reg >= VBNGCVTHD) && (reg <= VBNGCTL)) {
		i = ((reg - VBNGCVTHD) >> 2) + ((MIXERCTL - DACSRCCTL) >> 2) +
		    ((STCTL1 - DAHPCTL) >> 2) + 3;
	} else if ((reg >= HPCOEF0_H) && (reg <= HPCOEF71_L)) {
		i = ((reg - HPCOEF0_H) >> 2) + ((VBNGCTL - VBNGCVTHD) >> 2) +
		    ((MIXERCTL - DACSRCCTL) >> 2) + ((STCTL1 - DAHPCTL) >> 2) +
		    4;
	}
	BUG_ON(i >= VBC_DA_EFFECT_PARAS_LEN);
	return i;
}

static int vbc_ad_eq_reg_offset(u32 reg)
{
	int i = 0;
	if (reg == ADPATCHCTL) {
		i = 0;
	} else if (reg == ADHPCTL) {
		i = 1;
	} else if ((reg >= AD01_HPCOEF0_H) && (reg <= AD01_HPCOEF42_L)) {
		i = ((reg - AD01_HPCOEF0_H) >> 2) + 2;
	} else if ((reg >= AD23_HPCOEF0_H) && (reg <= AD23_HPCOEF42_L)) {
		i = ((reg - AD23_HPCOEF0_H) >> 2) + 2;
	}
	BUG_ON(i >= VBC_AD_EFFECT_PARAS_LEN);
	return i;
}

static inline void vbc_da_eq_reg_set(u32 reg, void *data)
{
	u32 *effect_paras = (u32 *) data;
	vbc_dbg("reg(0x%x) = (0x%x)\n", reg,
		effect_paras[vbc_da_eq_reg_offset(reg)]);
	vbc_reg_write2(reg, effect_paras[vbc_da_eq_reg_offset(reg)]);
}

static inline void vbc_da_eq_reg_set_range(u32 reg_start, u32 reg_end,
					   void *data)
{
	u32 reg_addr;
	for (reg_addr = reg_start; reg_addr <= reg_end; reg_addr += 4) {
		vbc_da_eq_reg_set(reg_addr, data);
	}
}

static inline void vbc_ad_eq_reg_set(u32 reg, void *data)
{
	u32 *effect_paras = (u32 *) data;
	vbc_dbg("reg(0x%x) = (0x%x)\n", reg,
		effect_paras[vbc_ad_eq_reg_offset(reg)]);
	vbc_reg_write2(reg, effect_paras[vbc_ad_eq_reg_offset(reg)]);
}

static inline void vbc_ad_eq_reg_set_range(u32 reg_start, u32 reg_end,
					   void *data)
{
	u32 reg_addr;
	for (reg_addr = reg_start; reg_addr <= reg_end; reg_addr += 4) {
		vbc_ad_eq_reg_set(reg_addr, data);
	}
}

static void vbc_eq_reg_apply(struct snd_soc_dai *codec_dai, void *data,
			     enum vbc_chan chan_id)
{
	if (chan_id == VBC_CHAN_DA) {
		vbc_da_eq_reg_set_range(DAALCCTL0, DAALCCTL10, data);
		vbc_da_eq_reg_set_range(HPCOEF0_H, HPCOEF71_L, data);

		vbc_da_eq_reg_set(DAHPCTL, data);
		vbc_da_eq_reg_set(DAPATCHCTL, data);

		vbc_da_eq_reg_set(STCTL0, data);
		vbc_da_eq_reg_set(STCTL1, data);

		vbc_da_eq_reg_set(DACSRCCTL, data);
		vbc_da_eq_reg_set(MIXERCTL, data);
		vbc_da_eq_reg_set_range(VBNGCVTHD, VBNGCTL, data);
	} else {
		vbc_ad_eq_reg_set(ADHPCTL, data);
		vbc_ad_eq_reg_set(ADPATCHCTL, data);
		if (chan_id == VBC_CHAN_AD01) {
			vbc_ad_eq_reg_set_range(AD01_HPCOEF0_H, AD01_HPCOEF42_L,
						data);
		} else {
			vbc_ad_eq_reg_set_range(AD23_HPCOEF0_H, AD23_HPCOEF42_L,
						data);
		}
	}
}

static void vbc_eq_profile_apply(struct snd_soc_dai *codec_dai, void *data,
				 enum vbc_chan chan_id)
{
	if (vbc_eq_setting.codec_dai) {
		vbc_eq_reg_apply(codec_dai, data, chan_id);
	}
}

static void vbc_eq_profile_close(enum vbc_chan chan_id)
{
	if (chan_id == VBC_CHAN_DA)
		vbc_eq_profile_apply(vbc_eq_setting.codec_dai,
				     &vbc_da_eq_profile_default, VBC_CHAN_DA);
	else if (chan_id == VBC_CHAN_AD01)
		vbc_eq_profile_apply(vbc_eq_setting.codec_dai,
				     &vbc_ad_eq_profile_default, VBC_CHAN_AD01);
	else if (chan_id == VBC_CHAN_AD23)
		vbc_eq_profile_apply(vbc_eq_setting.codec_dai,
				     &vbc_ad_eq_profile_default, VBC_CHAN_AD23);
}

static void vbc_eq_try_apply(struct snd_soc_dai *codec_dai,
			     enum vbc_chan chan_id)
{
	u32 *data;

	vbc_dbg("Entering %s 0x%x\n", __func__,
		(int)vbc_eq_setting.vbc_eq_apply);
	if (vbc_eq_setting.vbc_eq_apply) {
		mutex_lock(&load_mutex);
		if (chan_id == VBC_CHAN_DA) {
			struct vbc_da_eq_profile *now =
			    &(((struct vbc_da_eq_profile *)(vbc_eq_setting.
							    data[VBC_CHAN_DA]))
			      [vbc_eq_setting.now_profile[VBC_CHAN_DA]]);
			data = now->effect_paras;
			pr_info("vbc %s eq apply '%s'\n",
				vbc_get_chan_name(chan_id), now->name);
		} else {
			struct vbc_ad_eq_profile *now =
			    &(((struct vbc_ad_eq_profile *)(vbc_eq_setting.
							    data[chan_id]))
			      [vbc_eq_setting.now_profile[chan_id]]);
			data = now->effect_paras;
			pr_info("vbc %s eq apply '%s'\n",
				vbc_get_chan_name(chan_id), now->name);
		}
		vbc_eq_setting.vbc_eq_apply(codec_dai, data, chan_id);
		mutex_unlock(&load_mutex);
	}
	vbc_dbg("Leaving %s\n", __func__);
}

static int vbc_eq_profile_get(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
	    (struct soc_mixer_control *)kcontrol->private_value;
	int id = FUN_REG(mc->reg);
	ucontrol->value.integer.value[0] = vbc_eq_setting.now_profile[id];
	return 0;
}

static int vbc_eq_profile_put(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	int ret = 0;
	struct soc_mixer_control *mc =
	    (struct soc_mixer_control *)kcontrol->private_value;
	int id = FUN_REG(mc->reg);
	int profile_max;

	profile_max =
	    ((id ==
	      0) ? vbc_eq_setting.hdr.num_da : ((id ==
						 1) ? vbc_eq_setting.hdr.
						num_ad01 : vbc_eq_setting.hdr.
						num_ad23));
	pr_info("vbc %s eq select %ld max %d\n",
		((id == 0) ? "DA" : ((id == 1) ? "AD01" : "AD23")),
		ucontrol->value.integer.value[0], profile_max);

	ret = ucontrol->value.integer.value[0];
	if (ret == vbc_eq_setting.now_profile[id]) {
		return ret;
	}
	if (ret < profile_max) {
		vbc_eq_setting.now_profile[id] = ret;
	}
	if (vbc_eq_setting.is_active[id])
		vbc_eq_try_apply(vbc_eq_setting.codec_dai, id);

	vbc_dbg("Leaving %s\n", __func__);
	return ret;
}

static int vbc_eq_loading(struct snd_soc_codec *codec)
{
	int ret;
	const u8 *fw_data;
	const struct firmware *fw;
	int i;
	int offset = 0;
	int len = 0;
	int old_num_profile;
	int old_num_da;
	int old_num_ad01;
	int old_num_ad23;

	vbc_dbg("Entering %s\n", __func__);
	mutex_lock(&load_mutex);
	vbc_eq_setting.is_loading = 1;

	/* request firmware for VBC EQ */
	ret = request_firmware(&fw, "vbc_eq", vbc_eq_setting.dev);
	if (ret != 0) {
		pr_err("Failed to load firmware: %d\n", ret);
		goto req_fw_err;
	}
	fw_data = fw->data;
	old_num_profile = vbc_eq_setting.hdr.num_profile;
	old_num_da = vbc_eq_setting.hdr.num_da;
	old_num_ad01 = vbc_eq_setting.hdr.num_ad01;
	old_num_ad23 = vbc_eq_setting.hdr.num_ad23;

	memcpy(&vbc_eq_setting.hdr, fw_data, sizeof(vbc_eq_setting.hdr));

	if (strncmp(vbc_eq_setting.hdr.magic, VBC_EQ_FIRMWARE_MAGIC_ID,
		    VBC_EQ_FIRMWARE_MAGIC_LEN)) {
		pr_err("Firmware magic error!\n");
		ret = -EINVAL;
		goto load_err1;
	}

	if (vbc_eq_setting.hdr.profile_version != VBC_EQ_PROFILE_VERSION) {
		pr_err("Firmware support version is 0x%x!\n",
		       VBC_EQ_PROFILE_VERSION);
		ret = -EINVAL;
		goto load_err1;
	}

	if (vbc_eq_setting.hdr.num_profile > VBC_EQ_PROFILE_CNT_MAX) {
		pr_err("Firmware profile to large at %d, max count is %d!\n",
		       vbc_eq_setting.hdr.num_profile, VBC_EQ_PROFILE_CNT_MAX);
		ret = -EINVAL;
		goto load_err1;
	}

	if (vbc_eq_setting.hdr.num_profile !=
	    (vbc_eq_setting.hdr.num_da + vbc_eq_setting.hdr.num_ad01 +
	     vbc_eq_setting.hdr.num_ad23)) {
		pr_err("Firmware profile total number is  wrong!\n");
		ret = -EINVAL;
		goto load_err1;
	}

	len = vbc_eq_setting.hdr.num_da * sizeof(struct vbc_da_eq_profile);

	if (old_num_da != vbc_eq_setting.hdr.num_da) {
		vbc_safe_kfree(&vbc_eq_setting.data[VBC_CHAN_DA]);
	}
	if (vbc_eq_setting.data[VBC_CHAN_DA] == NULL) {
		vbc_eq_setting.data[VBC_CHAN_DA] = kzalloc(len, GFP_KERNEL);
		if (vbc_eq_setting.data[VBC_CHAN_DA] == NULL) {
			ret = -ENOMEM;
			goto load_err1;
		}
	}
	offset += sizeof(struct vbc_fw_header);
	memcpy(vbc_eq_setting.data[VBC_CHAN_DA], fw_data + offset, len);
	for (i = 0; i < vbc_eq_setting.hdr.num_da; i++) {
		if (strncmp
		    (((struct vbc_da_eq_profile *)(vbc_eq_setting.
						   data[VBC_CHAN_DA]))[i].magic,
		     VBC_EQ_FIRMWARE_MAGIC_ID, VBC_EQ_FIRMWARE_MAGIC_LEN)) {
			pr_err
			    ("DA Firmware profile[%d] magic error!magic: %s \n",
			     i,
			     ((struct vbc_da_eq_profile *)(vbc_eq_setting.
							   data[VBC_CHAN_DA]))
			     [i].magic);
			ret = -EINVAL;
			goto eq_err1;
		}
	}

	len = vbc_eq_setting.hdr.num_ad01 * sizeof(struct vbc_ad_eq_profile);

	if (old_num_ad01 != vbc_eq_setting.hdr.num_ad01) {
		vbc_safe_kfree(&vbc_eq_setting.data[VBC_CHAN_AD01]);
	}
	if (vbc_eq_setting.data[VBC_CHAN_AD01] == NULL) {
		vbc_eq_setting.data[VBC_CHAN_AD01] = kzalloc(len, GFP_KERNEL);
		if (vbc_eq_setting.data[VBC_CHAN_AD01] == NULL) {
			ret = -ENOMEM;
			goto load_err2;
		}
	}
	offset += vbc_eq_setting.hdr.num_da * sizeof(struct vbc_da_eq_profile);
	memcpy(vbc_eq_setting.data[VBC_CHAN_AD01], fw_data + offset, len);
	for (i = 0; i < vbc_eq_setting.hdr.num_ad01; i++) {
		if (strncmp
		    (((struct vbc_ad_eq_profile *)(vbc_eq_setting.
						   data[VBC_CHAN_AD01]))[i].
		     magic, VBC_EQ_FIRMWARE_MAGIC_ID,
		     VBC_EQ_FIRMWARE_MAGIC_LEN)) {
			pr_err
			    ("AD01 Firmware profile[%d] magic error!magic:%s \n",
			     i,
			     ((struct vbc_ad_eq_profile *)(vbc_eq_setting.
							   data[VBC_CHAN_AD01]))
			     [i].magic);
			ret = -EINVAL;
			goto eq_err2;
		}
	}

	len = vbc_eq_setting.hdr.num_ad23 * sizeof(struct vbc_ad_eq_profile);

	if (old_num_ad23 != vbc_eq_setting.hdr.num_ad23) {
		vbc_safe_kfree(&vbc_eq_setting.data[VBC_CHAN_AD23]);
	}
	if (vbc_eq_setting.data[VBC_CHAN_AD23] == NULL) {
		vbc_eq_setting.data[VBC_CHAN_AD23] = kzalloc(len, GFP_KERNEL);
		if (vbc_eq_setting.data[VBC_CHAN_AD23] == NULL) {
			ret = -ENOMEM;
			goto load_err3;
		}
	}
	offset +=
	    vbc_eq_setting.hdr.num_ad01 * sizeof(struct vbc_ad_eq_profile);
	memcpy(vbc_eq_setting.data[VBC_CHAN_AD23], fw_data + offset, len);
	for (i = 0; i < vbc_eq_setting.hdr.num_ad23; i++) {
		if (strncmp
		    (((struct vbc_ad_eq_profile *)(vbc_eq_setting.
						   data[VBC_CHAN_AD23]))[i].
		     magic, VBC_EQ_FIRMWARE_MAGIC_ID,
		     VBC_EQ_FIRMWARE_MAGIC_LEN)) {
			pr_err("AD23 Firmware profile[%d] magic error!\n", i);
			ret = -EINVAL;
			goto eq_err3;
		}
	}

	ret = 0;
	goto eq_out;

eq_err3:
	vbc_safe_kfree(&vbc_eq_setting.data[VBC_CHAN_AD23]);
eq_err2:
load_err3:
	vbc_safe_kfree(&vbc_eq_setting.data[VBC_CHAN_AD01]);

eq_err1:
load_err2:
	vbc_safe_kfree(&vbc_eq_setting.data[VBC_CHAN_DA]);
eq_out:
load_err1:
	release_firmware(fw);
req_fw_err:
	vbc_eq_setting.is_loading = 0;
	mutex_unlock(&load_mutex);
	if (ret >= 0) {
		for (i = 0; i <= 2; i++) {
			if (vbc_eq_setting.is_active[i])
				vbc_eq_try_apply(vbc_eq_setting.codec_dai, i);
		}
	}
	vbc_dbg("return %i\n", ret);
	vbc_dbg("Leaving %s\n", __func__);
	return ret;
}

static int vbc_switch_get(struct snd_kcontrol *kcontrol,
			  struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = vbc_control;
	return 0;
}

static int vbc_switch_put(struct snd_kcontrol *kcontrol,
			  struct snd_ctl_elem_value *ucontrol)
{
	struct soc_enum *texts = (struct soc_enum *)kcontrol->private_value;
	pr_info("VBC switch to %s\n",
		texts->texts[ucontrol->value.integer.value[0]]);
	if (vbc_control == ucontrol->value.integer.value[0])
		return 0;

	if (ucontrol->value.integer.value[0] == 3) {	/*switch to cp2-arm */

		arch_audio_vbc_switch(AUDIO_TO_CP2_ARM_CTRL);

#ifdef CONFIG_SPRD_VBC_LR_INVERT
		vbc_pcm_stereo_out.dev_paddr[0] = CP2_PHYS_VBDA1;
		vbc_pcm_stereo_out.dev_paddr[1] = CP2_PHYS_VBDA0;
#else
		vbc_pcm_stereo_out.dev_paddr[0] = CP2_PHYS_VBDA0;
		vbc_pcm_stereo_out.dev_paddr[1] = CP2_PHYS_VBDA1;
#endif
	} else {
		arch_audio_vbc_switch(ucontrol->value.integer.value[0] == 0 ?
				      AUDIO_TO_CP0_DSP_CTRL
				      : ((ucontrol->value.integer.value[0] ==
					  1) ? AUDIO_TO_CP1_DSP_CTRL :
					 AUDIO_TO_AP_ARM_CTRL));
		if (vbc_control == 3 && ucontrol->value.integer.value[0] == 2) {	/*cp2--> ap */
#ifdef CONFIG_SPRD_VBC_LR_INVERT
			vbc_pcm_stereo_out.dev_paddr[0] = PHYS_VBDA1;
			vbc_pcm_stereo_out.dev_paddr[1] = PHYS_VBDA0;
#else
			vbc_pcm_stereo_out.dev_paddr[0] = PHYS_VBDA0;
			vbc_pcm_stereo_out.dev_paddr[1] = PHYS_VBDA1;
#endif
		}
	}
	vbc_control = ucontrol->value.integer.value[0];

	vbc_dbg("Leaving %s\n", __func__);
	return 1;
}

static int vbc_eq_switch_get(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
	    (struct soc_mixer_control *)kcontrol->private_value;
	int id = FUN_REG(mc->reg);
	ucontrol->value.integer.value[0] = vbc_eq_setting.is_active[id];
	return 0;
}

static int vbc_eq_switch_put(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
	    (struct soc_mixer_control *)kcontrol->private_value;
	int id = FUN_REG(mc->reg);
	int ret;
	pr_info("VBC %s eq switch %s\n", vbc_get_chan_name(id),
		ucontrol->value.integer.value[0] ? "ON" : "OFF");

	ret = ucontrol->value.integer.value[0];
	if (ret == vbc_eq_setting.is_active[id]) {
		return ret;
	}
	if ((ret == 0) || (ret == 1)) {
		vbc_eq_setting.is_active[id] = ret;
		if (vbc_eq_setting.is_active[id]) {
			vbc_eq_setting.vbc_eq_apply = vbc_eq_profile_apply;
			vbc_eq_try_apply(vbc_eq_setting.codec_dai, id);
		} else {
			vbc_eq_setting.vbc_eq_apply = 0;
			vbc_eq_profile_close(id);
		}
	}

	vbc_dbg("Leaving %s\n", __func__);
	return ret;
}

static int vbc_eq_load_get(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = vbc_eq_setting.is_loading;
	return 0;
}

static int vbc_eq_load_put(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int ret;
	struct soc_enum *texts = (struct soc_enum *)kcontrol->private_value;
	pr_info("VBC eq %s\n", texts->texts[ucontrol->value.integer.value[0]]);

	ret = ucontrol->value.integer.value[0];
	if (ret == 1) {
		ret = vbc_eq_loading(codec);
	}

	vbc_dbg("Leaving %s\n", __func__);
	return ret;
}

static int vbc_dg_get(struct snd_kcontrol *kcontrol,
		      struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
	    (struct soc_mixer_control *)kcontrol->private_value;
	int id = FUN_REG(mc->reg);
	int vbc_idx = mc->shift;

	ucontrol->value.integer.value[0] = vbc[vbc_idx].dg_val[id];
	return 0;
}

static int vbc_dg_put(struct snd_kcontrol *kcontrol,
		      struct snd_ctl_elem_value *ucontrol)
{
	int ret = 0;
	struct soc_mixer_control *mc =
	    (struct soc_mixer_control *)kcontrol->private_value;
	int id = FUN_REG(mc->reg);
	int vbc_idx = mc->shift;

	pr_info("VBC %s%s DG set 0x%02x\n",
		vbc_get_chan_name(vbc_idx),
		id == VBC_LEFT ? "L" : "R",
		(int)ucontrol->value.integer.value[0]);

	ret = ucontrol->value.integer.value[0];
	if (ret == vbc[vbc_idx].dg_val[id]) {
		return ret;
	}
	if (ret <= VBC_DG_VAL_MAX) {
		vbc[vbc_idx].dg_val[id] = ret;
	}

	vbc_try_dg_set(vbc_idx, id);

	vbc_dbg("Leaving %s\n", __func__);
	return ret;
}

static int vbc_st_dg_get(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
	    (struct soc_mixer_control *)kcontrol->private_value;
	int id = FUN_REG(mc->reg);

	ucontrol->value.integer.value[0] = st_dg.dg_val[id];
	return 0;
}

static int vbc_st_dg_put(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	int ret = 0;
	struct soc_mixer_control *mc =
	    (struct soc_mixer_control *)kcontrol->private_value;
	int id = FUN_REG(mc->reg);

	pr_info("VBC ST%s DG set 0x%02x\n",
		id == VBC_LEFT ? "L" : "R",
		(int)ucontrol->value.integer.value[0]);

	ret = ucontrol->value.integer.value[0];
	if (ret == st_dg.dg_val[id]) {
		return ret;
	}
	if (ret <= VBC_DG_VAL_MAX) {
		st_dg.dg_val[id] = ret;
	}

	vbc_try_st_dg_set(id);

	vbc_dbg("Leaving %s\n", __func__);
	return ret;
}

static int vbc_dg_switch_get(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
	    (struct soc_mixer_control *)kcontrol->private_value;
	int id = FUN_REG(mc->reg);
	int vbc_idx = mc->shift;

	ucontrol->value.integer.value[0] = vbc[vbc_idx].dg_switch[id];
	return 0;
}

static int vbc_dg_switch_put(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	int ret = 0;
	struct soc_mixer_control *mc =
	    (struct soc_mixer_control *)kcontrol->private_value;
	int id = FUN_REG(mc->reg);
	int vbc_idx = mc->shift;

	pr_info("VBC %s%s DG switch %s\n",
		vbc_get_chan_name(vbc_idx),
		id == VBC_LEFT ? "L" : "R",
		ucontrol->value.integer.value[0] ? "ON" : "OFF");

	ret = ucontrol->value.integer.value[0];
	if (ret == vbc[vbc_idx].dg_switch[id]) {
		return ret;
	}

	vbc[vbc_idx].dg_switch[id] = ret;

	vbc_try_dg_set(vbc_idx, id);

	vbc_dbg("Leaving %s\n", __func__);
	return ret;
}

static int vbc_st_dg_switch_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
	    (struct soc_mixer_control *)kcontrol->private_value;
	int id = FUN_REG(mc->reg);

	ucontrol->value.integer.value[0] = st_dg.dg_switch[id];
	return 0;
}

static int vbc_st_dg_switch_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	int ret = 0;
	struct soc_mixer_control *mc =
	    (struct soc_mixer_control *)kcontrol->private_value;
	int id = FUN_REG(mc->reg);

	pr_info("VBC ST%s DG switch %s\n",
		id == VBC_LEFT ? "L" : "R",
		ucontrol->value.integer.value[0] ? "ON" : "OFF");

	ret = ucontrol->value.integer.value[0];
	if (ret == st_dg.dg_switch[id]) {
		return ret;
	}

	st_dg.dg_switch[id] = ret;

	vbc_try_st_dg_set(id);

	vbc_dbg("Leaving %s\n", __func__);
	return ret;
}

static int adc_dgmux_get(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
	    (struct soc_mixer_control *)kcontrol->private_value;
	int id = FUN_REG(mc->reg);

	ucontrol->value.integer.value[0] = adc_dgmux_val[id];
	return 0;
}

static int adc_dgmux_put(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	int ret = 0;
	struct soc_mixer_control *mc =
	    (struct soc_mixer_control *)kcontrol->private_value;
	int id = FUN_REG(mc->reg);

	pr_info("VBC AD%d DG mux : %ld\n", id,
		ucontrol->value.integer.value[0]);

	ret = ucontrol->value.integer.value[0];
	if (ret == adc_dgmux_val[id]) {
		return ret;
	}

	adc_dgmux_val[id] = ret;
	vbc_try_ad_dgmux_set(id);

	vbc_dbg("Leaving %s\n", __func__);
	return ret;
}

static int dac_iismux_get(struct snd_kcontrol *kcontrol,
			  struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = vbc_da_iis_port;
	return 0;
}

static int dac_iismux_put(struct snd_kcontrol *kcontrol,
			  struct snd_ctl_elem_value *ucontrol)
{
	struct soc_enum *texts = (struct soc_enum *)kcontrol->private_value;
	pr_info("VBC DA output to %s\n",
		texts->texts[ucontrol->value.integer.value[0]]);
	if (vbc_da_iis_port == ucontrol->value.integer.value[0])
		return 0;

	vbc_da_iis_port = ucontrol->value.integer.value[0];

	vbc_try_da_iismux_set();

	vbc_dbg("Leaving %s\n", __func__);
	return 1;
}

static int fm_sample_rate_get(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = ((fm_sample_rate == 32000) ? 0 : 1);
	return 0;
}

static int fm_sample_rate_set(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct soc_enum *texts = (struct soc_enum *)kcontrol->private_value;
	pr_info("fm_sample_rate is %s\n",
		texts->texts[ucontrol->value.integer.value[0]]);
	fm_sample_rate = (ucontrol->value.integer.value[0] ? 48000 : 32000);

	vbc_dbg("Leaving %s\n", __func__);
	return 1;
}

static const char *switch_function[] =
    { "cp0-dsp", "cp1-dsp", "ap", "cp2-arm" };
static const char *eq_load_function[] = { "idle", "loading" };
static const char *da_iis_mux_function[] =
    { "sprd-codec", "ext-codec-4", "ext-codec-6" };
static const char *fm_sample_rate_function[] = { "32000", "48000" };

static const struct soc_enum vbc_enum[] = {
	SOC_ENUM_SINGLE_EXT(4, switch_function),
	SOC_ENUM_SINGLE_EXT(2, eq_load_function),
	SOC_ENUM_SINGLE_EXT(3, da_iis_mux_function),
	SOC_ENUM_SINGLE_EXT(2, fm_sample_rate_function),
};

static const struct snd_kcontrol_new vbc_controls[] = {
	SOC_ENUM_EXT("VBC Switch", vbc_enum[0], vbc_switch_get,
		     vbc_switch_put),
	SOC_SINGLE_EXT("VBC DA EQ Switch", FUN_REG(VBC_CHAN_DA), 0, 1, 0,
		       vbc_eq_switch_get,
		       vbc_eq_switch_put),
	SOC_SINGLE_EXT("VBC AD01 EQ Switch", FUN_REG(VBC_CHAN_AD01), 0, 1, 0,
		       vbc_eq_switch_get,
		       vbc_eq_switch_put),
	SOC_SINGLE_EXT("VBC AD02 EQ Switch", FUN_REG(VBC_CHAN_AD23), 0, 1, 0,
		       vbc_eq_switch_get,
		       vbc_eq_switch_put),
	SOC_ENUM_EXT("VBC EQ Update", vbc_enum[1], vbc_eq_load_get,
		     vbc_eq_load_put),

	SOC_SINGLE_EXT("VBC DACL DG Set", FUN_REG(VBC_LEFT),
		       VBC_CHAN_DA, VBC_DG_VAL_MAX, 0, vbc_dg_get,
		       vbc_dg_put),
	SOC_SINGLE_EXT("VBC DACR DG Set", FUN_REG(VBC_RIGHT),
		       VBC_CHAN_DA, VBC_DG_VAL_MAX, 0, vbc_dg_get,
		       vbc_dg_put),
	SOC_SINGLE_EXT("VBC ADCL DG Set", FUN_REG(VBC_LEFT),
		       VBC_CHAN_AD01, VBC_DG_VAL_MAX, 0, vbc_dg_get,
		       vbc_dg_put),
	SOC_SINGLE_EXT("VBC ADCR DG Set", FUN_REG(VBC_RIGHT),
		       VBC_CHAN_AD01, VBC_DG_VAL_MAX, 0, vbc_dg_get,
		       vbc_dg_put),
	SOC_SINGLE_EXT("VBC ADC23L DG Set", FUN_REG(VBC_LEFT),
		       VBC_CHAN_AD23, VBC_DG_VAL_MAX, 0, vbc_dg_get,
		       vbc_dg_put),
	SOC_SINGLE_EXT("VBC ADC23R DG Set", FUN_REG(VBC_RIGHT),
		       VBC_CHAN_AD23, VBC_DG_VAL_MAX, 0, vbc_dg_get,
		       vbc_dg_put),
	SOC_SINGLE_EXT("VBC STL DG Set", FUN_REG(VBC_LEFT),
		       0, VBC_DG_VAL_MAX, 0, vbc_st_dg_get, vbc_st_dg_put),
	SOC_SINGLE_EXT("VBC STR DG Set", FUN_REG(VBC_RIGHT),
		       0, VBC_DG_VAL_MAX, 0, vbc_st_dg_get, vbc_st_dg_put),

	SOC_SINGLE_EXT("VBC DACL DG Switch", FUN_REG(VBC_LEFT),
		       VBC_CHAN_DA, 1, 0, vbc_dg_switch_get,
		       vbc_dg_switch_put),
	SOC_SINGLE_EXT("VBC DACR DG Switch", FUN_REG(VBC_RIGHT),
		       VBC_CHAN_DA, 1, 0, vbc_dg_switch_get,
		       vbc_dg_switch_put),
	SOC_SINGLE_EXT("VBC ADCL DG Switch", FUN_REG(VBC_LEFT),
		       VBC_CHAN_AD01, 1, 0, vbc_dg_switch_get,
		       vbc_dg_switch_put),
	SOC_SINGLE_EXT("VBC ADCR DG Switch", FUN_REG(VBC_RIGHT),
		       VBC_CHAN_AD01, 1, 0, vbc_dg_switch_get,
		       vbc_dg_switch_put),
	SOC_SINGLE_EXT("VBC ADC23L DG Switch", FUN_REG(VBC_LEFT),
		       VBC_CHAN_AD23, 1, 0, vbc_dg_switch_get,
		       vbc_dg_switch_put),
	SOC_SINGLE_EXT("VBC ADC23R DG Switch", FUN_REG(VBC_RIGHT),
		       VBC_CHAN_AD23, 1, 0, vbc_dg_switch_get,
		       vbc_dg_switch_put),
	SOC_SINGLE_EXT("VBC STL DG Switch", FUN_REG(VBC_LEFT),
		       0, 1, 0, vbc_st_dg_switch_get, vbc_st_dg_switch_put),
	SOC_SINGLE_EXT("VBC STR DG Switch", FUN_REG(VBC_RIGHT),
		       0, 1, 0, vbc_st_dg_switch_get, vbc_st_dg_switch_put),

	SOC_SINGLE_EXT("VBC AD0 DG Mux", FUN_REG(ADC0_DGMUX), 0, 1, 0,
		       adc_dgmux_get, adc_dgmux_put),
	SOC_SINGLE_EXT("VBC AD1 DG Mux", FUN_REG(ADC1_DGMUX), 0, 1, 0,
		       adc_dgmux_get, adc_dgmux_put),
	SOC_SINGLE_EXT("VBC AD2 DG Mux", FUN_REG(ADC2_DGMUX), 0, 1, 0,
		       adc_dgmux_get, adc_dgmux_put),
	SOC_SINGLE_EXT("VBC AD3 DG Mux", FUN_REG(ADC3_DGMUX), 0, 1, 0,
		       adc_dgmux_get, adc_dgmux_put),
	SOC_ENUM_EXT("VBC DA IIS Mux", vbc_enum[2], dac_iismux_get,
		     dac_iismux_put),
	SOC_ENUM_EXT("FM Sample Rate", vbc_enum[3], fm_sample_rate_get,
		     fm_sample_rate_set),

	SOC_SINGLE_EXT("VBC DA EQ Profile Select", FUN_REG(VBC_CHAN_DA), 0,
		       VBC_EQ_PROFILE_CNT_MAX, 0,
		       vbc_eq_profile_get, vbc_eq_profile_put),
	SOC_SINGLE_EXT("VBC AD01 EQ Profile Select", FUN_REG(VBC_CHAN_AD01), 0,
		       VBC_EQ_PROFILE_CNT_MAX, 0,
		       vbc_eq_profile_get, vbc_eq_profile_put),
	SOC_SINGLE_EXT("VBC AD23 EQ Profile Select", FUN_REG(VBC_CHAN_AD23), 0,
		       VBC_EQ_PROFILE_CNT_MAX, 0,
		       vbc_eq_profile_get, vbc_eq_profile_put),

};

int vbc_add_controls(struct snd_soc_codec *codec)
{
	int ret;
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	/*add routes and widgets into codec */
	ret = snd_soc_dapm_new_controls(dapm, vbc_dapm_widgets,
					ARRAY_SIZE(vbc_dapm_widgets));
	if (ret < 0) {
		pr_err("Failed to VBC add widgets\n");
	}
	ret = snd_soc_dapm_add_routes(dapm, vbc_intercon,
				      ARRAY_SIZE(vbc_intercon));
	if (ret < 0) {
		pr_err("Failed to VBC add routes\n");
	}
	ret = snd_soc_add_codec_controls(codec, vbc_controls,
					 ARRAY_SIZE(vbc_controls));
	if (ret < 0) {
		pr_err("Failed to VBC add controls\n");
	}

	vbc_proc_init(codec);

	return ret;
}

EXPORT_SYMBOL_GPL(vbc_add_controls);

static int __init vbc_init(void)
{
	int ret;
	arch_audio_vbc_switch(AUDIO_TO_AP_ARM_CTRL);
	ret = arch_audio_vbc_switch(AUDIO_NO_CHANGE);
	if (ret != AUDIO_TO_AP_ARM_CTRL) {
		pr_err("Failed to Switch VBC to AP\n");
		return -1;
	}
	vbc_control = 2;
	 /*AP*/ return platform_driver_register(&vbc_driver);
}

static void __exit vbc_exit(void)
{
	platform_driver_unregister(&vbc_driver);
}

module_init(vbc_init);
module_exit(vbc_exit);

MODULE_DESCRIPTION("SPRD ASoC VBC CUP-DAI driver");
MODULE_AUTHOR("Zhenfang Wang <zhenfang.wang@spreadtrum.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("cpu-dai:vbc");
