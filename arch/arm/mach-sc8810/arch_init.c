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

#include <linux/module.h>

#include <asm/hardware/cache-l2x0.h>
#include <asm/io.h>

#include <mach/hardware.h>
#include <mach/adi.h>


#define CACHE_EARLY_BRESP_ENABLE	((0) << 30)
#define CACHE_I_P_ENABLE		((0) << 29)
#define CACHE_D_P_ENABLE		((1) << 28)
#define CACHE_N_S_ACCESS		((1) << 27)
#define CACHE_N_L_ENABLE		((1) << 26)
#define CACHE_REPLACE_POLICY 		((1) << 25)
#define CACHE_FORCE_W_A 		(((0) & 0x3) << 23)

#define CACHE_S_O_ENABLE		((1) << 22)
#define CACHE_PARITY_ENABLE		((0) << 21)
#define CACHE_EVENT_ENABLE		((1) << 20)
#define CACHE_WAY_SIZE			(((2) & 0x3) << 17)
#define CACHE_ASSOCI 			((0) << 16)
#define CACHE_SHARED_INV_ENABLE		((0) << 13)

#define CACHE_EC_CONFIG			((0) << 12)
#define CACHE_STORE_BUFFER_LIMI_ENABLE	((1) << 11)
#define CACHE_HIGH_PRIORITY_SO_DEV	((0) << 10)
#define CACHE_FULL_LINE_ZERO_ENABLE	((0) << 0)

#define AUX_VALUE (CACHE_EARLY_BRESP_ENABLE | CACHE_I_P_ENABLE | CACHE_D_P_ENABLE |CACHE_N_S_ACCESS \
		| CACHE_N_L_ENABLE | CACHE_REPLACE_POLICY | CACHE_FORCE_W_A | CACHE_S_O_ENABLE | \
		CACHE_PARITY_ENABLE | CACHE_EVENT_ENABLE | CACHE_WAY_SIZE | \
		CACHE_ASSOCI | CACHE_SHARED_INV_ENABLE | CACHE_EC_CONFIG | \
		CACHE_STORE_BUFFER_LIMI_ENABLE | CACHE_HIGH_PRIORITY_SO_DEV | CACHE_FULL_LINE_ZERO_ENABLE)

#define PL310_CACHE_AUX_VALUE	AUX_VALUE
#define PL310_CACHE_AUX_MASK	0xffffffff

extern void arch_init_neon(void);

static int __init arch_init(void)
{
	arch_init_neon();

#ifdef CONFIG_CACHE_L2X0
	l2x0_init((void __iomem *)SPRD_CACHE310_BASE,
			PL310_CACHE_AUX_VALUE, PL310_CACHE_AUX_MASK);
#endif

	return 0;
}

early_initcall(arch_init);

#ifdef CONFIG_ARCH_SC7710

#include <mach/globalregs.h>
#include <mach/regs_ana_glb_sc7710.h>

static int glb_ana_chipid = 0;

/*some chip init global helper functions */
#define	AUDIO_ANA_CTRL				(SPRD_MISC_BASE + 0x0888)
#define CODEC_AP_AUDIF_BASE			(SPRD_MISC_PHYS + 0x0700)
#define HEADSET_INSERT_MIC_BIT				(BIT(6))
#define AUDIO_ARM_EN						(BIT(0))
#define AUDIO_SWITCH_ARM					(BIT(15))

void __init sc8810_init_ana_chipid(void)
{
	int reg_val;

	glb_ana_chipid = ((sci_adi_read(ANA_REG_GLB_CHIP_ID_HIGH) << 16) | sci_adi_read(ANA_REG_GLB_CHIP_ID_LOW));

	if (glb_ana_chipid == ANA_CHIP_ID_BA) {

		reg_val = sci_adi_read(AUDIO_ANA_CTRL);

		sci_adi_set(AUDIO_ANA_CTRL, reg_val | AUDIO_ARM_EN | AUDIO_SWITCH_ARM);

		if (!(sci_adi_read(CODEC_AP_AUDIF_BASE + 0xC0) & HEADSET_INSERT_MIC_BIT)) {
			glb_ana_chipid = ANA_CHIP_ID_BB;
		}

		sci_adi_set(AUDIO_ANA_CTRL, reg_val);
	}
}

int sci_get_chipid(void)
{
	return sprd_greg_read(REG_TYPE_AHB_GLOBAL, CHIP_ID);
}
int sci_get_ana_chipid(void)
{
	return glb_ana_chipid;
}
#endif
