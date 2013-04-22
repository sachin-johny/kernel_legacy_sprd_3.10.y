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

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <mach/hardware.h>
#include <mach/adi.h>
#include <linux/io.h>
#include <mach/regs_ana_glb_sc7710.h>

#define DCDC_CORE_V_09 (0x3)
#define DCDC_CORE_V_MASK (0x1f)

#define DCDC_LP_MASK (0xc0ff)

void enable_slp_db(int bits, int enable)
{
	if(enable)
		sci_adi_set(ANA_REG_GLB_CHIP_SLP_DB, bits);
	else
		sci_adi_clr(ANA_REG_GLB_CHIP_SLP_DB, bits);
}

void print_init_ana(void)
{
	printk("ANA_REG_GLB_DCDC_SLP_V %08x  = %08x\n", ANA_REG_GLB_DCDC_SLP_V, sci_adi_read(ANA_REG_GLB_DCDC_SLP_V));
	printk("ANA_REG_GLB_DCDC_LP_EN = %08x\n", sci_adi_read(ANA_REG_GLB_DCDC_LP_EN));

	printk("ANA_REG_GLB_LDO_AP_SLP_CTRL0 = %08x\n", sci_adi_read(ANA_REG_GLB_LDO_AP_SLP_CTRL0));
	printk("ANA_REG_GLB_LDO_AP_SLP_CTRL1 = %08x\n", sci_adi_read(ANA_REG_GLB_LDO_AP_SLP_CTRL1));

    printk("ANA_REG_GLB_LDO_CP_SLP_CTRL0 = %08x\n", sci_adi_read(ANA_REG_GLB_LDO_CP_SLP_CTRL0));
    printk("ANA_REG_GLB_LDO_CP_SLP_CTRL1 = %08x\n", sci_adi_read(ANA_REG_GLB_LDO_CP_SLP_CTRL1));

    printk("ANA_REG_GLB_LDO_DEEP_SLP_CTRL0 = %08x\n", sci_adi_read(ANA_REG_GLB_LDO_DEEP_SLP_CTRL0));
    printk("ANA_REG_GLB_LDO_DEEP_SLP_CTRL1 = %08x\n", sci_adi_read(ANA_REG_GLB_LDO_DEEP_SLP_CTRL1));

    printk("ANA_REG_GLB_LDO_XTL_SLP_CTRL0 = %08x\n", sci_adi_read(ANA_REG_GLB_LDO_XTL_SLP_CTRL0));
    printk("ANA_REG_GLB_LDO_XTL_SLP_CTRL1 = %08x\n", sci_adi_read(ANA_REG_GLB_LDO_XTL_SLP_CTRL1));
}

/*init ana global regs for pm*/
void init_ana_gr(void)
{
        /*set dcdc_core scale to 0.9v at chip sleep*/
	sci_adi_write(ANA_REG_GLB_DCDC_SLP_V, 0
			| BIT_SLP_DCDC_CORE_V_EN
			| BIT_XTL_SLP_DCDC_CORE_V_EN
			| BITS_SLP_DCDC_CORE_V(DCDC_CORE_V_09)
			, DCDC_CORE_V_MASK);

#if 0
        /* set dcdc mem power down */
        sci_adi_clr(ANA_REG_GLB_LDO_PD_RST, BIT_DCDC_MEM_PD_RST);
        sci_adi_set(ANA_REG_GLB_LDO_PD_SET, BIT_DCDC_MEM_PD);
#endif

#if 0
	/* set dcdc lowpower mode */
        sci_adi_write(ANA_REG_GLB_DCDC_LP_EN, 0
                        |BIT_XTL_SLP_DCDC_OTP_PD_EN
                        |BIT_XTL_SLP_WPA_DCDC_LP_EN
                        /*|BIT_XTL_SLP_DCDC_MEM_LP_EN*/
                        /*|BIT_XTL_SLP_DCDC_ARM_LP_EN*/
                        /*|BIT_XTL_SLP_DCDC_CORE_LP_EN*/
                        ,DCDC_LP_MASK);
#endif

	/* setup ldo AP sleep mode */
	sci_adi_raw_write(ANA_REG_GLB_LDO_AP_SLP_CTRL0, 0
		//|BIT_SLP_AP_LDOAMP_PD_EN
		//|BIT_SLP_AP_LDOVDD25_PD_EN
		//|BIT_SLP_AP_LDOVDD18_PD_EN
		//|BIT_SLP_AP_LDOVDD28_PD_EN
		//|BIT_SLP_AP_LDOAVDDBB_PD_EN
		//|BIT_SLP_AP_LDOSDIO_PD_EN
		//|BIT_SLP_AP_LDOSIM2_PD_EN
		|BIT_SLP_AP_LDOCAMA_PD_EN
		|BIT_SLP_AP_LDOCAMD1_PD_EN
		|BIT_SLP_AP_LDOCAMD0_PD_EN
		//|BIT_SLP_AP_LDOUSB_PD_EN
		//|BIT_SLP_AP_LDOSIM1_PD_EN
		//|BIT_SLP_AP_LDOSIM0_PD_EN
		//|BIT_SLP_AP_LDORF1_PD_EN
		//|BIT_SLP_AP_LDORF0_PD_EN
		);

	sci_adi_raw_write(ANA_REG_GLB_LDO_AP_SLP_CTRL1, 0
		//|BIT_SLP_AP_LDOMEM_PD_EN
		|BIT_SLP_AP_LDOEMMIO_PD_EN
		|BIT_SLP_AP_LDOEMMCORE_PD_EN
		|BIT_SLP_AP_LDOWIF1_PD_EN
		//BIT_SLP_AP_LDOWIF0_PD_EN
		//|BIT_SLP_AP_WPADCDC_PD_EN
		//|BIT_SLP_AP_DCDCMEM_PD_EN
		|BIT_SLP_AP_DCDCARM_PD_EN
		//|BIT_SLP_AP_DCDCCORE_PD_EN
		);

	/* setup ldo CP sleep mode */
	sci_adi_raw_write(ANA_REG_GLB_LDO_CP_SLP_CTRL0, 0
		//|BIT_SLP_CP_LDOAMP_PD_EN
		//|BIT_SLP_CP_LDOVDD25_PD_EN
		//|BIT_SLP_CP_LDOVDD18_PD_EN
		//|BIT_SLP_CP_LDOVDD28_PD_EN
		//|BIT_SLP_CP_LDOAVDDBB_PD_EN
		//|BIT_SLP_CP_LDOSDIO_PD_EN
		//|BIT_SLP_CP_LDOSIM2_PD_EN
		//|BIT_SLP_CP_LDOCAMA_PD_EN
		//|BIT_SLP_CP_LDOCAMD1_PD_EN
		//|BIT_SLP_CP_LDOCAMD0_PD_EN
		//|BIT_SLP_CP_LDOUSB_PD_EN
		//|BIT_SLP_CP_LDOSIM1_PD_EN
		//|BIT_SLP_CP_LDOSIM0_PD_EN
		//|BIT_SLP_CP_LDORF1_PD_EN
		//|BIT_SLP_CP_LDORF0_PD_EN
		);

	sci_adi_raw_write(ANA_REG_GLB_LDO_CP_SLP_CTRL1, 0
		//|BIT_SLP_CP_LDOMEM_PD_EN
		//|BIT_SLP_CP_LDOEMMIO_PD_EN
		//|BIT_SLP_CP_LDOEMMCORE_PD_EN
		//|BIT_SLP_CP_LDOWIF1_PD_EN
		//|BIT_SLP_CP_LDOWIF0_PD_EN
		|BIT_SLP_CP_WPADCDC_PD_EN
		//|BIT_SLP_CP_DCDCMEM_PD_EN
		//|BIT_SLP_CP_DCDCARM_PD_EN
		//|BIT_SLP_CP_DCDCCORE_PD_EN
		);

	/* setup ldo DEEP sleep mode */
	sci_adi_raw_write(ANA_REG_GLB_LDO_DEEP_SLP_CTRL0, 0
		|BIT_DEEP_SLP_LDOAMP_PD_EN
		|BIT_DEEP_SLP_LDOVDD25_PD_EN
		//|BIT_DEEP_SLP_LDOVDD18_PD_EN
		//|BIT_DEEP_SLP_LDOVDD28_PD_EN
		|BIT_DEEP_SLP_LDOAVDDBB_PD_EN
		//|BIT_DEEP_SLP_LDOSDIO_PD_EN
		//|BIT_DEEP_SLP_LDOSIM2_PD_EN
		|BIT_DEEP_SLP_LDOCAMA_PD_EN
		|BIT_DEEP_SLP_LDOCAMD1_PD_EN
		|BIT_DEEP_SLP_LDOCAMD0_PD_EN
		|BIT_DEEP_SLP_LDOUSB_PD_EN
		//|BIT_DEEP_SLP_LDOSIM1_PD_EN
		//|BIT_DEEP_SLP_LDOSIM0_PD_EN
		//|BIT_DEEP_SLP_LDORF1_PD_EN
		//|BIT_DEEP_SLP_LDORF0_PD_EN
		);

	sci_adi_raw_write(ANA_REG_GLB_LDO_DEEP_SLP_CTRL1, 0
		//|BIT_DEEP_SLP_LDOMEM_PD_EN
		//BIT_DEEP_SLP_LDOEMMIO_PD_EN
		//BIT_DEEP_SLP_LDOEMMCORE_PD_EN
		//BIT_DEEP_SLP_LDOWIF1_PD_EN
		//BIT_DEEP_SLP_LDOWIF0_PD_EN
		|BIT_DEEP_SLP_WPADCDC_PD_EN
		//|BIT_DEEP_SLP_DCDCMEM_PD_EN
		|BIT_DEEP_SLP_DCDCARM_PD_EN
		//|BIT_DEEP_SLP_DCDCCORE_PD_EN
		);

#if 1
	/* setup ldo XTL sleep mode */
	sci_adi_raw_write(ANA_REG_GLB_LDO_XTL_SLP_CTRL0, 0
		//|BIT_SLP_XTL_LDOAMP_PD_EN
		//|BIT_SLP_XTL_LDOVDD25_PD_EN
		//|BIT_SLP_XTL_LDOVDD18_PD_EN
		//|BIT_SLP_XTL_LDOVDD28_PD_EN
		//|BIT_SLP_XTL_LDOAVDDBB_PD_EN
		//|BIT_SLP_XTL_LDOSDIO_PD_EN
		//|BIT_SLP_XTL_LDOSIM2_PD_EN
		//|BIT_SLP_XTL_LDOCAMA_PD_EN
		//|BIT_SLP_XTL_LDOCAMD1_PD_EN
		//|BIT_SLP_XTL_LDOCAMD0_PD_EN
		//|BIT_SLP_XTL_LDOUSB_PD_EN
		//|BIT_SLP_XTL_LDOSIM1_PD_EN
		//|BIT_SLP_XTL_LDOSIM0_PD_EN
		|BIT_SLP_XTL_LDORF1_PD_EN
		|BIT_SLP_XTL_LDORF0_PD_EN
		);

	sci_adi_raw_write(ANA_REG_GLB_LDO_XTL_SLP_CTRL1, 0
		//|BIT_SLP_XTL_LDOMEM_PD_EN
		//|BIT_SLP_XTL_LDOEMMIO_PD_EN
		//|BIT_SLP_XTL_LDOEMMCORE_PD_EN
		//|BIT_SLP_XTL_LDOWIF1_PD_EN
		//|BIT_SLP_XTL_LDOWIF0_PD_EN
		//|BIT_SLP_XTL_WPADCDC_PD_EN
		//|BIT_SLP_XTL_DCDCMEM_PD_EN
		//|BIT_SLP_XTL_DCDCARM_PD_EN
		//|BIT_SLP_XTL_DCDCCORE_PD_EN
		);
#endif

	/* setup ldo deep sleep lowpower mode */
	sci_adi_raw_write(ANA_REG_GLB_SLP_LDO_LP_CTRL0, 0
		//|BIT_SLP_LDOAMP_LP_EN
		//|BIT_SLP_LDOVDD25_LP_EN
		//|BIT_SLP_LDOVDD18_LP_EN
		//|BIT_SLP_LDOVDD28_LP_EN
		//|BIT_SLP_LDOAVDDBB_LP_EN
		//|BIT_SLP_LDOSDIO_LP_EN
		//|BIT_SLP_LDOSIM2_LP_EN
		//|BIT_SLP_LDOCAMA_LP_EN
		//|BIT_SLP_LDOCAMD1_LP_EN
		//|BIT_SLP_LDOCAMD0_LP_EN
		//|BIT_SLP_LDOUSB_LP_EN
		//|BIT_SLP_LDOSIM1_LP_EN
		//|BIT_SLP_LDOSIM0_LP_EN
		//|BIT_SLP_LDORF1_LP_EN
		//|BIT_SLP_LDORF0_LP_EN
		);

	/* setup ldo xtl sleep lowpower mode */
	sci_adi_raw_write(ANA_REG_GLB_SLP_LDO_LP_CTRL1, 0
		//|BIT_SLP_MBG_LP_EN
		//|BIT_SLP_LDOMEM_LP_EN
		//|BIT_SLP_LDOEMMIO_LP_EN
		//|BIT_SLP_LDOEMMCORE_LP_EN
		//|BIT_SLP_LDOWIF1_LP_EN
		//|BIT_SLP_LDOWIF0_LP_EN
		);

	sci_adi_raw_write(ANA_REG_GLB_SLP_XTL_LDO_LP_CTRL0, 0
		//|BIT_SLP_XTL_LDOAMP_LP_EN
		//|BIT_SLP_XTL_LDOVDD25_LP_EN
		//|BIT_SLP_XTL_LDOVDD18_LP_EN
		//|BIT_SLP_XTL_LDOVDD28_LP_EN
		//|BIT_SLP_XTL_LDOAVDDBB_LP_EN
		//|BIT_SLP_XTL_LDOSDIO_LP_EN
		//|BIT_SLP_XTL_LDOSIM2_LP_EN
		//|BIT_SLP_XTL_LDOCAMA_LP_EN
		//|BIT_SLP_XTL_LDOCAMD1_LP_EN
		//|BIT_SLP_XTL_LDOCAMD0_LP_EN
		//|BIT_SLP_XTL_LDOUSB_LP_EN
		//|BIT_SLP_XTL_LDOSIM1_LP_EN
		//|BIT_SLP_XTL_LDOSIM0_LP_EN
		//|BIT_SLP_XTL_LDORF1_LP_EN
		//|BIT_SLP_XTL_LDORF0_LP_EN
		);

	sci_adi_raw_write(ANA_REG_GLB_SLP_XTL_LDO_LP_CTRL1, 0
		//|BIT_SLP_XTL_MBG_LP_EN
		//|BIT_SLP_XTL_LDOMEM_LP_EN
		//|BIT_SLP_XTL_LDOEMMIO_LP_EN
		//|BIT_SLP_XTL_LDOEMMCORE_LP_EN
		//|BIT_SLP_XTL_LDOWIF1_LP_EN
		//|BIT_SLP_XTL_LDOWIF0_LP_EN
		);

	/* enable ldo sleep mode*/
	sci_adi_set(ANA_REG_GLB_LDO_SLP_CTRL, 0
		|BIT_SLP_PD_EN
		);
	print_init_ana();
}

void sc7710_turnoff_allldo(void)
{
	/*select AP control all ldo*/
	sci_adi_raw_write(ANA_REG_GLB_LDO_SW, 0);

	/*turn off all modules ldo*/
	sci_adi_raw_write(ANA_REG_GLB_LDO_PD_CTRL1, 0x5555);
	sci_adi_raw_write(ANA_REG_GLB_LDO_PD_CTRL0, 0x5555);

	/*turn off all system cores ldo*/
	sci_adi_raw_write(ANA_REG_GLB_LDO_PD_RST, 0);
	sci_adi_raw_write(ANA_REG_GLB_LDO_PD_SET, 0xffff);
}
