/*
 * Copyright (C) 2013 Spreadtrum Communications Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *************************************************
 * Automatically generated C header: do not edit *
 *************************************************
 */
#ifndef __ASM_ARCH_SCI_CLOCK_TREE_H
#define __ASM_ARCH_SCI_CLOCK_TREE_H

#if defined(CONFIG_ARCH_SC8830)
#include "mach/regs_sc8830_aon_apb.h"
#include "mach/regs_sc8830_aon_ckg.h"
#include "mach/regs_sc8830_aon_clk.h"
#include "mach/regs_sc8830_ap_ahb.h"
#include "mach/regs_sc8830_ap_apb.h" 
#include "mach/regs_sc8830_ap_clk.h"
//#include "mach/regs_sc8830_gpu_apb.h"
#include "mach/regs_sc8830_gpu_clk.h"
#include "mach/regs_sc8830_mm_ahb.h"
#include "mach/regs_sc8830_mm_clk.h"
#include "mach/regs_sc8830_pmu_apb.h"
#include "mach/regs_sc8830_pub_apb.h"
#include "__sc8830_clock_tree.h"
#elif defined(CONFIG_ARCH_SC8825)
#include "__sc8825_clock_tree.h"
#else
#error "Unknown architecture specification"
#endif

#endif