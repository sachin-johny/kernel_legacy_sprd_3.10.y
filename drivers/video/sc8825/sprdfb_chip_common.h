/******************************************************************************
 ** File Name:    sprdfb_chip_common.h                                     *
 ** Author:       congfu.zhao                                           *
 ** DATE:         30/04/2013                                        *
 ** Copyright:    2013 Spreatrum, Incoporated. All Rights Reserved. *
 ** Description:                                                    *
 ******************************************************************************/
/******************************************************************************
 **                   Edit    History                               *
 **---------------------------------------------------------------------------*
 ** DATE          NAME            DESCRIPTION                       *

 ******************************************************************************/
#ifndef __DISPC_CHIP_COM_H_
#define __DISPC_CHIP_COM_H_

#if defined(CONFIG_FB_SCX35)
#include "sprdfb_chip_8830.h"
#elif defined(CONFIG_FB_SC8825)
#include "sprdfb_chip_8825.h"
#elif defined(CONFIG_FB_SC7710)
#include "sprdfb_chip_7710.h"
#else
#error "Unknown architecture specification"
#endif

#include <mach/hardware.h>
#include <mach/globalregs.h>
#include <linux/io.h>
#include <mach/irqs.h>
#include <mach/sci.h>


void __raw_bits_set_value(unsigned int reg, unsigned int value, unsigned int bit, unsigned int mask);

void dispc_pll_clk_set(unsigned int clk_src, unsigned int clk_div);

void dispc_dbi_clk_set(unsigned int clk_src, unsigned int clk_div);

void dispc_dpi_clk_set(unsigned int clk_src, unsigned int clk_div);

u32 dispc_glb_read(u32 reg);


#endif
