/*
 ****************************************************************
 *
 * Copyright (C) 2002-2007, VirtualLogix.
 *
 * This program is free software;  you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * #ident  "@(#)nk_f.h 1.5     08/07/04 VirtualLogix"
 *
 * Contributor(s):
 *   Vladimir Grouzdev <vladimir.grouzdev@virtuallogix.com>
 *   Guennadi Maslov   <guennadi.maslov@virtuallogix.com>
 *
 ****************************************************************
 */

#ifndef _NK_NK_F_H
#define _NK_NK_F_H

    /*
     * Nano-kernel OS IDs.
     */
#define NK_OS_NKERN	   0	/* index #0 is for nano-kernel itself */
#define NK_OS_PRIM	   1	/* index #1 is for primary OS */
#define NK_OS_LIMIT	  32	/* theoretical limit of supported OSes */

#define NK_OS_ANON	 (-1)	/* if NkOsId is used as owner tag, */
				/* this special value says: */
				/* owner is anonymous, or no owner */

    /*
     * Limit of the XIRQ number used to virtualize HW IRQs
     */
#define NK_HW_XIRQ_LIMIT 512

#if !defined(__ASM__) && !defined(__ASSEMBLY__)

typedef  unsigned char	     nku8_f;
typedef  unsigned short	     nku16_f;
typedef  unsigned int	     nku32_f;
typedef  unsigned long long  nku64_f;

typedef  nku32_f  NkPhAddr;	/* physical address */
typedef  nku32_f  NkPhSize;	/* physical size */

typedef  nku32_f  NkOsId;	/* OS identifier */
typedef  nku32_f  NkOsMask;	/* OS identifiers bit-mask */

typedef  nku32_f  NkXIrq;	/* cross interrupt request */
typedef  nku32_f  NkVex;	/* virtual exception number */

typedef  nku32_f  NkCpuId;	/* CPU identifier */
typedef  nku32_f  NkCpuMask;	/* CPU identifier bit-mask */

typedef struct NkSchedParams {  /* scheduling parameters */
    unsigned int fg_prio;	/* foreground priority of secondary OS */
    unsigned int bg_prio;       /* background priority of secondary OS */
    unsigned int quantum;       /* quantum in usecs */
} NkSchedParams;

    /* This is used by Performance Monitoring */
typedef nku64_f NkTime;
typedef nku32_f NkFreq;
typedef nku32_f NkPmonState;

typedef nku32_f NkPResourceId;	/* VLINK persistent resource identifier */

typedef nku32_f NkBool;		/* boolean value: true/false (1/0) */

#endif

#endif
