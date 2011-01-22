/*
 ****************************************************************
 *
 * Component = NKERNEL
 *
 * Copyright (C) 2003, VirtualLogix. All Rights Reserved.
 *
 * Use of this product is contingent on the existence of an executed license
 * agreement between VirtualLogix or one of its sublicensee, and your
 * organization, which specifies this software's terms of use. This software
 * is here defined as VirtualLogix Intellectual Property for the purposes
 * of determining terms of use as defined within the license agreement.
 *
 * #ident  "@(#)bconf_f.h 1.3     07/02/01 VirtualLogix"
 *
 * Contributor(s):
 *   Guennadi Maslov <guennadi.maslov@virtuallogix.com>
 *
 ****************************************************************
 */

#ifndef _NK_BOOT_BCONF_F_H
#define _NK_BOOT_BCONF_F_H

	/*
	 * These typedefs are used only in bconf
	 */

typedef unsigned char      uint8_f;
typedef unsigned short     uint16_f;
typedef unsigned int       uint32_f;
typedef unsigned long long uint64_f;

typedef uint32_f VmAddr;
typedef uint32_f VmSize;
typedef uint32_f PhAddr;
typedef uint32_f PhSize;

#endif
