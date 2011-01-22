/*
 ****************************************************************
 *
 *  Copyright (C) 2002-2009, VirtualLogix. All Rights Reserved.
 *
 * This program is free software;  you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Contributor(s):
 *	Vladimir Grouzdev <vladimir.grouzdev@vlx.com>
 *
 ****************************************************************
 */

#ifndef ASMARM_NKSMP_H
#define ASMARM_NKSMP_H

#include <asm/nkern.h>

struct cpumask;

#define hard_smp_processor_id()	(VCPU()->vcpuid)

    extern void
smp_cross_call (const struct cpumask* mask);

#endif
