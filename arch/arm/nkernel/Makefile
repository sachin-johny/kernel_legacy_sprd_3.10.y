#
#***************************************************************
#
# Component = Nano-Kernel device driver interface (NK DDI)
#
# Copyright (C) 2002-2005 Jaluna SA.
#
# This program is free software;  you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# #ident  "@(#)Makefile 1.1     07/10/18 VirtualLogix"
#
# Contributor(s):
#	Vladimir Grouzdev <vladimir.grouzdev@jaluna.com>
#	Guennadi Maslov <guennadi.maslov@jaluna.com>
#	Chi Dat Truong <chidat.truong@jaluna.com>
#
#***************************************************************
#

# Each configuration option enables a list of files.

obj-y			:= console.o ddi.o perfmon.o
obj-$(CONFIG_PROC_FS)	+= proc.o nktags.o
obj-$(CONFIG_SMP)	+= smp.o

ifneq ($(CONFIG_ARCH_SC8800G), y)
ifeq ($(CONFIG_GENERIC_CLOCKEVENTS),y)
obj-y += vtimer.o
else
obj-y += vtick.o
endif
endif

