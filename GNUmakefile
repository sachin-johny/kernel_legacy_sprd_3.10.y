#
#***************************************************************
#
# Component = Linux Kernel Source Component.
#
# Copyright (C) 2002-2007 VirtualLogix SA.
#
# This program is free software;  you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# #ident  "@(#)GNUmakefile 1.68     07/10/23 VirtualLogix"
#
# Contributor(s):
#   Sebastien Laborie (sebastien.laborie@virtuallogix.com)
#   Chi Dat Truong    (chidat.truong@virtuallogix.com)
#   Guennadi Maslov   (guennadi.maslov@virtuallogix.com)
#
#***************************************************************
#

# Include components definitions
include GNUmakefile.defs

# Include the profile file
ifneq ($(LINUXKERNEL_MERGE_PROFILE),)
include $(LINUXKERNEL_MERGE_PROFILE)
endif

#
# The following variables are setup in the board profile file:
#
# - ARCH            Linux architecture (name of the directory under arch/)
# - BSP             Board name of Linux config file (in arch/<arch>/configs/)
# - MMU             MMU support (yes/no)
# - OSWARE          OSware support (yes/no)
# - CROSS_COMPILE   Compiler prefix (e.g. arm-linux-, powerpc-chorusos-)
#
# Note: A default value is determined and assigned to each variable
#       that it is not explicitly defined in the board profile file.
#

#
# Default Linux architecture.
#

ARCH                           ?= i386

#
# Default board configuration file.
#

ifneq ($(BSP),)
BSP_CONFIG                     ?= "arch/$(ARCH)/configs/$(BSP)_defconfig"
endif

#
# Helper function.
#

IS_CONFIGURED=$(if $(shell . ./.config; echo $${CONFIG_$(1)}),yes,no)

#                                     
# Default MMU support.
#

MMU                            ?= $(call IS_CONFIGURED,MMU)

#
# Default OSware support.
#

OSWARE                         ?= $(call IS_CONFIGURED,NKERNEL)

#
# Determine the cross compiler prefix name.
#

CROSS_COMPILE_i386              = i386-linux-
CROSS_COMPILE_ppc               = powerpc-linux-
CROSS_COMPILE_arm               = arm-linux-
CROSS_COMPILE_armeb             = armeb-linux-
CROSS_COMPILE_mips              = mips-linux-
CROSS_COMPILE_mipsel            = mipsel-linux-
CROSS_COMPILE                  ?= $(PREFIX)$(CROSS_COMPILE_$(ARCH))

#
# Determine the cross compiler path.
#

PREFIX=
ifneq ($(CDS_DIR),)
  PREFIX=$(CDS_DIR)/bin/
endif

#
# Force the use of the ncurses provided by the GNU component if any.
#

ifneq ($(GNU_DIR),)
       NCURSES_CFLAGS  = -DLOCALE
       NCURSES_CFLAGS += -I$(GNU_DIR)/include -DCURSES_LOC="<ncurses.h>"
       NCURSES_LIBS    = -L$(GNU_DIR)/lib -lncurses
       NCURSES_VARS    = HOST_EXTRACFLAGS='$(NCURSES_CFLAGS)' \
	                 HOST_LOADLIBES='$(NCURSES_LIBS)'
export TERMINFO       ?= $(GNU_DIR)/share/terminfo
endif

#
# Update the PATH.
#

ifneq ($(GNU_DIR),)
PATH_HOST  = $(subst $(GNU_DIR)/bin:,,$(PATH))
PATH       = $(GNU_DIR)/bin:$(shell echo $$PATH)
else
PATH_HOST  = $(shell echo $$PATH)
endif

#
# VARS contains the Linux Makefile variables to be setup.
#

# bash is really assumed by the kernel Makefiles
VARS                    = SHELL=/bin/bash

# The architecture
ifneq ($(ARCH),)
VARS                   += ARCH=$(ARCH)
endif

# The cross-compiler prefix
ifneq ($(CROSS_COMPILE),)
VARS                   += CROSS_COMPILE=$(CROSS_COMPILE)
endif

# The verbose level
V                      := 0
VARS                   += V=$(V)

# The extra version
EXTRAVERSION            = -virtuallogix
VARS                   += EXTRAVERSION=$(EXTRAVERSION)

# Two different versions of gcc are likely to be used for compiling host
# programs: the one provided by the GNU component and, for the xconfig and
# the gconfig rules, the one provided by the host.
# In order to recompile tools when the compiler is changed, the HOSTCC
# variable is set explicitly (see Linux depends rules).

# Allow to overload HOSTCC

ifneq ($(HOSTCC),)
VARS                   += HOSTCC=$(HOSTCC)
else
ifneq ($(GNU_DIR),)
VARS                   += HOSTCC=$(GNU_DIR)/bin/gcc
else
VARS                   += HOSTCC=gcc
endif
endif

# The uboot mkimage tool
ifneq ($(BOOT_UTILS_DIR),)
_BDIR=$(BOOT_UTILS_DIR)/bin
VARS                   += MKIMAGE_UBOOT=$(_BDIR)/mkimage_uboot.sh
VARS                   += MKIMAGE_RRLOAD=$(_BDIR)/mkimage_rrload
endif

all:

FORCE:;

VMLINUX.conf: FORCE
	rm -f $@
	echo "ARCH=$(ARCH)" > $@.tmp
	echo "BSP=$(BSP)" >> $@.tmp
	echo "MMU=$(MMU)" >> $@.tmp
	echo "OSWARE=$(OSWARE)" >> $@.tmp
	LINUX_VER=`( cat Makefile ; \
	             echo "" ; echo 'echover:; @echo $$(KERNELVERSION)' \
	           ) | \
	           $(MAKE) -sf - echover dot-config=0 $(VARS) | \
	           grep '^[0-9]'` ; \
	if test -z "$$LINUX_VER"; \
	then \
	  echo "Cannot compute Linux version"; exit 1 ; \
	fi ; \
	echo "LINUX_VER=$$LINUX_VER" >> $@.tmp
	mv -f $@.tmp $@

.config:
	if test ! -f $(BSP_CONFIG); \
	then \
	  echo "Missing board configuration file"; exit 1 ; \
	fi
	cp -f $(BSP_CONFIG) $@

menuconfig:
	$(MAKE) $(MAKE_PARALLEL) -f Makefile $@ $(VARS) $(NCURSES_VARS)

xconfig gconfig: PATH=$(PATH_HOST)
xconfig gconfig:
	$(MAKE) $(MAKE_PARALLEL) -f Makefile $@ $(VARS) HOSTCC=gcc

all %: 
	$(MAKE) $(MAKE_PARALLEL) -f Makefile $@ $(VARS)
