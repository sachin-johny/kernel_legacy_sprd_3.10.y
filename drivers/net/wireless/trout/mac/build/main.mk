###############################################################################
#                                                                             #
#                      Ittiam 802.11n MAC SOFTWARE                            #
#                                                                             #
#                   ITTIAM SYSTEMS PVT LTD, BANGALORE                         #
#                          COPYRIGHT(C) 2009                                  #
#                                                                             #
#   This program is proprietary to Ittiam Systems Pvt. Ltd. and is protected  #
#   under Indian Copyright Law as an unpublished work. Its use and            #
#   disclosure is limited by the terms and conditions of a license agreement. #
#   It may not be copied or otherwise reproduced or disclosed to persons      #
#   outside the licensee's organization except in accordance with the terms   #
#   and conditions of such an agreement. All copies and reproductions shall   #
#   be the property of Ittiam Systems India Pvt. Ltd. and must bear this      #
#   notice in its entirety.                                                   #
#                                                                             #
#   File Name         : config.mk                                             #
#                                                                             #
#   Description       : This is the Makefile used to build the Linux project  #
#                       for MAC software.                                     #
#                                                                             #
#   Issues            : None                                                  #
#                                                                             #
#   Revision History  : DD MM YYYY   Author(s)       Changes                  #
#                       11 02 2009   Ittiam          Baseline                 #
#                                                                             #
###############################################################################
#MAC_C_ROOT = $(ANDROID_BUILD_PATHS)/kernel/drivers/net/wireless/trout 

include $(MAC_BUILD_DIR)/config.mk
include $(MAC_BUILD_DIR)/adv_config.mk

obj-m += $(TROUT_KO_NAME).o

#ifneq ( ,$(findstring -DTROUT_SDIO_INTERFACE,$(EXTRA_OPTION_FLAGS))) #use sdio

#ifneq ( ,$(findstring -DIBSS_BSS_STATION_MODE,$(MAC_MODE_FLAGS))) #use sta
#obj-m += itm_sta.o
#endif

#ifneq ( ,$(findstring -DBSS_ACCESS_POINT_MODE,$(MAC_MODE_FLAGS))) #use sta
#obj-m += itm_ap.o
#endif

#else #use spi

#ifneq ( ,$(findstring -DIBSS_BSS_STATION_MODE,$(MAC_MODE_FLAGS))) #use sta
#obj-m += itm_sta.o
#endif

#ifneq ( ,$(findstring -DBSS_ACCESS_POINT_MODE,$(MAC_MODE_FLAGS))) #use sta
#obj-m += itm_ap.o
#endif

#endif


# macro DV_SIM is used in dv test.
#EXTRA_CFLAGS += -D DV_SIM
#obj-m += sim/

ifneq ( ,$(findstring -DTROUT_B2B_TEST_MODE,$(EXTRA_OPTION_FLAGS)))
obj-m += testDev/
endif

include $(MAC_BUILD_DIR)/obj_list.mk

KERNEL_VERSION :=2.6.13smp 
# KERNEL_SOURCE =/usr/src/linux-2.6.13

INC_PATH     = -I$(KERNEL_SOURCE)/include -I$(KERNEL_SOURCE)/include/linux\
	-I$(COMMON_INCS) -I$(CON_INCS) -I$(CONAP_INCS) -I$(CONSTA_INCS) \
	-I$(CONIF_INCS) -I$(CORE_INCS) -I$(COREAP_INCS) -I$(CORESTA_INCS) \
	-I$(EDGEPHY_INCS_ITTIAM) -I$(MODEIF_INCS) -I$(EDGEIF_INCS) \
	-I$(EDGEPROC_PPC_INCS) \
	-I$(EDGEHOSTSDIO_INCS) -I$(EDGEHOSTUART_INCS) \
	-I$(EDGEPHY_INCS_11g) \
	-I$(EDGEPHY_INCS_11a) \
	-I$(EDGEPHY_INCS_11b) \
	-I$(EDGEPHY_INCS_11n) \
	-I$(EDGERF_ITTMAX_INCS) \
	-I$(EDGERF_ITTAIR_INCS) \
	-I$(EDGERF_GENERIC_INCS) \
	-I$(EDGEMACHW_INCS) -I$(CSL_INCS) \
	-I$(MM_INCS) -I$(QM_INCS) -I$(MSTEST_INCS) -I$(STAPROTIF_INCS)\
	-I$(APSTAPROTIF_INCS) -I$(APPROTIF_INCS) -I$(MHTEST_INCS)\
	-I$(APSTAM11E_INCS) -I$(APM11E_INCS) -I$(STAM11E_INCS) \
	-I$(APM11I_INCS) -I$(STAM11I_INCS) -I$(APSTAM11I_INCS) \
	-I$(WPSR_COMMON_INCS) -I$(WPSR_PROTOCOL_INCS) -I$(WPSR_REGISTRAR_INCS) \
	-I$(WPSR_IF_INCS) -I$(WPSE_INCS)\
	-I$(APM11N_INCS) -I$(STAM11N_INCS) -I$(APSTAM11N_INCS) \
	-I$(APM1X_INCS) -I$(STAMWPS_INCS) -I$(APMWPS_INCS)\
	-I$(APSTAMP2P_INCS) -I$(STAMP2P_INCS) -I$(APMP2P_INCS)\
	-I$(CONSTAPROT_INCS) -I$(CONSTAP2P_INCS)\
	-I$(APM11H_INCS) -I$(STAM11H_INCS) -I$(APSTAM11H_INCS)\
	-I$(UTILS_INCS) \
	-I$(TRW_INCS)\
	-I$(RUNMODE_INCS)\
	-I$(METER_INCS) \
	-I$(APMDOM_INCS) -I$(STAMDOM_INCS) -I$(APSTAMDOM_INCS) # 20120830 caisf add, merged ittiam mac v1.3 code

ifeq ($(CONFIG_ITTIAM_MAC),y)
# MAC Mode
ifeq ($(CONFIG_ITTIAM_STA),y)
EXTRA_CFLAGS    += -DIBSS_BSS_STATION_MODE 
endif # mode
ifeq ($(CONFIG_ITTIAM_AP),y)
EXTRA_CFLAGS    += -DBSS_ACCESS_POINT_MODE
endif # mode
ifeq ($(CONFIG_ITTIAM_DUT),y)
EXTRA_CFLAGS    += -DMAC_HW_UNIT_TEST_MODE
endif # mode

#Host support
ifeq ($(CONFIG_ITTIAM_ETH_HOST),y)
EXTRA_CFLAGS    += -DETHERNET_HOST
endif #CONFIG_ITTIAM_ETH_HOST

ifeq ($(CONFIG_ITTIAM_BIND_STACK),y)
EXTRA_CFLAGS    += -DETHERNET_DRIVER
endif #CONFIG_ITTIAM_BIND_STACK

ifeq ($(CONFIG_ITTIAM_DISABLE_LINUX_STACK),y)
EXTRA_CFLAGS    += -DLINUX_DRIVER_DISABLE
endif #CONFIG_ITTIAM_DISABLE_LINUX_STACK

#802_11E Support
ifeq ($(CONFIG_ITTIAM_11E),y)
EXTRA_CFLAGS    += -DMAC_802_11E -DEDCA_DEMO_KLUDGE
endif #802_11E

#802_11I Support
ifeq ($( CONFIG_ITTIAM_11I),y)
EXTRA_CFLAGS    += -DMAC_802_11I

#802_1X Support
ifeq ($(CONFIG_ITTIAM_1X),y)
EXTRA_CFLAGS    += -DMAC_802_1X
endif # 802_1X
endif # 802_11I

#802_11N Support
ifeq ($(CONFIG_ITTIAM_11N),y)
EXTRA_CFLAGS    += -DMAC_802_11N
endif # 802_11N

EXTRA_CFLAGS    += $(EDGE_FLAGS) $(RF_FLAGS) $(PROC_FLAGS) \
        $(INC_PATH) $(CSL_FLAGS) $(EXTRA_OPTION_FLAGS)

else # Ittiam MAC

EXTRA_CFLAGS    += $(EDGE_FLAGS) $(RF_FLAGS) \
		$(PROC_FLAGS) $(HOST_FLAGS) $(INC_PATH) $(CSL_FLAGS) \
        $(MAC_EDGE_FLAGS) $(MAC_MODE_FLAGS) $(EXTRA_OPTION_FLAGS) 


endif # Ittiam MAC

#EXTRA_CFLAGS += -g		#add by chengwg.

###############################################################################
# Rule to clear all object/binary files - Command is 'make clean'             #
###############################################################################
default: 
	make -C ${KERNEL_SOURCE} M=`pwd` modules 
	$(warning "strip...")
	arm-eabi-strip -d $(MAC_BUILD_DIR)/$(MAC_MODE)/$(TROUT_KO_NAME).ko
clean: 
	make -C ${KERNEL_SOURCE} M=`pwd` clean
