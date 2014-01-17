KERNEL_SOURCE =$(ANDROID_BUILD_TOP)/kernel
FM_ROOT := $(KERNEL_SOURCE)/drivers/media/radio/trout_fm

include $(FM_ROOT)/config.mk

#EXTRA_CFLAGS += -DTROUT_WIFI_POWER_SLEEP_ENABLE
EXTRA_CFLAGS += -DTROUT_PDN_ENABLE                    #Hugh: use trout powerdown pin.

obj-m := trout_fm.o

trout_fm-objs := trout_fm_ctrl.o
trout_fm-objs += trout_fm_audio.o 
trout_fm-objs += trout_rf_common.o 

ifeq ($(CONFIG_RF), RF_2829)
EXTRA_CFLAGS += -DUSE_RF_2829
trout_fm-objs += trout_rf_2829.o
endif

ifeq ($(CONFIG_RF), RF_55)
EXTRA_CFLAGS += -DUSE_RF_55
trout_fm-objs += trout_rf_55.o
endif

ifeq ($(CONFIG_INTERFACE), INTERFACE_SHARED)
EXTRA_CFLAGS += -DUSE_INTERFACE_SHARED
trout_fm-objs += trout_shared.o
endif
