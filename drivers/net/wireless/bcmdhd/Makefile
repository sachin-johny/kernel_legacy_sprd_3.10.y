# bcmdhd

DHDCFLAGS += -Wall -Wstrict-prototypes -Dlinux  -DLINUX -DBCMDRIVER     \
    -DBCMDONGLEHOST -DUNRELEASEDCHIP -DBCMDMA32 -DBCMFILEIMAGE          \
    -DDHDTHREAD -DSHOW_EVENTS -DBCMDBG -DWLP2P -DWIFI_ACT_FRAME         \
    -DKEEP_ALIVE  -DPKT_FILTER_SUPPORT -DARP_OFFLOAD_SUPPORT            \
    -DEMBEDDED_PLATFORM -DPNO_SUPPORT



DHDCFLAGS += -DCUSTOMER_HW2
DHDCFLAGS += -DWL_CFG80211

# VSDB
DHDCFLAGS += -DVSDB

# For p2p connection issue
DHDCFLAGS += -DWL_SCB_TIMEOUT=10

# TDLS enable
DHDCFLAGS += -DWLTDLS -DWLTDLS_AUTO_ENABLE
# For TDLS tear down inactive time 40 sec
DHDCFLAGS += -DCUSTOM_TDLS_IDLE_MODE_SETTING=40000
# for TDLS RSSI HIGH for establishing TDLS link
DHDCFLAGS += -DCUSTOM_TDLS_RSSI_THRESHOLD_HIGH=-60
# for TDLS RSSI HIGH for tearing down TDLS link
DHDCFLAGS += -DCUSTOM_TDLS_RSSI_THRESHOLD_LOW=-70

# Roaming
DHDCFLAGS += -DROAM_AP_ENV_DETECTION
DHDCFLAGS += -DROAM_ENABLE
DHDCFLAGS += -DENABLE_FW_ROAM_SUSPEND
# Roaming trigger
DHDCFLAGS += -DCUSTOM_ROAM_TRIGGER_SETTING=-75
DHDCFLAGS += -DCUSTOM_ROAM_DELTA_SETTING=10
# Set PM 2 always regardless suspend/resume
DHDCFLAGS += -DSUPPORT_PM2_ONLY

# For special PNO Event keep wake lock for 10sec
DHDCFLAGS += -DCUSTOM_PNO_EVENT_LOCK_xTIME=10
DHDCFLAGS += -DMIRACAST_AMPDU_SIZE=8

# Early suspend
DHDCFLAGS += -DDHD_USE_EARLYSUSPEND

# For Scan result patch
DHDCFLAGS += -DESCAN_RESULT_PATCH

# For Static Buffer
ifeq ($(CONFIG_BROADCOM_WIFI_RESERVED_MEM),y)
  DHDCFLAGS += -DCONFIG_DHD_USE_STATIC_BUF
  DHDCFLAGS += -DENHANCED_STATIC_BUF
  DHDCFLAGS += -DSTATIC_WL_PRIV_STRUCT
endif

ifneq ($(CONFIG_DHD_USE_SCHED_SCAN),)
DHDCFLAGS += -DWL_SCHED_SCAN
endif

# Ioctl timeout 5000ms
DHDCFLAGS += -DIOCTL_RESP_TIMEOUT=5000

# Prevent rx thread monopolize
DHDCFLAGS += -DWAIT_DEQUEUE

# Use Android wake lock mechanism
DHDCFLAGS += -DCONFIG_HAS_WAKELOCK

# idle count
DHDCFLAGS += -DDHD_USE_IDLECOUNT

# SKB TAILPAD to avoid out of boundary memory access
DHDCFLAGS += -DDHDENABLE_TAILPAD

# Wi-Fi Direct
DHDCFLAGS += -DWL_CFG80211_VSDB_PRIORITIZE_SCAN_REQUEST
DHDCFLAGS += -DWL_CFG80211_STA_EVENT
DHDCFLAGS += -DWL_IFACE_COMB_NUM_CHANNELS
DHDCFLAGS += -DWL_ENABLE_P2P_IF

################################
# MPS platform specific features
#################################

DHDCFLAGS += -DHW_OOB
DHDCFLAGS += -DGET_CUSTOM_MAC_ENABLE
#DHDCFLAGS += -DSUPPORT_MULTIPLE_REVISION#yangyufeng debug
DHDCFLAGS += -DSUPPORT_MULTIPLE_CHIPS
DHDCFLAGS += -DPLATFORM_MPS


DHDCFLAGS += -DCUSTOM_RXCHAIN=1
DHDCFLAGS += -DBCMSDIOH_TXGLOM 
DHDCFLAGS += -DBCMSDIOH_TXGLOM_HIGHSPEED
DHDCFLAGS += -DDHDTCPACK_SUPPRESS
DHDCFLAGS += -DUSE_WL_TXBF
DHDCFLAGS += -DUSE_WL_FRAMEBURST
DHDCFLAGS += -DRXFRAME_THREAD
DHDCFLAGS += -DCUSTOM_AMPDU_BA_WSIZE=64
DHDCFLAGS += -DCUSTOM_DPC_CPUCORE=0
DHDCFLAGS += -DPROP_TXSTATUS_VSDB
DHDCFLAGS += -DCUSTOM_MAX_TXGLOM_SIZE=32
DHDCFLAGS += -DWL11U
DHDCFLAGS += -DDHD_ENABLE_LPC
DHDCFLAGS += -DCUSTOM_PSPRETEND_THR=30
DHDCFLAGS += -DENABLE_INSMOD_NO_FW_LOAD#yangyufeng debug

#########
# Others
#########

ifneq ($(CONFIG_BCMDHD_SDIO),)
  DHDCFLAGS += -DBDC -DOOB_INTR_ONLY -DDHD_BCMEVENTS -DMMC_SDIO_ABORT
  DHDCFLAGS += -DBCMSDIO -DBCMLXSDMMC -DUSE_SDIOFIFO_IOVAR
  DHDCFLAGS += -DPROP_TXSTATUS
endif

ifneq ($(CONFIG_BCMDHD_PCIE),)
  DHDCFLAGS += -DPCIE_FULL_DONGLE -DBCMPCIE -DCUSTOM_DPC_PRIO_SETTING=-1
endif

ifeq ($(CONFIG_BCMDHD),m)
EXTRA_LDFLAGS += --strip-debug
endif

ifeq ($(CONFIG_BCMDHD),y)
#  DHDCFLAGS += -DENABLE_INSMOD_NO_FW_LOAD#yangyufeng debug
  DHDCFLAGS += -DUSE_LATE_INITCALL_SYNC
endif

EXTRA_CFLAGS += $(DHDCFLAGS) -DDHD_DEBUG
EXTRA_CFLAGS += -DSRCBASE=\"$(src)\"
EXTRA_CFLAGS += -I$(src)/include/ -I$(src)/common/include/ -I$(src)/
KBUILD_CFLAGS += -I$(LINUXDIR)/include -I$(shell pwd)

DHDOFILES = aiutils.o dhd_linux.o siutils.o bcmutils.o   \
	dhd_linux_sched.o dhd_pno.o bcmwifi_channels.o bcmevent.o hndpmu.o   \
	dhd_common.o dhd_ip.o linux_osl.o    \
	dhd_custom_gpio.o sbutils.o wldev_common.o wl_android.o     \
	wl_cfg80211.o wl_cfgp2p.o wl_linux_mon.o dhd_cfg80211.o \
	dhd_linux_platdev.o dhd_linux_wq.o wl_cfg_btcoex.o

ifneq ($(CONFIG_BCMDHD_SDIO),)
  DHDOFILES += bcmsdh.o bcmsdh_linux.o bcmsdh_sdmmc.o bcmsdh_sdmmc_linux.o
  DHDOFILES += dhd_cdc.o dhd_wlfc.o dhd_sdio.o
endif

ifneq ($(CONFIG_BCMDHD_PCIE),)
  DHDOFILES += dhd_pcie.o dhd_pcie_linux.o dhd_msgbuf.o circularbuf.o
endif

bcmdhd-objs := $(DHDOFILES)
obj-$(CONFIG_BCMDHD) += bcmdhd.o

all:
	@echo "$(MAKE) --no-print-directory -C $(KDIR) SUBDIRS=$(CURDIR) modules"
	@$(MAKE) --no-print-directory -C $(KDIR) SUBDIRS=$(CURDIR) modules

clean:
	rm -rf *.o *.ko *.mod.c *~ .*.cmd *.o.cmd .*.o.cmd \
	Module.symvers modules.order .tmp_versions modules.builtin

install:
	@$(MAKE) --no-print-directory -C $(KDIR) \
		SUBDIRS=$(CURDIR) modules_install
