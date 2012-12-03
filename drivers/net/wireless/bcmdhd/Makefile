# bcmdhd

DHDCFLAGS = -Wall -Wstrict-prototypes -DLINUX -DBCMDRIVER 		\
	-DBCMDONGLEHOST -DUNRELEASEDCHIP -DBCMDMA32 				\
	-DBCMFILEIMAGE -DDHDARCH_ARM -DDHDTHREAD -DWLBTAMP 	\
	-DBDC -DTOE -DDHD_BCMEVENTS -DSHOW_EVENTS -DARP_OFFLOAD_SUPPORT 	\
	-DBCMSDIO  -DTESTDONGLE  -DOEM_ANDROID 		\
	-DBCMLXSDMMC -DBCMPLATFORM_BUS -Dlinux 			\
	-DOEM_ANDROID -DEMBEDDED_PLATFORM -USDIO_ISR_THREAD -USOFTAP 		\
	-DENABLE_INSMOD_NO_FW_LOAD -DCUSTOM_OOB_GPIO_NUM=142 		\
	-DOOB_INTR_ONLY -DMMC_SDIO_ABORT -DCONFIG_WIFI_CONTROL_FUNC 		\
	-Dlinux -DOEM_ANDROID -DBCMDBG -DCONFIG_DHD_STATIC_BUF 			\
	-DCUSTOMER_HW2 -DBCMOKC -DKEEP_ALIVE -DBCMSDIO -DBCMLXSDMMC 		\
	-DBCMPLATFORM_BUS -DEMBEDDED_PLATFOR -DWL_CFG80211 -DWLP2P 		\
	-DWIFI_ACT_FRAM	-DGET_CUSTOM_MAC_ENABLE	-DPKT_FILTER_SUPPORT -DSHOW_EVENTS 	\
	-Idrivers/net/wireless/bcmdhd/	-Idrivers/net/wireless/bcmdhd/include

DHDOFILES = aiutils.o bcmsdh_sdmmc_linux.o dhd_linux.o siutils.o bcmutils.o   \
	dhd_linux_sched.o bcmwifi.o dhd_sdio.o bcmevent.o dhd_bta.o hndpmu.o  \
	bcmsdh.o dhd_cdc.o bcmsdh_linux.o dhd_common.o linux_osl.o            \
	dhd_custom_gpio.o sbutils.o wldev_common.o wl_android.o \

ifneq ($(CONFIG_WLAN_SDIO),)
DHDOFILES += bcmsdh_sdmmc.o
else
DHDOFILES += bcmspibrcm.o bcmsdspi_linux.o  bcmpcispi.o
DHDCFLAGS += -DSPRD_SPI -DBCMSDIOH_SPI -DBCMSPI
endif

obj-$(CONFIG_BCMDHD) += bcmdhd.o
bcmdhd-objs += $(DHDOFILES)
ifneq ($(CONFIG_WIRELESS_EXT),)
bcmdhd-objs += wl_iw.o
DHDCFLAGS += -DSOFTAP	-DCSCAN
endif
ifneq ($(CONFIG_CFG80211),)
bcmdhd-objs += dhd_cfg80211.o wl_cfg80211.o wl_cfgp2p.o wl_linux_mon.o
DHDCFLAGS += -DWL_CFG80211
endif
DHDCFLAGS += -DBCMWAPI_WPI -DBCMWAPI_WAI
EXTRA_CFLAGS = $(DHDCFLAGS)
ifeq ($(CONFIG_BCMDHD),m)
EXTRA_LDFLAGS += --strip-debug
endif
