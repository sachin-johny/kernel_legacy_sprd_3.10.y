ifeq (y,$(CONFIG_TROUT_WIFI_POWER_SLEEP_ENABLE))
EXTRA_CFLAGS += -DTROUT_WIFI_POWER_SLEEP_ENABLE
endif
EXTRA_CFLAGS += -DTROUT_PDN_ENABLE                    #Hugh: use trout powerdown pin.
EXTRA_CFLAGS += -DNPI_NV_CALIBRATION_ENABLE           #Caisf: npi nv calibration
EXTRA_CFLAGS += -DTROUT2_CHIP_VER_V2                  #wlh: judge the chip version 
obj-m := trout_sdio.o
trout_sdio-objs += trout2_interface.o
trout_sdio-objs += trout2_sdio.o
trout_sdio-objs += bt_ram_code.o
trout_sdio-objs += mxd_castor_sdk.o
trout_sdio-objs += phy_rw.o
