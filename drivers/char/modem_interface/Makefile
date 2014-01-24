# 
# Makefile for DWC_otg Highspeed USB controller driver
#

#PWD		:= $(shell pwd)
#PORTLIB		:= $(PWD)/../dwc_common_port

ifeq ($(CONFIG_MODEM_INTF_DEBUG),y)
        EXTRA_CFLAGS            += -DDEBUG
endif

sprdboard-$(CONFIG_MACH_SP7710GA_ORANGE) += board_sp7710ga_orange/
sprdboard-$(CONFIG_MACH_SP8810EA) += board-sp8810ea/
sprdboard-$(CONFIG_MACH_SP8810EC) += board-sp8810ec/
sprdboard-$(CONFIG_MACH_SP8810GA) += board-sp8810ga/
sprdboard-$(CONFIG_MACH_SP8810GB) += board-sp8810gb/
sprdboard-$(CONFIG_MACH_SP6820GA) += board-sp6820ga/
sprdboard-$(CONFIG_MACH_AMAZING)  += board-amazing/
sprdboard-$(CONFIG_MACH_KYLETD)  += board-kyletd/
sprdboard-$(CONFIG_MACH_KYLEW)  += board-kylew/
sprdboard-$(CONFIG_MACH_MINT)  += board-mint/
sprdboard-$(CONFIG_MACH_SP8810EB) += board-sp8810eb/
sprdboard-$(CONFIG_MACH_Z788)  += board-z788/
sprdboard-$(CONFIG_MACH_SP7710G_OPENPHONE)  += board-sp7710g_openphone/
sprdboard-$(CONFIG_MACH_SP7702) += board-sp7702/
sprdboard-$(CONFIG_MACH_OPENPHONE_SC6820) += board-sc6820-openphone/
sprdboard-$(CONFIG_MACH_SP7710GA)  += board-sp7710ga/
sprdboard-$(CONFIG_MACH_SP7710GA_TROUT)  += board-sp7710ga_trout/
sprdboard-$(CONFIG_MACH_NEVISTD)  += board-nevistd/

sprdboarddirs := $(patsubst %,arch/arm/mach-sc8810/%,$(sprdboard-y))
$(warning $(sprdboarddirs))
KBUILD_CPPFLAGS += $(patsubst %,-I%,$(sprdboarddirs))
$(warning $(KBUILD_CPPFLAGS))
obj-$(CONFIG_MODEM_INTF) += modem_interface.o

modem_interface-objs	:= boot_driver.o boot_protocol.o modem_buffer.o
modem_interface-objs	+= normal_mode.o  modem_gpio_drv.o
modem_interface-objs	+= modem_sdio_drv.o modem_interface_driver.o
