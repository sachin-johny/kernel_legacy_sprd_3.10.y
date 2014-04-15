ccflags-y += -DDEBUG
#ccflags-y += -DDUMP_TRANSMIT_PACKET
#ccflags-y += -DDUMP_RECEIVE_PACKET
#ccflags-y += -DDUMP_COMMAND
#ccflags-y += -DDUMP_COMMAND_RESPONSE
#ccflags-y += -DDUMP_EVENT

ccflags-y += -DCONFIG_ITM_WLAN_ENHANCED_PM

ccflags-y += -DCONFIG_ITM_WLAN_FW_ZEROCOPY

obj-$(CONFIG_ITTIAM) += ittiam.o

ittiam-objs:= main.o cfg80211.o sipc.o \
	      hostap_conf.o npi.o wifi_nvm.o \
	      wapi.o \
	      $(ittiam-y)
