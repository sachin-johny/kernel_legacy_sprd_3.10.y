ccflags-y += -DDEBUG
#ccflags-y += -DDUMP_TRANSMIT_PACKET
#ccflags-y += -DDUMP_RECEIVE_PACKET
#ccflags-y += -DDUMP_COMMAND
#ccflags-y += -DDUMP_COMMAND_RESPONSE
#ccflags-y += -DDUMP_EVENT

ccflags-y += -DCONFIG_ITM_WLAN_ENHANCED_PM

obj-$(CONFIG_ITTIAM) += ittiam.o

ittiam-objs:= itm_main.o itm_cfg80211.o itm_sipc.o \
	      itm_hostap_conf.o itm_npi.o wifi_nvm.o \
	      itm_wapi.o \
	      $(ittiam-y)
