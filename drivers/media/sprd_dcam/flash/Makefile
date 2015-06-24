
ifeq ($(CONFIG_MACH_VIVALTO),y)
obj-y += flash_vivalto.o
else
ifeq ($(CONFIG_MACH_KANAS_W),y)
obj-y += flash_kanas_w.o
else
ifeq ($(CONFIG_MACH_SP8730SEA),y)
obj-y += flash_sgm3140.o
else
ifeq ($(CONFIG_MACH_SP9630EA_4MOD),y)
obj-y += flash_parallel_connect_bg.o
else
ifeq ($(CONFIG_MACH_SP9630EA4MN),y)
obj-y += flash_parallel_connect_bg.o
else
ifeq ($(CONFIG_MACH_SP9830AEA_5M_H100), y)
obj-y += flash_parallel_connect_bg.o
else
ifeq ($(CONFIG_FLED_SM5701),y)
obj-y += flash_sm5701.o
else
obj-y += flash_common.o
endif
endif
endif
endif
endif
endif
endif
