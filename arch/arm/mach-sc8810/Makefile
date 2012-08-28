obj-y += io.o irq.o timer.o devices.o arch_asm.o arch_init.o globalregs.o \
         clock_common.o clock_sc8810.o regulator.o regulator-devices.o \
         dma.o reserve.o pinmap.o adi.o gpio_eic.o adc.o efuse.o dcdc_cal.o \
         watchdog.o

ifeq ($(CONFIG_NKERNEL),y)
obj-$(CONFIG_PM) += pm_vlx.o pm_sc8810.o standby.o pm_debug.o
else
obj-$(CONFIG_PM) += pm.o
endif

obj-$(CONFIG_SPRD_DCDC_DEBUG) += dcdc_debug.o

obj-$(CONFIG_DEBUG_FS) += clock_debug.o

obj-$(CONFIG_MACH_SP8810EA) += board-sp8810ea.o
obj-$(CONFIG_MACH_SP8810EC) += board-sp8810ec.o
obj-$(CONFIG_MACH_SP8810GA) += board-sp8810ga.o
obj-$(CONFIG_MACH_SP8810GB) += board-sp8810gb.o
obj-$(CONFIG_MACH_SP6820GA) += board-sp6820ga.o
obj-$(CONFIG_MACH_AMAZING)  += board-amazing.o
obj-$(CONFIG_MACH_SP8810EB) += board-sp8810eb.o
obj-$(CONFIG_MACH_Z788)  += board-z788.o
obj-$(CONFIG_MACH_SP7710G_OPENPHONE)  += board-sp7710g_openphone.o
obj-$(CONFIG_SC8810_VIBRATOR) += vibrator.o
obj-$(CONFIG_ANDROID_RAM_CONSOLE) += ram_console.o
obj-$(CONFIG_MACH_OPENPHONE_SC6820) += board-sc6820-openphone.o
obj-$(CONFIG_CPU_FREQ_SC8810) += cpufreq-sc8810.o
obj-$(CONFIG_BCMDHD)  += dhd_adapter.o
obj-$(CONFIG_SPRD_MEM_POOL)    += sprd_mem_pool.o
ifeq ($(CONFIG_RTL8723AS),m)
obj-y  += rtl8723_adapter.o
endif
