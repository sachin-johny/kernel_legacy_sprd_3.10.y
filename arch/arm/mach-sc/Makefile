obj-y += irq.o timer.o arch_init.o globalregs.o arch_lock.o \
         reserve.o adi.o adie_irq.o adc.o efuse.o \
         watchdog.o common.o glb.o clock.o

obj-$(CONFIG_ARCH_SC8825) += dcdc_cal.o
obj-$(CONFIG_ARCH_SC8825) += devices-sc8825.o io-sc8825.o
obj-$(CONFIG_ARCH_SC8830) += devices-sc8830.o io-sc8830.o

obj-$(CONFIG_PM) += pm.o pm_debug.o standby.o emc_repower.o
obj-$(CONFIG_CPU_FREQ) += cpufreq.o
CFLAGS_emc_repower.o += -fPIC

ifeq ($(CONFIG_PM),y)
	obj-$(CONFIG_ARCH_SC8825) += pm-sc8825.o
	obj-$(CONFIG_ARCH_SC8830) += pm-sc8830.o
endif

obj-$(CONFIG_SPRD_DCDC_DEBUG) += dcdc_debug.o
obj-$(CONFIG_SMP) += platsmp.o headsmp.o
obj-$(CONFIG_SIPC) += sipc.o
obj-$(CONFIG_HOTPLUG_CPU) += hotplug.o
obj-$(CONFIG_SC_VIBRATOR) += vibrator.o

obj-$(CONFIG_BCMDHD)  += dhd_adapter.o

obj-$(CONFIG_MACH_SP8825EA) += board-sp8825ea.o pinmap-sp8825ea.o dma_r1p0.o
obj-$(CONFIG_MACH_SP8830FPGA) += board-sp8830fpga.o pinmap-sp8830fpga.o dma_r4p0.o
