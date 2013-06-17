obj-y += irq.o timer.o arch_init.o arch_lock.o \
         reserve.o adi.o adie_irq.o adc.o efuse.o \
         watchdog.o common.o glb.o

obj-$(CONFIG_ARCH_SC8825) += dcdc_cal.o
obj-$(CONFIG_ARCH_SC8825) += devices-sc8825.o io-sc8825.o  clock-sc8825.o arch_misc.o globalregs.o
obj-$(CONFIG_ARCH_SCX35) += devices-sc8830.o io-sc8830.o  clock-sc8830.o arch_misc.o thm.o
obj-$(CONFIG_ARCH_SCX35) += emc_earlysuspend_8830.o

obj-$(CONFIG_PM) += pm.o emc_repower.o
obj-$(CONFIG_FIX_V7TAGRAM_BUG) += fix_v7_tag_ram_bug.o
obj-$(CONFIG_CPU_FREQ) += cpufreq.o
CFLAGS_emc_repower.o += -fPIC

ifeq ($(CONFIG_PM),y)
	obj-$(CONFIG_ARCH_SC8825) += pm-sc8825.o standby-sc8825.o pm_debug.o
	obj-$(CONFIG_ARCH_SCX35) += pm-sc8830.o standby-sc8830.o pm_debug_sc8830.o
endif

obj-$(CONFIG_SPRD_DCDC_DEBUG) += dcdc_debug.o
obj-$(CONFIG_SMP) += platsmp.o headsmp.o
obj-$(CONFIG_SIPC) += sipc.o
obj-$(CONFIG_HOTPLUG_CPU) += hotplug.o
obj-$(CONFIG_SC_VIBRATOR) += vibrator.o
obj-$(CONFIG_ANDROID_RAM_CONSOLE) += ram_console.o
obj-$(CONFIG_SPRD_SIMCTRL) += simctrl.o
obj-$(CONFIG_BCMDHD)  += dhd_adapter.o

obj-$(CONFIG_MACH_SP8825EA) += board-sp8825ea.o pinmap-sp8825ea.o dma_r1p0.o
obj-$(CONFIG_MACH_SPX35EA) += board-sp8830ea.o dma_r4p0.o
obj-$(CONFIG_MACH_SPX35EB) += board-sp8830eb.o dma_r4p0.o
obj-$(CONFIG_MACH_SPX35PAD) += board-sp8830pad.o dma_r4p0.o
obj-$(CONFIG_MACH_SPX35FPGA) += board-sp8830fpga.o pinmap-sp8830fpga.o dma_r4p0.o
obj-$(CONFIG_MACH_SP7735EC) += board-sp7735ec.o dma_r4p0.o
obj-$(CONFIG_MACH_SPX15FPGA) += board-sp8830fpga.o pinmap-sp8830fpga.o dma_r4p0.o
obj-$(CONFIG_SPRD_DEBUG) += sprd_debug.o
