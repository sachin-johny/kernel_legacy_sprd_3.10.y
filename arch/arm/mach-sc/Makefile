obj-y += irq.o arch_init.o arch_lock.o \
         reserve.o efuse.o common.o glb.o

obj-$(CONFIG_SC_INTERNAL_ADI) += adi.o adie_irq.o
obj-$(CONFIG_SC_INTERNAL_ADC) += adc.o
obj-$(CONFIG_SC_INTERNAL_WATCHDOG) += watchdog.o

obj-$(CONFIG_ARCH_SC8825) += dcdc_cal.o timer_sc8825.o
obj-$(CONFIG_ARCH_SC8825) += devices-sc8825.o io-sc8825.o  clock-sc8825.o arch_misc.o globalregs.o
obj-$(CONFIG_ARCH_SCX35) += devices-sc8830.o io-sc8830.o  clock-sc8830.o arch_misc.o pin_switch.o
obj-$(CONFIG_ARCH_SCX35) += emc_earlysuspend_8830.o timer_sc8830.o

obj-$(CONFIG_PM) += pm.o
obj-$(CONFIG_FIX_V7TAGRAM_BUG) += fix_v7_tag_ram_bug.o
obj-$(CONFIG_CPU_FREQ) += cpufreq.o
ifeq ($(CONFIG_BUS_MONITOR),y)
obj-$(CONFIG_ARCH_SCX35) += bm_sc8830.o
endif

ifeq ($(CONFIG_CPU_IDLE),y)
	obj-$(CONFIG_ARCH_SCX35) += cpuidle-sc8830.o
endif

ifeq ($(CONFIG_PM),y)
	obj-$(CONFIG_ARCH_SC8825) += pm-sc8825.o standby-sc8825.o pm_debug.o emc_repower.o
	CFLAGS_emc_repower.o += -fPIC
	obj-$(CONFIG_ARCH_SCX35) += pm-sc8830.o standby-sc8830.o pm_debug_sc8830.o
endif
obj-$(CONFIG_SPRD_MEM_POOL) += sprd_mem_pool.o
obj-$(CONFIG_SCXX30_AP_DFS) += ap_dmc_freq.o
obj-$(CONFIG_SPRD_DCDC_DEBUG) += dcdc_debug.o
obj-$(CONFIG_SMP) += platsmp.o headsmp.o
obj-$(CONFIG_SIPC) += sipc.o
obj-$(CONFIG_HOTPLUG_CPU) += hotplug.o
obj-$(CONFIG_SC_VIBRATOR) += vibrator.o
obj-$(CONFIG_ANDROID_RAM_CONSOLE) += ram_console.o
obj-$(CONFIG_SPRD_SIMCTRL) += simctrl.o
ifeq ($(CONFIG_BCMDHD),m)
obj-y += dhd_adapter.o
endif
obj-$(CONFIG_MACH_SP8825EA) += board-sp8825ea.o pinmap-sp8825ea.o dma_r1p0.o
obj-$(CONFIG_MACH_SPX35EA) += board-sp8830ea.o dma_r4p0.o
obj-$(CONFIG_MACH_SPX35EB) += board-sp8830eb.o dma_r4p0.o
obj-$(CONFIG_MACH_SP8835EB) += board-sp8835eb.o dma_r4p0.o
obj-$(CONFIG_MACH_SPX35EC) += board-sp8830ec.o dma_r4p0.o
obj-$(CONFIG_MACH_SP7730EC) += board-sp7730ec.o dma_r4p0.o
obj-$(CONFIG_MACH_SP5735) += board-sp5735.o dma_r4p0.o
obj-$(CONFIG_MACH_SP5735PAD) += board-sp5735pad.o dma_r4p0.o
obj-$(CONFIG_MACH_SPX35FPGA) += board-sp8830fpga.o pinmap-sp8830fpga.o dma_r4p0.o
obj-$(CONFIG_MACH_SP7735EC) += board-sp7735ec.o dma_r4p0.o
obj-$(CONFIG_MACH_SPX15FPGA) += board-sp8830fpga.o pinmap-sp8830fpga.o dma_r4p0.o
obj-$(CONFIG_SPRD_DEBUG) += sprd_debug.o
