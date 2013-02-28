obj-y += io.o irq.o timer.o devices.o arch_asm.o arch_init.o globalregs.o \
         pm_debug.o arch_lock.o \
         dma.o reserve.o pinmap.o adi.o adie_irq.o gpio_eic.o adc.o efuse.o \
         watchdog.o common.o


obj-y += glb.o
obj-$(CONFIG_NKERNEL) += clock_vlx.o
obj-y += clock.o

obj-y += dcdc_cal.o
obj-$(CONFIG_SPRD_DCDC_DEBUG) += dcdc_debug.o

ifeq ($(CONFIG_NKERNEL),)
obj-$(CONFIG_SMP)		+= platsmp.o headsmp.o
endif

ifeq ($(CONFIG_NKERNEL),y)
obj-$(CONFIG_PM)		+= pm_vlx.o
else
obj-$(CONFIG_PM)		+= pm.o 
endif
obj-$(CONFIG_PM)		+= pm_sc8825.o standby.o pm_debug.o emc_repower.o emc_earlysuspend.o 
CFLAGS_emc_repower.o += -fPIC

obj-$(CONFIG_CPU_FREQ) += cpufreq-sc8825.o cpufreq_user.o

obj-$(CONFIG_SIPC)		+= sipc.o

obj-$(CONFIG_HOTPLUG_CPU)	+= hotplug.o

sprdboard-$(CONFIG_MACH_SP8825_FPGA) += board_sp8825_fpga/
sprdboard-$(CONFIG_MACH_SP8825_OPENPHONE) += board_sp8825_openphone/
sprdboard-$(CONFIG_MACH_SP8825EA) += board_sp8825ea/
sprdboard-$(CONFIG_MACH_SP8825EB) += board_sp8825eb/
sprdboard-$(CONFIG_MACH_SP6825GA) += board_sp6825ga/
sprdboard-$(CONFIG_MACH_SP6825GB) += board_sp6825gb/
sprdboard-$(CONFIG_MACH_GARDA) += board_garda/
sprdboard-$(CONFIG_MACH_SP8825GA_OPENPHONE) += board_sp8825ga_openphone/

obj-y += $(sprdboard-y)

obj-$(CONFIG_MACH_SP6825GB) += ar6302g_adapter.o
obj-$(CONFIG_SC8825_VIBRATOR) += vibrator.o
ifeq ($(CONFIG_BCMDHD),m)
obj-y  += dhd_adapter.o
endif
obj-$(CONFIG_BCMDHD89)  += dhd_adapter.o
ifeq ($(CONFIG_RTL8723AS),m)
	obj-y  += rtl8723_adapter.o
endif
obj-$(CONFIG_ANDROID_RAM_CONSOLE) += ram_console.o
obj-$(CONFIG_SPRD_MEM_POOL)    += sprd_mem_pool.o
obj-$(CONFIG_SPRD_SC8825_MEMFREQ_ONDEMAND)    += memfreq_ondemand.o

sprdboarddirs := $(patsubst %,arch/arm/mach-sc8825/%,$(sprdboard-y))
KBUILD_CPPFLAGS += $(patsubst %,-I%,$(sprdboarddirs))
