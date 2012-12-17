obj-y += io.o irq.o timer.o arch_init.o globalregs.o arch_lock.o \
         dma.o reserve.o pinmap.o adi.o adie_irq.o gpio_eic.o adc.o efuse.o \
         watchdog.o common.o

obj-y += glb.o
obj-y += clock.o

obj-y += dcdc_cal.o
obj-$(CONFIG_SPRD_DCDC_DEBUG) += dcdc_debug.o

obj-$(CONFIG_SMP)		+= platsmp.o headsmp.o

#obj-$(CONFIG_PM)		+= pm.o pm_debug.o
#obj-$(CONFIG_PM)		+= pm_sc8830.o standby.o emc_repower.o
CFLAGS_emc_repower.o += -fPIC

obj-$(CONFIG_SIPC)		+= sipc.o

obj-$(CONFIG_HOTPLUG_CPU)	+= hotplug.o

obj-$(CONFIG_SC_VIBRATOR) += vibrator.o

ifeq ($(CONFIG_RTL8723AS),m)
	obj-y  += rtl8723_adapter.o
endif

obj-$(CONFIG_ANDROID_RAM_CONSOLE) += ram_console.o

obj-$(CONFIG_ARCH_SC8825) += devices-sc8825.o
obj-$(CONFIG_MACH_SP8825EA) += board-sp8825ea.o

obj-$(CONFIG_ARCH_SC8830) += devices-sc8830.o
obj-$(CONFIG_MACH_SP8830EA) += board-sp8830ea.o
obj-$(CONFIG_MACH_SP8830FPGA) += board-sp8830fpga.o

