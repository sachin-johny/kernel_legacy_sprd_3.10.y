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
obj-$(CONFIG_PM)		+= pm_sc8825.o standby.o pm_debug.o emc_repower.o 
CFLAGS_emc_repower.o += -fPIC


obj-$(CONFIG_SIPC)		+= sipc.o

obj-$(CONFIG_HOTPLUG_CPU)	+= hotplug.o
obj-$(CONFIG_MACH_SP8825_FPGA) += board_sp8825_fpga.o
obj-$(CONFIG_MACH_SP8825_OPENPHONE) += board_sp8825_openphone.o board_sp8825_openphone_wifi.o
obj-$(CONFIG_MACH_SP8825EA) += board_sp8825ea.o board_sp8825ea_wifi.o
obj-$(CONFIG_MACH_SP8825EB) += board_sp8825eb.o
obj-$(CONFIG_MACH_SP6825GA) += board_sp6825ga.o
obj-$(CONFIG_MACH_GARDA) += board_garda.o board_garda_wifi.o
obj-$(CONFIG_SC8825_VIBRATOR) += vibrator.o
ifeq ($(CONFIG_RTL8723AS),m)
	obj-y  += rtl8723_adapter.o
endif
obj-$(CONFIG_ANDROID_RAM_CONSOLE) += ram_console.o
