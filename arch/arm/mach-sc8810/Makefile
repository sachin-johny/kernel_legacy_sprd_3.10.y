obj-y += io.o irq.o timer.o devices.o arch_asm.o arch_init.o globalregs.o \
         clock_common.o clock_sc8810.o regulator.o regulator-devices.o \
         dma.o reserve.o pinmap.o adi.o gpio_eic.o adc.o efuse.o \
         watchdog.o

ifeq ($(CONFIG_NKERNEL),y)
obj-$(CONFIG_PM) += pm_vlx.o pm_sc8810.o
else
obj-$(CONFIG_PM) += pm.o
endif



obj-$(CONFIG_MACH_SP8810EA) += board-sp8810ea.o
obj-$(CONFIG_MACH_SP8810GA) += board-sp8810ga.o
obj-$(CONFIG_MACH_SP6820GA) += board-sp6820ga.o
obj-$(CONFIG_MACH_AMAZING)  += board-amazing.o
obj-$(CONFIG_SC8810_VIBRATOR) += vibrator.o
