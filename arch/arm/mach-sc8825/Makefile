obj-y += io.o irq.o timer.o devices.o arch_asm.o arch_init.o globalregs.o \
         pm_debug.o \
         dma.o reserve.o pinmap.o adi.o adie_irq.o gpio_eic.o adc.o efuse.o \
         watchdog.o

obj-y += glb.o
obj-y += clock.o

ifeq ($(CONFIG_NKERNEL),)
obj-$(CONFIG_SMP)		+= platsmp.o headsmp.o
endif

obj-$(CONFIG_HOTPLUG_CPU)	+= hotplug.o
obj-$(CONFIG_MACH_SP8825_FPGA) += board_sp8825_fpga.o
