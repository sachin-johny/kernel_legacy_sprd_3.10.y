obj-y += io.o idle.o irq.o timer.o gpio.o gpio_phy.o\
	mfp_sprd.o \
	dma.o ldo.o adi_drv.o \
	rfkill_dummy.o gpio_set.o adc.o chg_phy.o eic.o arch_init.o arch_asm.o
obj-m +=driver_rw.o
obj-$(CONFIG_TS0710_MUX_UART)+= ts0710_mux.o mux_buffer.o 
# Common code for board init
obj-y += common.o clock_common.o clock_sc8800g.o power_domain.o clock_test.o i2c.o \
		 pm_deepsleep_prepare.o driver_test.o spinlock_hw_vlx.o   

obj-$(CONFIG_MACH_G2PHONE) += board-g2phone.o
obj-$(CONFIG_MACH_SC8810OPENPHONE) += board-openphone.o
obj-$(CONFIG_MACH_SP8810) += board-sp8810.o

obj-$(CONFIG_PM)		+= pm.o standby.o idle_v7.o pm_devices.o

obj-$(CONFIG_CPU_FREQ_SC8800G) += cpufreq-sc8800g.o
obj-$(CONFIG_CPU_IDLE_SC8800G) += cpuidle-sc8800g.o

obj-$(CONFIG_SC8800G_VIBRATOR) += vibrator.o
obj-$(CONFIG_CACHE_L3X0)	+= cache-l3x0.o

