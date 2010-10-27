obj-y += io.o idle.o irq.o timer.o gpio.o\
	mfp_sprd.o vibrator.o  \
	ts0710_mux.o mux_buffer.o \
	dma.o

# Common code for board init
obj-y += common.o

obj-$(CONFIG_MACH_G2PHONE) += board-g2phone.o
obj-$(CONFIG_MACH_OPENPHONE) += board-openphone.o

#obj-$(CONFIG_PM)		+= pm.o standby.o
#obj-$(CONFIG_CPU_FREQ_SC8800G) += cpufreq-sc8800g.o
#obj-$(CONFIG_CPU_IDLE_SC8800G) += cpuidle-sc8800g.o
