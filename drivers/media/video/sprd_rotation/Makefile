# SC88XX Platform Support
ifeq ($(CONFIG_ARCH_SC8810),y)
sprd_rotation-objs := rotation_drv.o
obj-y += sprd_rotation.o
endif

ifeq ($(CONFIG_ARCH_SC8825),y)
sprd_rotation-objs := rot_drv.o
obj-y += sprd_rotation.o
endif

ifeq ($(CONFIG_ARCH_SC8830),y)
sprd_rotation-objs := sc8830/rot_drv.o sc8830/img_rot.o
obj-y += sprd_rotation.o
endif

