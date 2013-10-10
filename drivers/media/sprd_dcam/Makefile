# SC88XX Platform Support

obj-y += sprd_dcam.o

ifeq ($(CONFIG_ARCH_SC8825),y)
sprd_dcam-objs := sc8825/dcam_drv_sc8825.o sc8825/gen_scale_coef.o sc8825/sin_cos.o sc8825/dcam_v4l2.o \
sc8825/csi2/csi_access.o sc8825/csi2/csi_api.o sc8825/csi2/csi_driver.o sc8825/csi2/csi_log.o sc8825/csi2/csi_system.o
endif

ifeq ($(CONFIG_ARCH_SCX35),y)
sprd_dcam-objs := sc8830/dcam_drv_sc8830.o sc8830/gen_scale_coef.o sc8830/sin_cos.o sc8830/dcam_v4l2.o \
sc8830/csi2/csi_access.o sc8830/csi2/csi_api.o sc8830/csi2/csi_driver.o sc8830/csi2/csi_log.o sc8830/csi2/csi_system.o
endif

