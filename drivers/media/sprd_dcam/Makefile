# SC88XX Platform Support

obj-y += sprd_dcam.o


ifeq ($(CONFIG_ARCH_SCX35),y)
ccflags-y  += -Idrivers/media/sprd_dcam/common -Idrivers/media/sprd_dcam/sc8830

sprd_dcam-objs := common/dcam_v4l2.o  common/gen_scale_coef.o common/sin_cos.o  \
common/csi2/csi_access.o common/csi2/csi_driver.o common/csi2/csi_log.o common/csi2/csi_system.o   \
common/csi2/csi_api.o   sc8830/dcam_drv_sc8830.o
endif

ifeq ($(CONFIG_ARCH_SCX15),y)
ccflags-y  += -Idrivers/media/sprd_dcam/common -Idrivers/media/sprd_dcam/scx15

sprd_dcam-objs := common/dcam_v4l2.o  common/gen_scale_coef.o common/sin_cos.o  \
common/csi2/csi_access.o common/csi2/csi_driver.o common/csi2/csi_log.o common/csi2/csi_system.o   \
common/csi2/csi_api_null.o    scx15/dcam_drv.o 
endif
