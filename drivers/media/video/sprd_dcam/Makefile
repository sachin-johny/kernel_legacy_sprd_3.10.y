# SC88XX Platform Support

#obj-$(CONFIG_SND_SC88XX_SOC_SPRDPHONE) += sprdphone.o vbc-codec.o sc88xx-vbc.o sc88xx-pcm.o
#obj-y += sensor_2655.o sensor.o dcam_drv_sc8800s.o dcam.o
#obj-m += sensor_2655.o sensor.o dcam_drv_sc8800s.o dcam.o dcam_v4l2.o
#sprd_dcam-objs := sensor_2655.o sensor.o dcam_drv_sc8800s.o dcam.o dcam_v4l2.o
ifdef CONFIG_ARCH_SC8810
sprd_dcam_sc8800g2-objs := sc8810/sensor_drv.o sensor_gc0309.o sensor_gt2005.o sensor_ov2655.o sensor_ov7675.o sensor_ov7690.o sc8810/dcam_service_sc8810.o sc8810/dcam_drv_sc8810.o sc8810/dcam_v4l2.o sc8810/isp_control.o  sensor_ov2640.o sensor_ov5640.o sensor_ov7660.o sensor_nmi601.o sc8810/dc_cfg.o sc8810/dc_product_cfg.o
else
sprd_dcam_sc8800g2-objs := sensor_cfg.o sensor_drv.o sensor_gc0309.o sensor_gt2005_new.o sensor_ov2655.o sensor_ov7675.o sensor_ov7690.o dcam_sc8800g2.o dcam_drv_sc8800g2.o dcam_v4l2_sc8800g2.o
endif
#obj-$(CONFIG_NMI_ATV) += sensor_nmi601.o
#obj-m += sprd_dcam_sc8800g2.o
obj-y += sprd_dcam_sc8800g2.o
