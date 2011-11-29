# SC88XX Platform Support

#EXTRA_CFLAGS := -lgcc
ifdef CONFIG_ARCH_SC8810
sprd_scale_sc8800g2-objs := sin_cos.o gen_scale_coef.o scale_sc8810.o
else
sprd_scale_sc8800g2-objs := sin_cos.o gen_scale_coef.o scale_sc8800g2.o
endif
obj-y += sprd_scale_sc8800g2.o
#obj-m += sprd_scale_sc8800g2.o
