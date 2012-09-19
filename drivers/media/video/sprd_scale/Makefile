ifeq ($(CONFIG_ARCH_SC8810),y)
sprd_scale-objs := sin_cos.o gen_scale_coef.o scale_sc8810.o
endif

ifeq ($(CONFIG_ARCH_SC8825),y)
sprd_scale-objs := tiger/sin_cos.o tiger/gen_scale_coef.o tiger/scale_drv.o tiger/img_scale.o
endif
obj-y += sprd_scale.o

