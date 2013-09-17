# SC88XX Platform Support

obj-$(CONFIG_SND_SC88XX_SOC_SPRDPHONE) += sprdphone.o vbc-codec.o speaker-pa.o sc88xx-vbc.o sc88xx-pcm.o aud_enha_exp.o aud_enha_config.o audio_dev_hal.o filter_calc.o vb_phy_v3.o dol_phy_v0.o

iis-for-trout-fm-objs			:= i2s.o
obj-$(CONFIG_IIS_FOR_TROUT_FM)		+= iis-for-trout-fm.o

iis-for-ext-fm-objs			:= i2s-fm.o
obj-$(CONFIG_SND_SPRD_SOC_IIS_FM)	+= iis-for-ext-fm.o

obj-$(CONFIG_SND_SPRD_SOC_I2S)		+= dai/i2s/
obj-$(CONFIG_SND_SOC_NULL_CODEC)	+= codec/null-codec/
