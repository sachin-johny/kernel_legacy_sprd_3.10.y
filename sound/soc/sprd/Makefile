# SC88XX Platform Support
obj-$(CONFIG_SND_SC88XX_SOC_SPRDPHONE) += sprdphone.o
obj-$(CONFIG_SND_SC88XX_SOC)           += sc88xx-pcm.o
obj-$(CONFIG_SND_SC88XX_SOC_VBC)       += sc88xx-vbc.o
obj-$(CONFIG_SND_SOC_VBC)              += vbc-codec.o vbc-codec-eq.o speaker-pa.o audio_dev_hal.o filter_calc.o vb_phy_v3.o dol_phy_v0.o

# SPRD Platform Support
snd-soc-sc881x-objs         		:= sc881x.o
obj-$(CONFIG_SND_SPRD_SOC_SC881X)    	+= snd-soc-sc881x.o
snd-soc-kylew-objs         		:= kylew.o
obj-$(CONFIG_SND_SPRD_SOC_KYLEW)    	+= snd-soc-kylew.o
snd-soc-sc882x-objs         		:= sc882x.o
obj-$(CONFIG_SND_SPRD_SOC_SC882X)    	+= snd-soc-sc882x.o
obj-$(CONFIG_SND_SPEAKER_PA)         	+= speaker-pa.o

obj-$(CONFIG_SND_SPRD_SOC_VBC)	     	+= dai/vbc/
obj-$(CONFIG_SND_SPRD_SOC_VAUDIO)     	+= dai/vaudio/
obj-$(CONFIG_SND_SPRD_SOC_I2S)		+= dai/i2s/
obj-$(CONFIG_SND_SOC_DOLPHIN)		+= codec/dolphin/
obj-$(CONFIG_SND_SOC_SPRD_CODEC)	+= codec/sprd/
