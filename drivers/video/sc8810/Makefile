# core framebuffer
#
obj-y:= sprdfb_main.o lcdc.o

# lcd driver
#

#obj-y+= sc8800g_lcd_s6d0139.o
obj-$(CONFIG_MACH_SP8810) += lcd_hx8369.o lcd_dummy.o
obj-$(CONFIG_MACH_SP8810G_BRCM) += lcd_hx8357.o lcd_dummy.o
obj-$(CONFIG_MACH_SP6820A) += lcd_hx8369.o lcd_dummy.o
obj-$(CONFIG_MACH_SC8810OPENPHONE)+=lcd_hx8357.o lcd_dummy.o


