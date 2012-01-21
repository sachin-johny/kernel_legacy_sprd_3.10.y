/*
  *     Deep sleep testing mode.
  *
  *     Download small piece of code into DSP and make it sleep for ever.
  *
  */


#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/initrd.h>
#include <linux/android_pmem.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>

#include <mach/board.h>
#include <mach/hardware.h>

#include <asm/io.h>
#include <asm/delay.h>

#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <mach/gpio.h>
#include <mach/adi_hal_internal.h>
#include <mach/regs_ana.h>
#include <mach/regs_cpc.h>

#include <linux/clk.h>
#include <mach/clock_common.h>
#include <mach/clock_sc8800g.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/dcam_sensor.h>
#include <mach/common.h>
#ifdef CONFIG_SENSORS_MMC31XX
#include <linux/mmc31xx.h>
#endif
#ifdef CONFIG_SENSORS_MXC622X
#include <linux/mxc622x.h>
#endif


//i2c pad:  the high two bit of the addr is the pad control bit
static struct i2c_board_info __initdata i2c_boardinfo[] = {
#ifdef CONFIG_SENSORS_MMC31XX
	{I2C_BOARD_INFO(MMC31XX_I2C_NAME, MMC31XX_I2C_ADDR),},
#endif
#ifdef CONFIG_SENSORS_MXC622X
	{I2C_BOARD_INFO(MXC622X_I2C_NAME,MXC622X_I2C_ADDR),},
#endif
};

//i2c pad:  the high two bit of the addr is the pad control bit
static struct i2c_board_info __initdata i2c_boardinfo1[] = {
	{I2C_BOARD_INFO(SENSOR_MAIN_I2C_NAME,SENSOR_MAIN_I2C_ADDR),},
	{I2C_BOARD_INFO(SENSOR_SUB_I2C_NAME,SENSOR_SUB_I2C_ADDR),},
};

static struct i2c_board_info __initdata i2c_boardinfo2[] = {
	{I2C_BOARD_INFO("pixcir_ts", 0x5C),},
};

static void sprd8810_i2c2pin_config(void)
{	
	unsigned int reg;
	reg = __raw_readl(SPRD_GREG_BASE+0x0028);
	reg |= 0x0100;
	__raw_writel(reg, (SPRD_GREG_BASE+0x0028));
}

int __init sprd_i2c_init(void)
{
	sprd8810_i2c2pin_config();
	sprd_register_i2c_bus(0, i2c_boardinfo,
			ARRAY_SIZE(i2c_boardinfo));
	sprd_register_i2c_bus(1, i2c_boardinfo1,
			ARRAY_SIZE(i2c_boardinfo1));
	sprd_register_i2c_bus(2, i2c_boardinfo2,
			ARRAY_SIZE(i2c_boardinfo2));
	sprd_register_i2c_bus(3, NULL, 0);
	return 0;
}
