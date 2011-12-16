/*
 * linux/arch/arm/plat-sc88xx/i2c.c
 *
 *
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>

#include <mach/hardware.h>
#include <mach/board.h>
#include <mach/irqs.h>

#define SPRD_I2C_SIZE		0x1000
#define SPRD_I2C_IRQ0		11
#define SPRD_I2C_IRQ1		11
#define SPRD_I2C_IRQ2		14
#define SPRD_I2C_IRQ3		14

static const char name[] = "sc8810-i2c";

#define I2C_RESOURCE_BUILDER(base, irq)			\
	{						\
		.start	= (base),			\
		.end	= (base) + SPRD_I2C_SIZE - 1,	\
		.flags	= IORESOURCE_MEM,		\
	},						\
	{						\
		.start	= (irq),			\
		.end	= (irq),			\
		.flags	= IORESOURCE_IRQ,		\
	},

static struct resource i2c_resources[][2] = {
	{ I2C_RESOURCE_BUILDER(SPRD_I2C0_BASE, SPRD_I2C_IRQ0) },
	{ I2C_RESOURCE_BUILDER(SPRD_I2C1_BASE, SPRD_I2C_IRQ1) },
	{ I2C_RESOURCE_BUILDER(SPRD_I2C2_BASE, SPRD_I2C_IRQ2) },
	{ I2C_RESOURCE_BUILDER(SPRD_I2C3_BASE, SPRD_I2C_IRQ3) },
};

#define I2C_DEV_BUILDER(bus_id, res)		\
	{						\
		.id	= (bus_id),			\
		.name	= name,				\
		.num_resources	= ARRAY_SIZE(res),	\
		.resource	= (res),		\
	}

static struct platform_device sprd_i2c_devices[] = {
	I2C_DEV_BUILDER(0, i2c_resources[0]),
	I2C_DEV_BUILDER(1, i2c_resources[1]),
	I2C_DEV_BUILDER(2, i2c_resources[2]),
	I2C_DEV_BUILDER(3, i2c_resources[3]),
};

static int __init sprd_i2c_add_bus(int bus_id)
{
	struct platform_device *pdev;

	pdev = &sprd_i2c_devices[bus_id];

	return platform_device_register(pdev);
}

/**
 * SPRD_register_i2c_bus - register I2C bus with device descriptors
 * @bus_id: bus id counting from number 0
 * @info: pointer into I2C device descriptor table or NULL
 * @len: number of descriptors in the table
 *
 * Returns 0 on success or an error code.
 */
int __init sprd_register_i2c_bus(int bus_id,
			  struct i2c_board_info const *info,
			  unsigned len)
{
	int err;

	BUG_ON(bus_id < 0 || bus_id > 3);

	if (info) {
		err = i2c_register_board_info(bus_id, info, len);
		if (err)
			return err;
	}

	return sprd_i2c_add_bus(bus_id);
}
