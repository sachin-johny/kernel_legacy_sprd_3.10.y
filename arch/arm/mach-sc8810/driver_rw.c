/*
 * sc8810 driver poke Routines
 * * 
 * Copyright (c) 2011 Spreadtrum, Inc.
 *
 * created for sc8810, 2011
 * steve.zhan
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <mach/gpio.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/fs.h>

#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/delay.h>

#include <asm/io.h>
#include <mach/hardware.h>
#include <mach/regs_adi.h>
#include <mach/adi_hal_internal.h>

//#define DEBUG_RW_TEST
#ifdef DEBUG_RW_TEST
#define printk_dbg	printk
#else
#define printk_dbg(...)
#endif
static void driver_read_vir_addr(unsigned int addr)
{
	uint32_t value = 0;
	value = *(volatile uint32_t *)addr;
	printk("driver_read_vir_addr(0x%x) == value(0x%x)\n", addr, value);
}

static void driver_read_phy_addr(unsigned int addr)
{
	uint32_t die_type_a = 0;
	uint32_t value = 0;
	uint32_t *_addr = 0;

	if (addr >= 0x82000000 && addr <= 0x82003000)
		die_type_a = 1;
	else
		die_type_a = 0;

	if (die_type_a) {
		unsigned int vir_addr = __adi_phy_to_virt(addr);
		value = ADI_Analogdie_reg_read((unsigned int)vir_addr);
	} else {
		_addr = ioremap(addr, PAGE_SIZE);
		value = *(volatile uint32_t *)_addr;
		iounmap(_addr);
	}
	printk("driver_read_phy_addr(0x%x) == value(0x%x)\n", addr, value);
}

static void driver_write_vir_addr(unsigned int addr, unsigned int value)
{
	printk("driver_write_vir_addr(0x%x) == value(0x%x)\n", addr, value);
	*(volatile uint32_t *)addr = value;
}

static void driver_write_phy_addr(unsigned int addr, unsigned int value)
{
	uint32_t die_type_a = 0;
	uint32_t *_addr = 0;

	if (addr >= 0x82000000 && addr <= 0x82003000)
		die_type_a = 1;
	else
		die_type_a = 0;

	_addr = ioremap(addr, PAGE_SIZE);

	if (die_type_a) {
		ADI_Analogdie_reg_write((unsigned int)_addr, value);
	} else {
		*(volatile uint32_t *)_addr = value;
	}

	iounmap(_addr);
	printk("driver_write_phy_addr(0x%x) == value(0x%x)\n", addr, value);
}

static int driver_set_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int driver_set_release(struct inode *inode, struct file *filp)
{
	return 0;
}

typedef enum {
	p_ADDR = 0,		//read
	v_ADDR,
	P_ADDR,
	V_ADDR,

	V_FUNC,
} ADDR_MODE;

static ADDR_MODE mode = p_ADDR;

static void set_addr_mode(ADDR_MODE m)
{
	mode = m;
}

static int do_trigger(unsigned int value)
{
	static int index = 0;
	static int _value[2] = { 0 };
	_value[index++] = value;
	if (index == 2 || (index == 1 && (mode == p_ADDR || mode == v_ADDR))) {
		//do actions
		char *s_P = "P_ADDR";
		char *s_V = "V_ADDR";
		char *s_p = "p_ADDR";
		char *s_v = "v_ADDR";
		char *p = 0;

		if (mode == P_ADDR) {
			p = s_P;
			driver_write_phy_addr(_value[0], _value[1]);
			printk_dbg("do_trigger write = 0x%x, 0x%x, mode = %s\n",
				   _value[0], _value[1], p);
		} else if (mode == V_ADDR) {
			p = s_V;
			driver_write_vir_addr(_value[0], _value[1]);
			printk_dbg("do_trigger write = 0x%x, 0x%x, mode = %s\n",
				   _value[0], _value[1], p);
		} else if (mode == p_ADDR) {
			p = s_p;
			driver_read_phy_addr(_value[0]);
			printk_dbg("do_trigger read = 0x%x mode = %s\n",
				   _value[0], p);
		} else if (mode == v_ADDR) {
			p = s_v;
			driver_read_vir_addr(_value[0]);
			printk_dbg("do_trigger read = 0x%x mode = %s\n",
				   _value[0], p);
		}

		index = 0;
	}
	return 0;
}

static int string_strtok(char **start)
{
	if (**start == 0 || **start == '\0')
		return -1;

	while (**start && (**start != ','))
		(*start)++;
	*(*start)++ = 0;
	return 0;
}

ssize_t driver_set_write(struct file * file, const char __user * string,
			 size_t length, loff_t * offset)
{
	char buf[256];
	char *p = buf;
	char *p1 = buf;

	if (!string || length > 256 || length <= 0)
		goto ErrExit;

	if (length > 256)
		length = 256;

	copy_from_user(buf, string, length);
	buf[length] = 0;
	printk_dbg("%s\n", buf);

	while (p - buf < length) {
		if (p[0] == '(') {
			if (!p[1] || !p[2] || (p[2] != ')'))
				goto ErrExit;
			else {
				switch (p[1]) {
				case 'p':
					set_addr_mode(p_ADDR);
					break;
				case 'v':
					set_addr_mode(v_ADDR);
					break;
				case 'P':
					set_addr_mode(P_ADDR);
					break;
				case 'V':
					set_addr_mode(V_ADDR);
					break;
				default:
					goto ErrExit;
					break;
				}

				while (*p && *(p - 1) != ',')
					p++;

			}

			printk_dbg("sub_string:%s\n", p);

		}

		p1 = p;
		if (-1 == string_strtok(&p))
			goto ErrExit;
		{
			int value = 0;
			value = simple_strtoul(p1, NULL, 0);
			do_trigger(value);
		}

	}

	return length;

ErrExit:
	return -1;
}

static const struct file_operations driver_poke_fops = {
	.open = driver_set_open,
	.write = driver_set_write,
	.release = driver_set_release,
};

static struct miscdevice driver_poke_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "driver_poke",
	.fops = &driver_poke_fops
};

static int driver_poke_init(void)
{
	return misc_register(&driver_poke_dev);
}

static void driver_poke_exit(void)
{
	misc_deregister(&driver_poke_dev);
}

late_initcall(driver_poke_init);
module_exit(driver_poke_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Steve Zhan <steve.zhan@spreadtrum.com>");
MODULE_DESCRIPTION("Driver for read and write to kernel");
