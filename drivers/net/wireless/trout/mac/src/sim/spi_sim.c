#include <linux/module.h>
#include <linux/kallsyms.h>
#include <linux/ctype.h>
#include "spi_sim.h"
#include "reg_space.h"
#include "share_mem.h"
#include "virt_irq.h"
#include "trout.h"
#include "trout_reg_def.h"

//#define SPI_SIM_DBG

#ifdef SPI_SIM_DBG
static inline
void invalid_reg_addr_report(UWORD32 addr)
{
	DV_DBG("not the trout register, addr:%#.08x\n", addr);
}

static inline 
void trout_reg_debug(UWORD32 reg_addr, UWORD32 val, const char *action)
{
	unsigned long stack;

	stack = (u32)&stack;

	if (reg_addr == rMAC_PA_VER
		|| reg_addr == rMAC_PHY_REG_ACCESS_CON
		|| reg_addr == rMAC_PHY_REG_RW_DATA) {
		return;
	}

	DV_DBG("de %s register: %#.08x, value: %#.08x", action, reg_addr, val);
	print_symbol(", at %s", (unsigned long)__builtin_return_address(0));
	print_symbol(" <= %s", (unsigned long)__builtin_return_address(1));
	print_symbol(" <= %s\n", (unsigned long)__builtin_return_address(2));
}

static inline void invalid_addr_range_report(const void *trout_addr,
		UWORD32 length)
{
	DV_DBG("invalid trout address range, tour addr:%p, length: %u\n",
			trout_addr, length);
}

static inline void hexdump_line_head(u32 base_addr)
{
	printk(KERN_DEBUG "[%.04x]:  ", base_addr);
}

static inline void show_content(void *data, int size, int offset, int i)
{
	if (offset<=i && i<(offset+size)) {
		printk("%.02x", ((u8*)data)[i-offset]);
	} else {
		printk("  ");
	}
}

static inline void hexdump_content(void *data, int size, int offset)
{
	int i;

	for (i=0; i<16; ++i) {
		show_content(data, size, offset, i);
		if (((i+1) & 0x07) == 0) {
			printk("   ");
		} else if (((i+1) & 0x03) == 0) {
			printk("  ");
		} else {
			printk(" ");
		}
	}
}

static inline void show_ascii(void *data, int size, int offset, int i)
{
	u8 ch;
	if (offset<=i && i<(offset+size)) {
		ch = ((u8*)data)[i-offset];

		if (isascii(ch) && isprint(ch)) {
			printk("%c", ch);
			return;
		}
	}

	printk(".");
}

static inline void hexdump_ascii(void *data, int size, int offset)
{
	int i;

	printk("|");
	for (i=0; i<16; ++i) {
		show_ascii(data, size, offset, i);
	}
	printk("|");
}

static int hexdump_line(void *data, int size, u32 addr)
{
	int offset = addr & 0x0F;
	size = min(16 - offset, size);

	hexdump_line_head(addr - offset);

	hexdump_content(data, size, offset);

	hexdump_ascii(data, size, offset);

	printk("\n");

	return size;
}

void hexdump(void *data, int size, u32 addr)
{
	int ret;

	BUG_ON(!data || size < 0);

	while (size) {
		ret = hexdump_line(data, size, addr);
		size -= ret;
		addr += ret;
		data += ret;
	}
}

static void hexdump_ram(void *addr, UWORD32 size)
{
	const UWORD32 MAX_DUMP_SIZE = 256;

	DV_DBG("hexdump: addr %p, size %u", addr, size);

	size = (size < MAX_DUMP_SIZE) ? size : MAX_DUMP_SIZE;

	hexdump(addr, size, 0);
}

static inline void xfer_debug(void *trout_addr, void *addr,
		UWORD32 length,const char *xfer)
{
	DV_DBG("host %s trout ram, trout addr %p\n", xfer, trout_addr);
	hexdump_ram(addr, length);
}
#else
static inline void invalid_reg_addr_report(UWORD32 addr){}
static inline void invalid_addr_range_report(const void *trout_addr,
		UWORD32 length){}
static inline
void trout_reg_debug(UWORD32 reg_addr, UWORD32 val, const char *action){}

static inline void xfer_debug(void *trout_addr, void *addr,
		UWORD32 length,const char *xfer){}
#endif // SPI_SIM_DBG


static inline int is_valid_reg_addr(UWORD32 reg_addr)
{
	return (PA_BASE <= reg_addr && reg_addr <= PA_BASE + 0x81C)
		|| (rCOMM_INT_CLEAR <= reg_addr && reg_addr <= rCOMM_INT_STAT);
}

UWORD32 host_read_trout_reg(UWORD32 reg_addr)
{
	UWORD32 value;

	if (!is_valid_reg_addr(reg_addr)) {
		invalid_reg_addr_report(reg_addr);
		return 0;
	}

	value = de_read_reg((const volatile void *)reg_addr);
	trout_reg_debug(reg_addr, value, "read");

	return value;
}

EXPORT_SYMBOL(host_read_trout_reg);

UWORD32 host_write_trout_reg(UWORD32 val, UWORD32 reg_addr)
{
	if (!is_valid_reg_addr(reg_addr)) {
		invalid_reg_addr_report(reg_addr);
		return 0;
	}

	trout_reg_debug(reg_addr, val, "write");
	de_write_reg(val, (volatile void *)reg_addr);
	return 0;
}

EXPORT_SYMBOL(host_write_trout_reg);

static void xfer_end(UWORD32 length)
{
	// ATTENTION: for now there is no spi DMA interrupt
	/*
	if (length > 64) {
		raise_virt_irq(SPI_DMA_INTR);
	}
	*/
}

UWORD32 host_read_trout_ram(void *host_addr, void *trout_addr, UWORD32 length)
{
	void *addr;

	if (is_valid_trout_addr_range(trout_addr, length)) {
		addr = __from_trout_addr(trout_addr);
		memcpy(host_addr, addr, length);
		xfer_debug(trout_addr, addr, length, "read");
		xfer_end(length);
		return 0;
	} else {
		invalid_addr_range_report(trout_addr, length);
		BUG();
		return -1;
	}
}

EXPORT_SYMBOL(host_read_trout_ram);

UWORD32 host_write_trout_ram(void *trout_addr, void *host_addr, UWORD32 length)
{
	void *addr;

	if (is_valid_trout_addr_range(trout_addr, length)) {
		addr = __from_trout_addr(trout_addr);
		memcpy(addr, host_addr, length);
		xfer_debug(trout_addr, addr, length, "write");
		xfer_end(length);
		return 0;
	} else {
		invalid_addr_range_report(trout_addr, length);
		BUG();
		return -1;
	}
}

EXPORT_SYMBOL(host_write_trout_ram);

