/*
 * sc8810 Power Management Routines
 *
 * Copyright (c) 2011 Spreadtrum, Inc.
 *
 * created for sc8810, 2011
 * robot.lian
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License.
 */
void __attribute__ ((naked)) hold(void)
{
	while (1) ;		//used for trace32 debug
}

typedef unsigned long u32;

typedef struct {
	volatile u32 addr;
	u32 val;
} pm_reg_s;

#if defined(__KERNEL__)
u32 __p2v(u32 addr)
{
	if (0) {
	} else if (addr >= 0x20900000 && addr < 0x20901000) {
		addr = 0xE000A000 + (addr & 0xfff);
	} else if (addr >= 0x80003000 && addr < 0x80004000) {
		addr = 0xE0000000 + (addr & 0xfff);
	} else if (addr >= 0x81000000 && addr < 0x81001000) {
		addr = 0xE001a000 + (addr & 0xfff);
	} else if (addr >= 0x82000000 && addr < 0x82001000) {
		addr = 0xE0037000 + (addr & 0xfff);
	} else if (addr >= 0x87000000 && addr < 0x87001000) {
		addr = 0xE002c000 + (addr & 0xfff);
	} else if (addr >= 0x87003000 && addr < 0x87004000) {
		addr = 0xE002d000 + (addr & 0xfff);
	} else if (addr >= 0x8b000000 && addr < 0x8b001000) {
		addr = 0xE0033000 + (addr & 0xfff);
	} else {
		hold();
	}
	return addr;
}

#define _reg(_a_)	(*(volatile u32 * const)(__p2v(_a_)))

#else //no mmu while minicode testing
#define _reg(_a_)	(*(volatile u32 * const)(_a_))
#endif

inline int io_read(u32 addr)
{
	return _reg(addr);
}

inline int io_write(u32 addr, u32 val)
{
	_reg(addr) = val;
	return 0;
}

inline u32 adi_read(u32 addr)
{
	u32 val;
	_reg(0x82000024) = addr;
	while ((val = _reg(0x82000028)) & (1 << 31 /*CMD_BUSY */ )) ;	//why not check return adireg address
	return val & 0x0000ffff;
}

inline int adi_write(u32 addr, u32 val)
{
	while (_reg(0x8200002c) & (1 << 11 /*FIFO_FULL */ )) ;
	_reg(addr) = val;
	return 0;
}

inline int or_io_write(u32 addr, u32 val)
{
	_reg(addr) |= val;
	return 0;
}

inline int or_adi_write(u32 addr, u32 val)
{
	adi_write(addr, adi_read(addr) | val);
	return 0;
}

inline int andnot_io_write(u32 addr, u32 val)
{
	_reg(addr) &= ~val;
	return 0;
}

inline int andnot_adi_write(u32 addr, u32 val)
{
	adi_write(addr, adi_read(addr) & ~val);
	return 0;
}

////////////////////////////////////////////////////////////
static u32 global_saved[] = {
	0x20900200,		//AHB_CTRL0
	0x8b000008,		//GR_GEN0
	0x8b000018,		//GR_GEN1
	0x8b000044,		//GR_BUSCLK
	0x8b000074,		//GR_CLK_EN
	0x80003008,		//INT_IRQ_ENABLE
};

//BUGBUG: maybe move iram later, be sure there nobuffer nocache (l2)
static u32 temp_area[sizeof(global_saved) / sizeof(u32)];

int save_global_regs(u32 safe_area[])
{
	register int i = 0;
	for (i = 0; i < sizeof(global_saved) / sizeof(u32); i++) {
		safe_area[i] = io_read(global_saved[i]);
	}
	return 0;
}

int restore_global_regs(u32 safe_area[])
{
	register int i = 0;
	for (i = 0; i < sizeof(global_saved) / sizeof(u32); i++) {
		io_write(global_saved[i], safe_area[i]);
	}
	return 0;
}

int deepsleep_prepare(void)
{
	save_global_regs(temp_area);

	io_write(0x8000300c, 0x00000028);	//prevent uart1, time0 int while sleep
	io_write(0x20900214, 0x00000002);	//now, only arm core and sys sleep

	return 0;
}

int deepsleep_finish(void)
{
	restore_global_regs(temp_area);
	//BUGBUG: wait a moment for sys and peri ready
	return 0;
}
