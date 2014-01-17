#ifndef _SPI_SIM_H
#define _SPI_SIM_H
#include "typedef.h"
#include "debug.h"

extern UWORD32 host_read_trout_reg(UWORD32 reg_addr);
extern UWORD32 host_write_trout_reg(UWORD32 val, UWORD32 reg_addr);

extern UWORD32 host_read_trout_ram(void *host_addr, void *trout_addr,
		UWORD32 length);
extern UWORD32 host_write_trout_ram(void *trout_addr, void *host_addr,
		UWORD32 length);
#endif

