#ifndef __TROUT_SPI_INTERFACE_H_
#define __TROUT_SPI_INTERFACE_H_


#ifdef TROUT_SDIO_INTERFACE
#include "csl_linux.h"
#include <linux/mmc/core.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>
#endif


#define SPI_SDIO_WRITE_RAM_CMD_WIDTH		8	//CMD(4byte)+DSCR(4byte)

#define TROUT_SYS_REG_BASE		0
#define TROUT_SYS_REG_WIFI_RST (TROUT_SYS_REG_BASE + (0x51 << 2))
#define TROUT_SYS_REG_END	   (0x1FF << 2)	/* byte address */

#define TROUT_SHARE_MEM_BASE_ADDR 0x10000	/* word address */
#define BLOCK_WRITE_CAP		16

#ifdef TROUT_SDIO_INTERFACE

int sdio_trout_init(struct sdio_trout_data *sdiodata);

extern unsigned int host_read_trout_reg(unsigned int reg_addr);
extern unsigned int host_write_trout_reg (unsigned int val, unsigned int reg_addr);
extern unsigned int host_read_trout_ram(void *host_addr, void *trout_addr, unsigned int length);
extern unsigned int host_write_trout_ram (void *trout_addr, void *host_addr, unsigned int length);
extern unsigned int host_block_read_trout_ram(unsigned int host_addr, unsigned int trout_addr[], unsigned int block_len[], unsigned int block_num);
extern unsigned int host_block_write_trout_ram(unsigned int trout_addr[], unsigned int host_addr, unsigned int block_len[], unsigned int block_num);

#else
//int spitrout_init(struct spidev_data *spidata);
//int spitrout_exit(struct spidev_data *spidata);
//void print_spi_info(void);

extern WORD32 host_read_trout_reg(UWORD32 reg_addr);
extern WORD32 host_write_trout_reg(UWORD32 val, UWORD32 reg_addr);
extern WORD32 host_read_trout_ram(void *host_addr, void *trout_addr, UWORD32 length);
extern WORD32 host_write_trout_ram(void *trout_addr, void *host_addr, UWORD32 length);
extern WORD32 host_block_read_trout_ram(UWORD32 host_addr, UWORD32 trout_addr[], UWORD32 block_len[], UWORD32 block_num);
extern WORD32 host_block_write_trout_ram(UWORD32 trout_addr[], UWORD32 host_addr, UWORD32 block_len[], UWORD32 block_num);
#endif

#endif	/* __TROUT_SPI_INTERFACE_H_ */

