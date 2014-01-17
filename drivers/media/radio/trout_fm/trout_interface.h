#ifndef TROUT_INTERFACE_H__
#define TROUT_INTERFACE_H__

typedef struct
{
    char *name;
    unsigned int (*init)(void);
    unsigned int (*read_reg)(u32 addr, u32 *val);
    unsigned int (*write_reg)(u32 addr,  u32 val);
    unsigned int (*exit)(void);
}trout_interface_t;

extern int trout_sdio_init(trout_interface_t **p);
extern int trout_spi_init(trout_interface_t **p);
extern int trout_shared_init(trout_interface_t **p);

#endif