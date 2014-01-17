#ifndef _TROUT_H
#define _TROUT_H
#include "trout_struct.h"
#include "virt_irq.h"

#define TROUT_INTR_VEC 12
#define SPI_DMA_INTR 13

/*
 * all the bit in the interrupt status register left moved
 * 2 bit just for hardware implementation reason
 */
#define RX_COMPLETE (1U << (0 + 2))
#define TX_COMPLETE (1U << (1 + 2))
#define TBTT (1U << (2 + 2))
#define HIP_RX_COMPLETE (1U << (10 + 2))

extern struct trout *get_trout(void);

static inline void raise_trout_irq(void)
{
	raise_virt_irq(TROUT_INTR_VEC);
}

static inline void flush_pending_trout_irq(void)
{
	flush_pending_irq(TROUT_INTR_VEC);
}

extern int trout_init(void);

extern void trout_cleanup(void);

/*
 * if the trout received or transmited frame complete,
 * and wait for raising interrupt ?
 */
extern int is_trout_frame_pending(void);

extern void tbtt_restart(void);

extern void tbtt_stop(void);

#endif
