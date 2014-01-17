#ifndef __SIM_TROUT_TX_FRAME_H
#define __SIM_TROUT_TX_FRAME_H
#include <linux/types.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include "dv_ioctl.h"

struct tx_frame {
	int frame_len;
	struct list_head node;
	struct tx_info info;
	u8 data[0];
};

extern struct tx_frame *alloc_tx_frame(int frame_len);

static inline void free_tx_frame(struct tx_frame *frame)
{
	kfree(frame);
}

extern void add_tx_frame(struct tx_frame *frame);

extern struct tx_frame *fetch_tx_frame(void);

extern void del_tx_frame_tail(void);

extern void cleanup_send_list(void);

#endif
