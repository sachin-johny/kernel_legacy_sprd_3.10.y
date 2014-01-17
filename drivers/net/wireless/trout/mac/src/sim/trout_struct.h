#ifndef _SIM_TROUT_STRUCT_H
#define _SIM_TROUT_STRUCT_H
#include <linux/types.h>
#include <linux/spinlock.h>
#include "normal_reg.h"
#include "share_mem.h"

struct pending_frame {
	u32 frame_ptr;
	u32 mpdu_cnt;
	spinlock_t lock;
};

static inline void __clear_pending_frame(struct pending_frame *pf)
{
	pf->frame_ptr = 0;
	pf->mpdu_cnt = 0;
}

static inline void init_pending_frame(struct pending_frame *pf)
{
	__clear_pending_frame(pf);
	spin_lock_init(&pf->lock);
}

static inline void pending_frame_lock(struct pending_frame *pf)
{
	spin_lock(&pf->lock);
}

static inline void pending_frame_unlock(struct pending_frame *pf)
{
	spin_unlock(&pf->lock);
}

static inline int is_frame_pending(struct pending_frame *pf)
{
	int ret = 0;
	pending_frame_lock(pf);
	if (!pf->mpdu_cnt) {
		ret = 1;
	}
	pending_frame_unlock(pf);

	return ret;
}

static inline void add_pending_frame(struct pending_frame *pf,
		u32 frame_ptr, u32 mpdu_cnt)
{
	pending_frame_lock(pf);
	if (!pf->frame_ptr) {
		pf->frame_ptr = frame_ptr;
	}
	pf->mpdu_cnt += mpdu_cnt;
	pending_frame_unlock(pf);
}

static inline void flush_pending_frame(struct pending_frame *pf,
		u32 *frame_ptr, u32 *mpdu_cnt)
{
	pending_frame_lock(pf);
	*mpdu_cnt = pf->mpdu_cnt;
	*frame_ptr = pf->frame_ptr;
	__clear_pending_frame(pf);
	pending_frame_unlock(pf);
}

struct trout_reg {

	/* general register */
	struct normal_reg pa_version;
	struct normal_reg pa_control;
	struct normal_reg pa_status;
	struct normal_reg mac_hw_id;
	struct normal_reg mac_hw_reset_ctrl;

	/* rx queue relative register */
	struct normal_reg rx_buffer;
	struct normal_reg hip_rx_buffer;

	/* tx queue relative register */
	struct normal_reg bk_queue_ptr;
	struct normal_reg be_queue_ptr;
	struct normal_reg vi_queue_ptr;
	struct normal_reg vo_queue_ptr;
	struct normal_reg hp_queue_ptr;
	struct normal_reg cf_queue_ptr;

	/* Beacon relative register */
	struct normal_reg tsf_ctrl;
	struct normal_reg beacon_pkt_ptr;
	struct normal_reg beacon_tx_param;

	/* rx interrupt relative register */
	struct normal_reg rx_frame_ptr;
	struct normal_reg rx_mpdu_count;
	struct normal_reg hip_rx_frame_pointer;

	/* tx interrupt relative register */
	struct normal_reg tx_frame_ptr;
	struct normal_reg tx_mpdu_count;

	/* interrupt register */
	struct normal_reg intr_clear;
	struct normal_reg intr_mask;
	struct normal_reg intr_status;
	struct normal_reg err_code;
	struct normal_reg err_intr_status;
	struct normal_reg err_intr_mask;

	/* phy interface and parameter register */
	struct normal_reg phy_reg_access_ctrl;
	struct normal_reg phy_reg_rw_data;
};

struct trout {
	struct trout_reg trout_reg;

	struct share_mem share_mem;

	struct pending_frame tx_pending;
	struct pending_frame rx_pending;
	struct pending_frame hip_rx_pending;

	atomic_t rx_intr_seq_nr;
};

#endif