#include <linux/slab.h>
#include <linux/module.h>
#include <linux/random.h>
#include <linux/completion.h>
#include "trout.h"
#include "share_mem.h"
#include "trout_tx.h"
#include "trout_tx_frame.h"
#include "trout_reg_def.h"
#include "normal_reg.h"
#include "debug.h"

/* redefined staff, need remove after merge */
#define DE_MAC_WMM
#ifndef DE_MAC_WMM
#define NUM_TX_QUEUE 3 
#else
#define NUM_TX_QUEUE 5 
#endif

enum {
	AC_VO_Q = 1,
	AC_VI_Q = 2,
	AC_BE_Q = 3,
	AC_BK_Q = 4,
};
/*-----------------------------------------*/

static u32 get_tx_buf_reg_addr(int txq_nr)
{
	const u32 reg_array[] = {
		rMAC_EDCA_PRI_HP_Q_PTR,
		rMAC_EDCA_PRI_VO_Q_PTR,
#ifndef DE_MAC_WMM
		rMAC_EDCA_PRI_CF_Q_PTR,
#else
		rMAC_EDCA_PRI_VI_Q_PTR,
		rMAC_EDCA_PRI_BE_Q_PTR,
		rMAC_EDCA_PRI_BK_Q_PTR,
#endif
	};

	return reg_array[txq_nr];
}

static u8 is_block_ack_request(struct tx_dscr *tx_dscr)
{
	u8 ack_policy = tx_dscr->ign[2] >> 4;
	return ack_policy == 6;
}

static u8 get_tx_status(struct tx_dscr *tx_dscr)
{
	return (tx_dscr->status_and_qos >> 5) & 0x03;
}

static void set_tx_status(u8 status, struct tx_dscr *tx_dscr)
{
	const u8 status_mask = 0x03 << 5;
	tx_dscr->status_and_qos = (tx_dscr->status_and_qos & ~status_mask)
					| (status << 5);
}

static int is_valid_tx_status(struct tx_dscr *tx_dscr)
{
	const u8 pending = 0x01;

	return get_tx_status(tx_dscr) == pending;
}

static int is_valid_tid(int txq_nr, struct tx_dscr *tx_dscr)
{
	const u8 tid = tx_dscr->status_and_qos & 0x0F;
	const int tid_to_qnr[] = {
		AC_BE_Q, AC_BK_Q, AC_BK_Q, AC_BE_Q,
		AC_VI_Q, AC_VI_Q, AC_VO_Q, AC_VO_Q,
		AC_BE_Q, AC_BE_Q, AC_BE_Q, AC_BE_Q,
		AC_BE_Q, AC_BE_Q, AC_BE_Q, AC_BE_Q,
	};

	return (txq_nr==0 || (tid_to_qnr[tid] == txq_nr));
}

static int is_valid_tx_dscr(int txq_nr, struct tx_dscr *tx_dscr)
{
	return is_valid_tx_status(tx_dscr)
		&& is_valid_tid(txq_nr, tx_dscr)
		&& (tx_dscr->q_nr == txq_nr);
}

static struct tx_dscr *do_get_cur_tx_dscr(int *output_txq_nr)
{
	int txq_nr;
	u32 trout_addr;

	for (txq_nr=0; txq_nr<NUM_TX_QUEUE; ++txq_nr) {
		trout_addr = dv_read_reg(get_tx_buf_reg_addr(txq_nr));
		if (trout_addr) {
			*output_txq_nr = txq_nr;
			return from_trout_addr((void *)trout_addr);
		}
	}

	return NULL;
}

static struct tx_dscr *get_cur_tx_dscr(void)
{
	int txq_nr;
	struct tx_dscr *cur;

	cur = do_get_cur_tx_dscr(&txq_nr);
	if (!cur) {
		return ERR_PTR(-EAGAIN);
	}

	if (is_valid_tx_dscr(txq_nr, cur)) {
		return cur;
	} else {
		DV_DBG("invalid tx descriptor\n");
		BUG();
		return ERR_PTR(-EINVAL);
	}
}

void *get_tx_buf(struct tx_dscr *dscr)
{
	u32 trout_addr = dscr->tx_buf + dscr->mac_header_offset;
	return from_trout_addr((void *)trout_addr);
}

static void update_tx_dscr(struct tx_dscr *cur, int status)
{
	u8 not_pending = 0x02;
	status = status ?: not_pending;

	set_tx_status(status, cur);
	cur->ign_1[0] = 0xFFFFFFFF;
}

static void raise_tx_irq(struct tx_dscr *cur)
{
	u32 frame_ptr = (u32)__to_trout_addr(cur);
	struct pending_frame *tx_pending = &get_trout()->tx_pending;

	add_pending_frame(tx_pending, frame_ptr, 1);

	raise_trout_irq();
}

static int tx_err_rate;
module_param(tx_err_rate, int, S_IRUGO | S_IWUSR);

static int generate_random_tx_status(void)
{
	int status = 0;
	if (tx_err_rate) {
		u8 rnd;
		get_random_bytes(&rnd, sizeof(rnd));
		if (rnd % 100 < tx_err_rate) {
			status = 3;
		}
	}

	return status;
}

static void do_fill_tx_frame(struct tx_frame *frame, struct tx_dscr *cur,
		struct sub_msdu_tab *tab)
{
	void *dst = frame->data;
	void *sub_msdu = get_tx_buf(cur) + tab->offset;

	memcpy(dst, get_tx_buf(cur), cur->mac_header_len);
	dst += cur->mac_header_len;

	BUG_ON(sub_msdu != __from_trout_addr((void*)tab->buf + tab->offset));
	memcpy(dst, sub_msdu, tab->len);
}

static int fill_tx_frame(struct tx_dscr *cur, int status)
{
	int i;
	struct sub_msdu_tab *tab = from_trout_addr((void *)cur->sub_msdu_tab);

	for (i=0; i<cur->sub_msdu_nr; ++i, ++tab) {
		struct tx_info *info;
		struct tx_frame *frame = alloc_tx_frame(cur->frame_len);

		if (!frame) {
			goto err;
		}

		info = &frame->info;
		info->is_amsdu = (cur->sub_msdu_nr > 1) ? 1 : 0;
		info->status = status;
		info->sub_msdus_of_buf = cur->sub_msdu_nr;
		info->sub_msdu_idx = i;

		do_fill_tx_frame(frame, cur, tab);

		add_tx_frame(frame);
	}
	return 0;
err:
	while (i--) {
		del_tx_frame_tail();
	}
	return -1;
}

#define BLOCK_ACK_FRAME_LEN 12

struct block_ack_info {
	struct completion cmpl;
	u8 ba_content[BLOCK_ACK_FRAME_LEN];
};

static struct block_ack_info *get_ba_info(void)
{
	static struct block_ack_info singleton = {
		.cmpl = COMPLETION_INITIALIZER(singleton.cmpl),
	};
	return &singleton;
}

static void wait_for_block_ack(void)
{
	wait_for_completion(&get_ba_info()->cmpl);
}

void block_ack_arrived(const u8 *ba)
{
	memcpy(get_ba_info()->ba_content, ba, BLOCK_ACK_FRAME_LEN);
	complete(&get_ba_info()->cmpl);
}

static int prepare_trout_tx_frame(struct tx_dscr *cur)
{
	int status = generate_random_tx_status();

	if (fill_tx_frame(cur, status) < 0) {
		return -1;
	}

	if (is_block_ack_request(cur)) {
		wait_for_block_ack();

		/* BA H/W Flags */
		cur->ign_1[2] = 0x03;

		/* copy BA Frame content to the BAR TX descriptor */
		memcpy(&cur->ign_1[3], get_ba_info()->ba_content,
				BLOCK_ACK_FRAME_LEN);
	}

	update_tx_dscr(cur, status);

	dv_write_reg(cur->next_dscr, get_tx_buf_reg_addr(cur->q_nr));

	raise_tx_irq(cur);

	return 0;
}

extern void wake_up_trout_send(void);

void trout_tx_frame(void)
{
	struct tx_dscr *cur;

	while (1) {
		cur = get_cur_tx_dscr();
		if (IS_ERR(cur)) {
			break;
		}

		if (prepare_trout_tx_frame(cur) < 0) {
			break;
		}
	}

	wake_up_trout_send();
}

void trout_beacon_frame(u32 beacon_ptr)
{
	void *beacon = from_trout_addr((void *)beacon_ptr);
	int frame_len = dv_read_reg(rMAC_BEACON_TX_PARAMS) & 0xFFFF;
	struct tx_frame *frame;

	frame = alloc_tx_frame(frame_len);

	if (!frame) {
		return;
	}

	memcpy(frame->data, beacon, frame_len);

	add_tx_frame(frame);
}

int trout_send_to(DV_IOCTL_T *ioctl, void *dst)
{
	int ret;
	int size = ioctl->dataLen;
	struct tx_frame *frame = fetch_tx_frame();

	if (!frame) {
		return -EAGAIN;
	}

	if (frame->frame_len > size) {
		return -EFBIG;
	}

	memcpy(dst, frame->data, frame->frame_len);
	memcpy(&ioctl->tx_info, &frame->info, sizeof(struct tx_info));
	ret = frame->frame_len;

	free_tx_frame(frame);
	return ret;
}
