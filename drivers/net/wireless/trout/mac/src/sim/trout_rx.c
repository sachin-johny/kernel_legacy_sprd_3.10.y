#include "trout.h"
#include "trout_rx.h"
#include "trout_tx.h"
#include "share_mem.h"
#include "trout_reg_def.h"
#include "normal_reg.h"
#include "debug.h"

#define HIGH_PRI_RXQ 1

static inline u32 get_rx_buf_reg_addr(int rxq_nr)
{
	return (rxq_nr == HIGH_PRI_RXQ) ? rMAC_HIP_RX_BUFF_ADDR
					: rMAC_RX_BUFF_ADDR;
}

static void *get_rx_buf(struct rx_dscr *rx_dscr)
{
	void *trout_rx_buf = (void *)rx_dscr->rx_buf;

	return from_trout_addr(trout_rx_buf);
}

static inline int get_rx_dscr_status(struct rx_dscr *rx_dscr)
{
	return (rx_dscr->status_and_cipher_type >> 4);
}

static inline void set_rx_dscr_status(struct rx_dscr *rx_dscr, int status)
{
	rx_dscr->status_and_cipher_type = 
		(rx_dscr->status_and_cipher_type & 0x0f)
		| ((status & 0x0f)<<4);
}

static inline void set_rx_dscr_cipher_type(struct rx_dscr *rx_dscr, int type)
{
	rx_dscr->status_and_cipher_type = 
		(rx_dscr->status_and_cipher_type & 0xf0)
		| (type & 0x0f);
}

static int is_valid_rx_dscr(struct rx_dscr *rx_dscr)
{
	/*
	 * S/W must initialize the status of the rx descriptor to
	 * INVALID_DSCR before appending it to rx queue
	 */
	return get_rx_dscr_status(rx_dscr) == INVALID_DSCR;
}

static struct rx_dscr *get_cur_rx_dscr(int rxq_nr)
{
	void *cur = (void *)dv_read_reg(get_rx_buf_reg_addr(rxq_nr));

	if (cur) {
		return (struct rx_dscr *)from_trout_addr(cur);
	} else {
		return NULL;
	}
}

static void update_normal_rx_dscr(struct rx_dscr *cur, struct rx_info *info)
{
	cur->msdus_in_buf = 1;
	cur->flags_and_reserved = (0x02 << 4);
	cur->bufs_of_mpdu = 1;
}

static inline int is_first_buf_of_amsdu(struct rx_info *info)
{
	return !info->buf_idx;
}

static inline int is_last_buf_of_amsdu(struct rx_info *info)
{
	return info->bufs_of_amsdu == info->buf_idx + 1;
}

static void update_amsdu_rx_dscr(struct rx_dscr *cur, struct rx_info *info)
{
	cur->msdus_in_buf = info->sub_msdus_of_buf;
	cur->bufs_of_mpdu = info->bufs_of_amsdu;

	if (is_first_buf_of_amsdu(info)) {
		cur->flags_and_reserved = (0x02 << 4);
	} else {
		cur->flags_and_reserved = 0;
	}
}

static void update_rx_dscr(struct rx_dscr *cur, int frame_len,
		struct rx_info *info)
{
	int status = (info->status) ? : RX_SUCCESSFUL;
	int no_cipher = 0x03;

	set_rx_dscr_status(cur, status);
	set_rx_dscr_cipher_type(cur, no_cipher);

	cur->phy_rx_mode = 0;
	cur->next_defrag_buf = 0;

	cur->frame_len = frame_len;
	cur->sequence_nr = atomic_inc_return(&get_trout()->rx_intr_seq_nr);

	if (info->is_amsdu) {
		update_amsdu_rx_dscr(cur, info);
	} else {
		update_normal_rx_dscr(cur, info);
	}
	
}

static inline void update_rx_queue_register(struct rx_dscr *cur, int rxq_nr)
{
	dv_write_reg(cur->next_dscr, get_rx_buf_reg_addr(rxq_nr));
}

static void raise_hip_rx_irq(u32 frame_ptr)
{
	struct pending_frame *hip_rx_pending = &get_trout()->hip_rx_pending;

	add_pending_frame(hip_rx_pending, frame_ptr, 1);

	raise_trout_irq();
}

static void raise_normal_rx_irq(u32 frame_ptr, struct rx_info *info)
{
	static u32 amsdu_base_frame_ptr = 0;

	struct pending_frame *rx_pending = &get_trout()->rx_pending;
	u32 mpdu_cnt = 1;

	if (info->is_amsdu) {
		if (is_first_buf_of_amsdu(info)) {
			amsdu_base_frame_ptr = frame_ptr;
		}
		if(!is_last_buf_of_amsdu(info)) {
			return;
		}
		frame_ptr = amsdu_base_frame_ptr;
		mpdu_cnt = info->bufs_of_amsdu;
	}

	add_pending_frame(rx_pending, frame_ptr, mpdu_cnt);
	raise_trout_irq();
}

static void raise_rx_irq(int rxq_nr, struct rx_dscr *cur, struct rx_info *info)
{
	u32 frame_ptr = (u32)__to_trout_addr(cur);

	if (rxq_nr == HIGH_PRI_RXQ) {
		raise_hip_rx_irq(frame_ptr);
	} else {
		raise_normal_rx_irq(frame_ptr, info);
	}
}

static int do_recv_from_dv(int rxq_nr, struct rx_dscr *cur, const void *src,
		int size, struct rx_info *info)
{
	if (size > (int)RX_PACKET_SIZE) {
		return -ENOSPC;
	}

	/*
	 * the rx buf (except the first one) of A-MSDU
	 * frame don't contain the mac header, skip it
	 */
	if (info->is_amsdu && !is_first_buf_of_amsdu(info)) {
		src += info->hdr_len;
		size -= info->hdr_len;
	}

	memcpy(get_rx_buf(cur), src, size);

	update_rx_dscr(cur, size, info);

	update_rx_queue_register(cur, rxq_nr);

	raise_rx_irq(rxq_nr, cur, info);

	return 0;
}

static int is_block_ack_frame(const u8 *src)
{
	return (src[0] >> 2) == 0x25;
}

int trout_recv_from(DV_IOCTL_T *ioctl, const void *src)
{
	int rxq_nr = ioctl->addr;
	int size = ioctl->dataLen;
	struct rx_info *info = &ioctl->rx_info;
	struct rx_dscr *cur;

	if (is_block_ack_frame(src)) {

		/* the offset of block ack frame's content is 16 */
		block_ack_arrived(src + 16);
		return 0;
	}

	cur = get_cur_rx_dscr(rxq_nr);

	if (!cur) {
		return -EAGAIN;
	} else if (!is_valid_rx_dscr(cur)) {
		DV_DBG("invalid rx descriptor\n");
		BUG();
		return -EINVAL;
	}

	return do_recv_from_dv(rxq_nr, cur, src, size, info);	
}

