#ifndef _TROUT_TX_H
#define _TROUT_TX_H
#include <linux/types.h>
#include "typedef.h"
#include "share_mem_info.h"
#include "dv_ioctl.h"

/* for more information, refere the Document: IWLANMAC_ABGEIN_HDL_UG.pdf */
struct tx_dscr {
	u8 ign[3];
	u8 status_and_qos;
	u32 ign_0[3];
	u32 next_dscr;
	u32 ign_1[22];
	u16 ign_2;
	u16 frame_len;
	u32 ign_3[2];
	u32 tx_buf;
	u32 sub_msdu_tab;
	u8 q_nr;
	u8 sub_msdu_nr;
	u8 mac_header_len;
	u8 mac_header_offset;
	u8 data_rate[7];
	u8 tssi_val;

	u32 reserve[1];
};

struct sub_msdu_tab {
	u32 buf;
	u16 len;
	u16 offset;
};

struct tx_mem_struct {
	struct tx_dscr dscr;
	UWORD8 tx_buf[TX_BUFF_SIZE];
};

extern void block_ack_arrived(const u8 *ba);

extern void trout_tx_frame(void);

extern void trout_beacon_frame(u32 beacon_ptr);

extern int trout_send_to(DV_IOCTL_T *ioctl, void *dst);

#endif
