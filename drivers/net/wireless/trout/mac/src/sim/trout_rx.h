#ifndef _TROUT_RX_H
#define _TROUT_RX_H
#include <linux/types.h>
#include "typedef.h"
#include "share_mem_info.h"
#include "dv_ioctl.h"

enum rx_dscr_status {
	INVALID_DSCR	= 0,
	RX_SUCCESSFUL	= 1,
};

/* for more information, refere the Document: IWLANMAC_ABGEIN_HDL_UG.pdf */
struct rx_dscr {
	u16 frame_len;
	u8 data_rate;
	u8 status_and_cipher_type;
	u8 time_stamp;
	u8 snr;
	u8 rssi;
	u8 silence_lvl;
	u32 phy_rx_mode;
	u16 rx_legacy_len;
	u16 service_field;
	u16 signal_quality;
	u16 reserved;
	u8 msdus_in_buf;
	u8 flags_and_reserved;
	u8 bufs_of_mpdu;
	u8 sequence_nr;
	u32 next_defrag_buf;
	u32 rx_buf;
	u32 next_dscr;
	u32 prev_dscr;
	u32 reserve;
};

/* for simplicity, we bind the rx descriptor and the rx buffer together */
struct rx_q_element {
	struct rx_dscr dscr;
	UWORD8 rx_buf[RX_PACKET_SIZE];
};

struct rx_mem_struct
{
	struct rx_q_element normal_rx_q[NUM_RX_BUFFS];
	struct rx_q_element high_prio_rx_q[NUM_HIPR_RX_BUFFS];
};

extern int trout_recv_from(DV_IOCTL_T *ioctl, const void *src);
#endif
