#include "trout_reg.h"
#include "trout_tx.h"

static inline int is_bit_clear_to_set(u32 cur_val, u32 old_val, int bit_index)
{
	u32 change_bit = cur_val ^ old_val;
	u32 specify_bit = 1U << bit_index;

	return (change_bit & specify_bit)
		&& (specify_bit & cur_val);
}

static inline int is_bit_set_to_clear(u32 cur_val, u32 old_val, int bit_index)
{
	u32 change_bit = cur_val ^ old_val;
	u32 specify_bit = 1U << bit_index;

	return (change_bit & specify_bit)
		&& (specify_bit & old_val);
}

static void reset_reg_event(struct normal_reg *reg,
		enum reg_event_type type, u32 old_val)
{
	int need_reset = (reg->old_val ^ reg->val) & 0x01;
	normal_reg_unlock(reg);

	if (need_reset) {
		tbtt_stop();
		reg_space_reset();
	}
	normal_reg_lock(reg);
}

static void pa_control_reg_event(struct normal_reg *reg,
		enum reg_event_type type, u32 old_val)
{
	u32 cur_val = reg->val;

	normal_reg_unlock(reg);

	if (is_bit_clear_to_set(cur_val, old_val, 12)) {
		dv_set_reg_bit(1U << 6, rMAC_PA_STAT);
	} else if (is_bit_set_to_clear(cur_val, old_val, 12)) {
		dv_clear_reg_bit(1U << 6, rMAC_PA_STAT);
	}

	normal_reg_lock(reg);
}

static void init_general_reg(struct trout_reg *trout_reg)
{
	reg_default_register(&trout_reg->pa_version, rMAC_PA_VER);

	reg_with_sync_event_register(&trout_reg->pa_control, rMAC_PA_CON,
			pa_control_reg_event, 0);

	reg_default_register(&trout_reg->pa_status, rMAC_PA_STAT);

	reg_with_reset_val_register(&trout_reg->mac_hw_id,
			rMAC_HW_ID, 0x4E4D4143);

	reg_with_sync_event_register(&trout_reg->mac_hw_reset_ctrl,
			rMAC_RESET_CTRL, reset_reg_event, 0);
}

static void init_rx_queue_reg(struct trout_reg *trout_reg)
{
	reg_default_register(&trout_reg->rx_buffer, rMAC_RX_BUFF_ADDR);

	reg_default_register(&trout_reg->hip_rx_buffer, rMAC_HIP_RX_BUFF_ADDR);
}

static void tx_queue_event(struct work_struct *work)
{
	trout_tx_frame();
}

static void init_tx_queue_reg(struct trout_reg *trout_reg)
{
	reg_with_async_event_register(&trout_reg->bk_queue_ptr,
			rMAC_EDCA_PRI_BK_Q_PTR, tx_queue_event, 0);

	reg_with_async_event_register(&trout_reg->be_queue_ptr,
			rMAC_EDCA_PRI_BE_Q_PTR, tx_queue_event, 0);

	reg_with_async_event_register(&trout_reg->vi_queue_ptr,
			rMAC_EDCA_PRI_VI_Q_PTR, tx_queue_event, 0);

	reg_with_async_event_register(&trout_reg->vo_queue_ptr,
			rMAC_EDCA_PRI_VO_Q_PTR, tx_queue_event, 0);

	reg_with_async_event_register(&trout_reg->hp_queue_ptr,
			rMAC_EDCA_PRI_HP_Q_PTR, tx_queue_event, 0);

	reg_with_async_event_register(&trout_reg->cf_queue_ptr,
			MAC_EDCA_PRI_CF_Q_PTR, tx_queue_event, 0);

	
}

static void tsf_ctrl_event(struct normal_reg *reg, enum reg_event_type type,
		u32 old_val)
{
	if (is_bit_clear_to_set(reg->val, old_val, 0)) {
		tbtt_restart();
	}

	if (is_bit_set_to_clear(reg->val, old_val, 0)) {
		tbtt_stop();
	}
}

static void init_beacon_reg(struct trout_reg *trout_reg)
{
	reg_with_sync_event_register(&trout_reg->tsf_ctrl,
			rMAC_TSF_CON, tsf_ctrl_event, 0);
	reg_default_register(&trout_reg->beacon_pkt_ptr, rMAC_BEACON_POINTER);
	reg_default_register(&trout_reg->beacon_tx_param, rMAC_BEACON_TX_PARAMS);
}

static void intr_clear_reg_event(struct normal_reg *reg,
		enum reg_event_type type, u32 old_val)
{
	u32 clear_bit = reg->val;
	normal_reg_unlock(reg);

	if (clear_bit) {
		dv_clear_reg_bit(clear_bit, rCOMM_INT_STAT);
		flush_pending_trout_irq();
	}

	normal_reg_lock(reg);
}

static void intr_mask_reg_event(struct normal_reg *reg,
		enum reg_event_type type, u32 old_val)
{
	u32 clear_bit = reg->val ^ old_val;

	normal_reg_unlock(reg);

	/*
	 * if any interrupt be unmasked, flush the interrupt,
	 * the isr would be scheduled if it had pending
	 */
	if (clear_bit & old_val) {
		flush_pending_trout_irq();
	}

	normal_reg_lock(reg);
}

static void init_common_intr_reg(struct trout_reg *trout_reg)
{
	reg_with_sync_event_register(&trout_reg->intr_clear, rCOMM_INT_CLEAR,
			intr_clear_reg_event, 0);

	reg_with_sync_event_register(&trout_reg->intr_mask, rCOMM_INT_MASK,
			intr_mask_reg_event, 0);

	reg_with_write_mask_register(&trout_reg->intr_status, rCOMM_INT_STAT,
			READ_ONLY);

	reg_default_register(&trout_reg->err_code, rMAC_ERROR_CODE);

	reg_default_register(&trout_reg->err_intr_status, rMAC_ERROR_STAT);

	reg_default_register(&trout_reg->err_intr_mask, rMAC_ERROR_MASK);
}

static void init_rx_intr_reg(struct trout_reg *trout_reg)
{
	reg_with_write_mask_register(&trout_reg->rx_frame_ptr,
			rMAC_RX_FRAME_POINTER, READ_ONLY);

	reg_with_write_mask_register(&trout_reg->rx_mpdu_count,
			rMAC_RX_MPDU_COUNT, 0xFFFFFF00);
	
	reg_with_write_mask_register(&trout_reg->hip_rx_frame_pointer,
			rMAC_HIP_RX_FRAME_POINTER, READ_ONLY);
}

static void init_tx_intr_reg(struct trout_reg *trout_reg)
{
	reg_with_write_mask_register(&trout_reg->tx_frame_ptr,
			rMAC_TX_FRAME_POINTER, READ_ONLY);

	reg_with_write_mask_register(&trout_reg->tx_mpdu_count,
			rMAC_TX_MPDU_COUNT, 0xFFFFFF00);
}

static void phy_reg_access_ctrl_event(struct normal_reg *reg,
		enum reg_event_type type, u32 old_val)
{
	if ((reg->old_val ^ reg->val) & 0x01) {
		reg->val &= ~1U;
	}
	normal_reg_unlock(reg);

	dv_write_reg(0, rMAC_PHY_REG_RW_DATA);

	normal_reg_lock(reg);
}

static void init_phy_reg(struct trout_reg *trout_reg)
{
	reg_with_sync_event_register(&trout_reg->phy_reg_access_ctrl,
			rMAC_PHY_REG_ACCESS_CON, phy_reg_access_ctrl_event,
			0);

	reg_default_register(&trout_reg->phy_reg_rw_data, rMAC_PHY_REG_RW_DATA);
}

int trout_reg_init(struct trout_reg *trout_reg)
{
	init_general_reg(trout_reg);
	init_rx_queue_reg(trout_reg);
	init_tx_queue_reg(trout_reg);
	init_beacon_reg(trout_reg);
	init_common_intr_reg(trout_reg);
	init_rx_intr_reg(trout_reg);
	init_tx_intr_reg(trout_reg);
	init_phy_reg(trout_reg);
	return 0;
}
