#include "trout.h"
#include "trout_reg.h"
#include "trout_tx_frame.h"
#include "trout_tx.h"

struct trout *get_trout(void)
{
	static struct trout singleton;
	return &singleton;
}

static inline int is_irq_could_raise(u32 mask, u32 status, u32 flag)
{
	return !((mask & flag) || (status & flag));
}

static int prepare_hip_rx_irq(struct trout *trout, u32 mask, u32 status)
{
	u32 frame_ptr;
	u32 mpdu_cnt = 0;

	if (!is_irq_could_raise(mask, status, HIP_RX_COMPLETE)) {
		return 0;
	}
	
	flush_pending_frame(&trout->hip_rx_pending, &frame_ptr, &mpdu_cnt);

	if (!mpdu_cnt) {
		return 0;
	}

	dv_write_reg(frame_ptr, rMAC_HIP_RX_FRAME_POINTER);
	append_intr_status(HIP_RX_COMPLETE);
	return 1;
}

static int prepare_rx_irq(struct trout *trout, u32 mask, u32 status)
{
	u32 frame_ptr;
	u32 mpdu_cnt = 0;

	if (!is_irq_could_raise(mask, status, RX_COMPLETE)) {
		return 0;
	}

	flush_pending_frame(&trout->rx_pending, &frame_ptr, &mpdu_cnt);

	if (!mpdu_cnt) {
		return 0;
	}

	dv_write_reg(mpdu_cnt, rMAC_RX_MPDU_COUNT);
	dv_write_reg(frame_ptr, rMAC_RX_FRAME_POINTER);
	append_intr_status(RX_COMPLETE);
	return 1;
}

static int prepare_tx_irq(struct trout *trout, u32 mask, u32 status)
{
	u32 frame_ptr;
	u32 mpdu_cnt = 0;


	if (!is_irq_could_raise(mask, status, TX_COMPLETE)) {
		return 0;
	}

	flush_pending_frame(&trout->tx_pending, &frame_ptr, &mpdu_cnt);

	if (!mpdu_cnt) {
		return 0;
	}

	dv_write_reg(mpdu_cnt, rMAC_TX_MPDU_COUNT);
	dv_write_reg(frame_ptr, rMAC_TX_FRAME_POINTER);
	append_intr_status(TX_COMPLETE);
	return 1;
}

struct tbtt {
	struct timer_list timer;
	atomic_t pending;
	struct work_struct work;
};

static struct tbtt *get_tbtt(void);

static void tbtt_fun(struct work_struct *unused)
{
	if( reset_mac_trylock() == 0 ) return;
	u32 beacon_ptr = dv_read_reg(rMAC_BEACON_POINTER);
	u32 tsf_ctrl = dv_read_reg(rMAC_TSF_CON);

	atomic_set(&get_tbtt()->pending, 1);

	if (!(tsf_ctrl & (1U << 2))) {
		trout_beacon_frame(beacon_ptr);
	}
	raise_trout_irq();
	tbtt_restart();
	reset_mac_unlock();
}

static void tbtt_timer_fun(unsigned long unused)
{
  if((data != NULL) && ((BOOL_T)atomic_read(&g_mac_reset_done) == BTRUE))
	schedule_work_on(0, &get_tbtt()->work);
}

static struct tbtt *get_tbtt(void)
{
	static struct tbtt tbtt = {
		.timer = TIMER_INITIALIZER(tbtt_timer_fun, 0, 0),
		.work = __WORK_INITIALIZER(tbtt.work, tbtt_fun),
	};
	return &tbtt;
}

static int is_tbtt_pending(void)
{
	return atomic_read(&get_tbtt()->pending);
}

static void clear_tbtt_pending(void)
{
	atomic_set(&get_tbtt()->pending, 0);
}

void tbtt_restart(void)
{
	mod_timer(&get_tbtt()->timer, jiffies + 1*HZ);
}

void tbtt_stop(void)
{
	del_timer_sync(&get_tbtt()->timer);
}

static int prepare_tbtt_irq(struct trout *trout, u32 mask, u32 status)
{
	if (!is_irq_could_raise(mask, status, TBTT)) {
		return 0;
	}

	if (is_tbtt_pending()) {
		clear_tbtt_pending();
		append_intr_status(TBTT);
		return 1;
	}

	return 0;
}

static int trout_irq_prepare(void *param)
{
	int ret = 0;
	struct trout *trout = param;
	u32 intr_mask = get_intr_mask();
	u32 intr_status = get_intr_status();

	if (prepare_hip_rx_irq(trout, intr_mask, intr_status)) {
		ret = 1;
	}

	if (prepare_rx_irq(trout, intr_mask, intr_status)) {
		ret = 1;
	}

	if (prepare_tx_irq(trout, intr_mask, intr_status)) {
		ret = 1;
	}

	if (prepare_tbtt_irq(trout, intr_mask, intr_status)) {
		ret = 1;
	}

	return ret;
}

int trout_init(void)
{
	struct trout *trout = get_trout();

	atomic_set(&trout->rx_intr_seq_nr, 1);

	init_pending_frame(&trout->tx_pending);
	init_pending_frame(&trout->rx_pending);
	init_pending_frame(&trout->hip_rx_pending);

	alloc_virt_irq(TROUT_INTR_VEC, trout_irq_prepare, trout);
	alloc_virt_irq(SPI_DMA_INTR, NULL, NULL);

	trout_reg_init(&trout->trout_reg);

	return 0;
}

void trout_cleanup(void)
{
	tbtt_stop();
	cleanup_send_list();
}

int is_trout_frame_pending(void)
{
	struct trout *trout = get_trout();

	return is_frame_pending(&trout->hip_rx_pending)
		|| is_frame_pending(&trout->rx_pending)
		|| is_frame_pending(&trout->tx_pending);
}

