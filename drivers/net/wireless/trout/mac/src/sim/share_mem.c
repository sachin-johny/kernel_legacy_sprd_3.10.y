#include <linux/module.h>
#include "trout.h"
#include "share_mem.h"
#include "debug.h"

struct share_mem *get_share_mem(void)
{
	return &get_trout()->share_mem;
}

static int rx_struct_check(void)
{
	if (sizeof(struct rx_dscr) != RX_DSCR_LEN) {
		DV_DBG("rx_dscr size error\n");
		return -1;
	}

	if (sizeof(struct rx_q_element) != RX_DSCR_LEN+RX_PACKET_SIZE) {
		DV_DBG("rx_q_element size error\n");
		return -1;
	}

	if (sizeof(struct rx_mem_struct) != RX_MEM_SIZE) {
		DV_DBG("rx_mem_struct size error\n");
		return -1;
	}

	return 0;
}

static int tx_struct_check(void)
{
	if (sizeof(struct tx_dscr) != TX_DSCR_LEN) {
		DV_DBG("tx_dscr size error\n");
		return -1;
	}

	if (sizeof(struct tx_mem_struct) != TX_MEM_SIZE) {
		DV_DBG("tx_mem_struct size error\n");
		return -1;
	}

	return 0;
}

void struct_checker(void)
{
	if (rx_struct_check() < 0 || tx_struct_check() < 0) {
		BUG();
	}
}

void *from_trout_addr(void *trout_addr)
{
	if (is_valid_trout_addr(trout_addr)) {
		return __from_trout_addr(trout_addr);
	} else {
		DV_DBG("invalid trout address: %p\n", trout_addr);
		BUG();
		return NULL;
	}
}

