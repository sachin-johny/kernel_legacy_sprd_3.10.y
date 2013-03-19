#ifndef __SPRD_BUSMONITOR_H__
#define __SPRD_BUSMONITOR_H__

enum sci_bm_index {
	AXI_BM0 = 0x0,
	AXI_BM1,
	AXI_BM2,
	AXI_BM3,
	AXI_BM4,
	AXI_BM5,
	AXI_BM6,
	AXI_BM7,
	AXI_BM8,
	AXI_BM9,
	AHB_BM0,
	AHB_BM1,
	AHB_BM2,
	BM_SIZE,
};

struct sci_bm_cfg {
	u32 addr_min;
	u32 addr_max;
	u32 addr_msk;
	u64 data_min;
	u64 data_max;
	u32 data_msk;
};

struct sci_bm_match_data {
	u32 match_addr;
	u32 match_cmd;
	u64 match_data;
};

/*
int sci_bm_config(int bm_index, const struct sci_bm_cfg *cfg, int rw, void(*call_back)(void *));
int sci_bm_enable(int bm_index);
int sci_bm_disable(int bm_imdex);
*/
#endif
