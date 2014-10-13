#ifndef __SPRD_BM_H__
#define __SPRD_BM_H__

#define AXI_BM_INTC_REG				0x0
#define AXI_BM_CFG_REG				0x4
#define AXI_BM_ADDR_MIN_REG			0x8
#define AXI_BM_ADDR_MAX_REG			0xc
#define AXI_BM_ADDR_MSK_REG			0x10
#define AXI_BM_DATA_MIN_L_REG		0x14
#define AXI_BM_DATA_MIN_H_REG		0x18
#define AXI_BM_DATA_MAX_L_REG		0x1c
#define AXI_BM_DATA_MAX_H_REG		0x20
#define AXI_BM_DATA_MSK_L_REG		0x24
#define AXI_BM_DATA_MSK_H_REG		0x28
#define AXI_BM_CNT_WIN_LEN_REG		0x2c
#define AXI_BM_PEAK_WIN_LEN_REG		0x30
#define AXI_BM_MATCH_ADDR_REG		0x34
#define AXI_BM_MATCH_CMD_REG		0x38
#define AXI_BM_MATCH_DATA_L_REG		0x3c
#define AXI_BM_MATCH_DATA_H_REG		0x40
#define AXI_BM_RTRANS_IN_WIN_REG	0x44
#define AXI_BM_RBW_IN_WIN_REG		0x48
#define AXI_BM_RLATENCY_IN_WIN_REG	0x4c
#define AXI_BM_WTRANS_IN_WIN_REG	0x50
#define AXI_BM_WBW_IN_WIN_REG		0x54
#define AXI_BM_WLATENCY_IN_WIN_REG	0x58
#define AXI_BM_PEAKBW_IN_WIN_REG	0x5c

#define SPRD_AHB_BM_NAME "sprd_ahb_bm"
#define SPRD_AXI_BM_NAME "sprd_axi_bm"

#define SPRD_BM_SUCCESS	0
#define BM_INT_MSK_STS	BIT(31)
#define BM_INT_CLR		BIT(29)
#define BM_INT_EN		BIT(28)
#define BM_CNT_CLR		BIT(4)
#define BM_CNT_START	BIT(3)
#define BM_CNT_EN		BIT(1)
#define BM_CHN_EN		BIT(0)
#define BM_DEBUG_ALL_CHANNEL	0xFF

/*depending on the platform*/
enum sci_bm_index {
	AXI_BM0_CA7 = 0x0,
	AXI_BM1_DISP,
	AXI_BM2_GSP_GPU,
	AXI_BM3_ZIP_AP,
	AXI_BM4_MM,
	AXI_BM5_CP0ARM_WCDMA,
	AXI_BM6_CP0_DSP,
	AXI_BM7_HARQ_LTEACC,
	AXI_BM8_CP1_DPS,
	AXI_BM9_CP1_A5,
	AHB_BM0_DAP_A7_DMA,
	AHB_BM1_SDIO_EMMC,
	AHB_BM2_NFC_USB,
	BM_SIZE,
};

enum sci_ahb_bm_index {
	AHB_BM0_DAP = AHB_BM0_DAP_A7_DMA,
	AHB_BM0_CA7,
	AHB_BM0_DMA_WR,
	AHB_BM0_DMA_RD,
	AHB_BM1_SDIO0,
	AHB_BM1_SDIO1,
	AHB_BM1_SDIO2,
	AHB_BM1_EMMC,
	AHB_BM2_NFC,
	AHB_BM2_USB,
	AHB_BM2_HSIC,
	BM_CHANNEL_SIZE,
};

enum sci_bm_chn {
	CHN0 = 0x0,
	CHN1,
	CHN2,
	CHN3,
};

enum sci_bm_mode {
	R_MODE,
	W_MODE,
	RW_MODE,
	/*
	PER_MODE,
	RST_BUF_MODE,
	*/
};

struct bm_per_info {
	u32 t_start;
	u32 t_stop;
	u32 tmp1;
	u32 tmp2;
	u32 per_data[10][6];
};

struct sci_bm_cfg {
	u32 bm_bits;
	u32 win_len;
	u32 addr_min;
	u32 addr_max;
	u32 addr_msk;
	u32 data_min_l;
	u32 data_min_h;
	u32 data_max_l;
	u32 data_max_h;
	u32 data_msk_l;
	u32 data_msk_h;
	u32 chn;
	u32 fun;
	u32 data;
	u32 peak_win_len;
	enum sci_bm_mode bm_mode;
};

struct sci_bm_reg {
	u32 intc;
	u32 cfg;
	u32 addr_min;
	u32 addr_max;
	u32 addr_msk;
	u32 data_min_l;
	u32 data_min_h;
	u32 data_max_l;
	u32 data_max_h;
	u32 data_msk_l;
	u32 data_msk_h;
	u32 cnt_win_len;
	u32 peak_win_len;
	u32 match_addr;
	u32 match_cmd;
	u32 match_data_l;
	u32 match_data_h;
	u32 rtrans_in_win;
	u32 rbw_in_win;
	u32 rlatency_in_win;
	u32 wtrans_in_win;
	u32 wbw_in_win;
	u32 wlatency_in_win;
	u32 peakbw_in_win;
};

#endif
