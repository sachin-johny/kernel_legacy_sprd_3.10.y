#ifndef __SPRD_BM_H__
#define __SPRD_BM_H__

#define BM_DEFAULT_VALUE_SET

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
#define AXI_BM_MATCH_ID_REG			0x64

#define AHB_BM_INTC_REG				0x00

#define SPRD_BM_SUCCESS	0
#define BM_INT_MSK_STS	BIT(31)
#define BM_INT_CLR		BIT(29)
#define BM_INT_EN		BIT(28)
#define BM_CNT_CLR		BIT(4)
#define BM_CNT_START	BIT(3)
#define BM_CNT_EN		BIT(1)
#define BM_CHN_EN		BIT(0)
#define BM_DEBUG_ALL_CHANNEL	0xFF

#define BM_DBG(f,x...)	printk(KERN_DEBUG "BM_INFO " f, ##x)
#define BM_INFO(f,x...)	printk(KERN_INFO "BM_INFO " f, ##x)
#define BM_WRN(f,x...)	printk(KERN_WARNING"BM_INFO " f, ##x)
#define BM_ERR(f,x...)	printk(KERN_ERR "BM_INFO " f, ##x)

#define PER_COUTN_LIST_SIZE 128
/*the buf can store 8 secondes data*/
#define PER_COUNT_RECORD_SIZE 800
#define PER_COUNT_BUF_SIZE (64 * 4 * PER_COUNT_RECORD_SIZE)

#define LOG_FILE_PATH "/mnt/obb/axi_per_log"
/*the log file size about 1.5Mbytes per min*/
#define LOG_FILE_SECONDS (60  * 30)
#define LOG_FILE_MAX_RECORDS (LOG_FILE_SECONDS * 100)
#define BM_CONTINUE_DEBUG_SIZE	20

static struct file *log_file;
static struct semaphore bm_seam;
static int buf_read_index;
static int buf_write_index;
static bool bm_irq_in_process;
/*the star log include a lot of unuseful info, we need skip it.*/
static int buf_skip_cnt;
long long t_stamp;
static void *per_buf = NULL;

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

struct bm_callback_desc {
	void (*fun)(void *);
	void *data;
};
static struct bm_callback_desc bm_callback_set[BM_SIZE];

enum sci_bm_cmd_index {
	BM_STATE = 0x0,
	BM_CHANNELS,
	BM_AXI_DEBUG_SET,
	BM_AHB_DEBUG_SET,
	BM_PERFORM_SET,
	BM_PERFORM_UNSET,
	BM_OCCUR,
	BM_CONTINUE_SET,
	BM_CONTINUE_UNSET,
	BM_DFS_SET,
	BM_DFS_UNSET,
	BM_PANIC_SET,
	BM_PANIC_UNSET,
	BM_BW_CNT_START,
	BM_BW_CNT_STOP,
	BM_BW_CNT_RESUME,
	BM_BW_CNT,
	BM_BW_CNT_CLR,
	BM_DBG_INT_CLR,
	BM_DBG_INT_SET,
	BM_CMD_MAX,
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

struct bm_debug_info{
	u32 bm_index;
	u32 msk_addr;
	u32 msk_cmd;
	u32 msk_data_l;
	u32 msk_data_h;
	u32 msk_id;
};
static struct bm_debug_info debug_bm_int_info[BM_SIZE];

struct bm_continue_debug{
	bool bm_continue_dbg;
	u32 loop_cnt;
	u32 current_cnt;
	struct bm_debug_info bm_ctn_info[BM_CONTINUE_DEBUG_SIZE];
};
static struct bm_continue_debug bm_ctn_dbg;

struct bm_state_info{
	bool bm_dbg_st;
	bool bm_dfs_off_st;
	bool bm_panic_st;
	bool bm_stack_st;
};
static struct bm_state_info bm_st_info;

struct bm_chn_name_info {
	u32 chn_num;
	unsigned char *chn_name;
};
static struct bm_chn_name_info bm_chn_name[BM_CHANNEL_SIZE + 1] = {
	{AXI_BM0_CA7, "CA7"},
	{AXI_BM1_DISP, "DISP"},
	{AXI_BM2_GSP_GPU, "GSP/GPU"},
	{AXI_BM3_ZIP_AP, "ZIP/AP"},
	{AXI_BM4_MM, "MM"},
	{AXI_BM5_CP0ARM_WCDMA, "CP0 ARM0/1/WCDMA"},
	{AXI_BM6_CP0_DSP, "CP0 DSPx2"},
	{AXI_BM7_HARQ_LTEACC, "HARQ/LTE ACC"},
	{AXI_BM8_CP1_DPS, "CP1 DPS"},
	{AXI_BM9_CP1_A5, "CP1 A5"},

	{AHB_BM0_DAP, "DAP"},
	{AHB_BM0_CA7, "CA7"},
	{AHB_BM0_DMA_WR, "DMA WRITE"},
	{AHB_BM0_DMA_RD, "DMA READ"},
	{AHB_BM1_SDIO0, "SDIO 0"},
	{AHB_BM1_SDIO1, "SDIO 1"},
	{AHB_BM1_SDIO2, "SDIO 2"},
	{AHB_BM1_EMMC, "EMMC"},
	{AHB_BM2_NFC, "NFC"},
	{AHB_BM2_USB, "USB"},
	{AHB_BM2_HSIC, "HSIC"},
	{BM_CHANNEL_SIZE,""},
};

struct bm_id_name {
	unsigned char chn_name[4];
};
static struct bm_id_name bm_match_id[AXI_BM9_CP1_A5] = {
	{"CA7"},
	{"DISPC","TMC"},
	{"GSP","GPU"},
	{"ZIPENC","ZIPDEC","AP"},
	{"DCAM","ISP","VSP","JPG"},
	{"CP0 WCDMA","CP0 ARM0","CP0 ARM1","CP0 WCDMA"},
	{"CP0 DSP"},
	{"CP1 LTEACC/HARQ"},
	{"CP1 DSP"},
	{"CA5","CA5 AHB"},
};

#ifdef BM_DEFAULT_VALUE_SET
struct bm_chn_def_val {
	u32 str_addr;
	u32 end_addr;
	u32 min_data;
	u32 max_data;
	u32 mode;
	u32 chn_sel;
};
static struct bm_chn_def_val bm_store_vale[BM_SIZE];
static struct bm_chn_def_val bm_def_value[BM_SIZE] = {
	{0x00009000, 0x0062acbc, 0x0fffffff, 0x00000000, W_MODE, 0}, //ca7
	{0x00009000, 0x0062acbc, 0x0fffffff, 0x00000000, W_MODE, 0}, //DISP
	{0x00009000, 0x0062acbc, 0x0fffffff, 0x00000000, W_MODE, 0}, //GSP/GPU
	{0x00009000, 0x0062acbc, 0x0fffffff, 0x00000000, W_MODE, 0}, //ZIP/AP
	{0x00009000, 0x0062acbc, 0x0fffffff, 0x00000000, W_MODE, 0}, //MM
	{0x00009000, 0x0062acbc, 0x0fffffff, 0x00000000, W_MODE, 0}, //CP0 ARM0/1/WCDMA
	{0x00009000, 0x0062acbc, 0x0fffffff, 0x00000000, W_MODE, 0}, //CP0 DSPx2
	{0x00009000, 0x0062acbc, 0x0fffffff, 0x00000000, W_MODE, 0}, //HARQ/LTE ACC
	{0x00009000, 0x0062acbc, 0x0fffffff, 0x00000000, W_MODE, 0}, //CP1 DPS
	{0x00009000, 0x0062acbc, 0x0fffffff, 0x00000000, W_MODE, 0}, //CP1 A5

	{0x00000000, 0x00000000, 0x00000000, 0x00000000, W_MODE, 0}, //0-DAP, 1-CA7, 2-DMA WRITE, 3-DMA READ
	{0x00000000, 0x00000000, 0x00000000, 0x00000000, W_MODE, 0}, //0-SDIO 0, 1-SDIO 1, 2-SDIO 2, 3-EMMC
	{0x00000000, 0x00000000, 0x00000000, 0x00000000, W_MODE, 0}, //0-NFC, 1-USB, 2-HSIC
};
#endif

#endif
