#ifndef TROUT_RF_H__
#define TROUT_RF_H__

typedef struct
{
	int	seek_succes;
	u32	freq_set;
	u32	freq_seek;
	u32	freq_offset;
	u32 rssi;
	u32 rf_rssi;
	u32 seek_cnt;
	u32 inpwr_sts;
	u32 fm_sts;
	atomic_t	fm_searching;
	atomic_t	fm_en;
    struct mutex mutex;
}trout_fm_info_t;

extern int trout_fm_init(void);
extern int trout_fm_deinit(void);
extern int trout_fm_en(void);
extern int trout_fm_dis(void);
extern int trout_fm_get_status(int *status);
extern int trout_fm_set_tune(u16 freq);
extern int trout_fm_seek(u16  frequency, u8   seek_dir,
		u32  time_out, u16 *freq_found);
extern int trout_fm_get_frequency(u16 *freq);
extern int trout_fm_rf_spi_read(u32 addr, u32 *data);
extern int trout_fm_rf_spi_write(u32 addr, u32 data);
extern int  trout_fm_stop_seek(void);

extern int trout_fm_pcm_pin_cfg(void);
extern int trout_fm_iis_pin_cfg(void);

#endif
