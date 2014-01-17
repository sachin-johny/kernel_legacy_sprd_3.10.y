//zhuyg create this file for npi function.
//date: 2013-05-27

#ifdef TROUT_WIFI_NPI

#ifndef TROUT_WIFI_NPI_H
#define TROUT_WIFI_NPI_H

#include "common.h"
#include "mh.h"

extern UWORD8 g_mac_addr[6];
extern UWORD8 g_user_tx_rate;

#define COM_REG_CAPTURE_CTRL 	(0x4028<<2)
#define rSYSREG_WIFI_CFG        ((0x004e<<2))

typedef enum 
{
	TYPE_80211_B=0,
	TYPE_80211_G=1,  // eq TYPE_80211_A
	TYPE_80211_N=2,
	TYPE_80211_INVALID=3,
} WORK_MODE_TYPE_T;

UWORD8 get_tx_mode(UWORD8 tx_rate);
void clr_rx_count_reg(void);
UWORD32 int_to_str(UWORD32 i, char *string);
extern int scan_limit;
extern UWORD8 g_connect_ok_flag ;
extern int g_npi_scan_flag;

int npi_dispatch(struct ifreq * req);
extern void npi_tx_data_init(void);
extern void npi_tx_data_release(void);
extern void npi_tx_data_qwk(void);
extern void npi_tx_data_cwk(void);
#endif
#endif
