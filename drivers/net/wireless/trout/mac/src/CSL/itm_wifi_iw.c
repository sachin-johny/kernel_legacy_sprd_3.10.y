
#include <linux/types.h>
#include <linux/if_arp.h>
#include <linux/ctype.h>
#include <linux/rtnetlink.h>
#include <linux/wireless.h>

#include <linux/inetdevice.h>
#include <asm/uaccess.h>
#include <linux/fs.h>

#include "itm_wifi_iw.h"
#include "maccontroller.h"
#include "frame.h"

//chenq add for debug
#include "csl_linux.h"
#include "common.h"
#include "mac_init.h"

//xuanyang 2013.4.24 get mac address
#include "mib.h"

#include "trout_trace.h"
//libing add for read hostap.conf 
#ifdef	BSS_ACCESS_POINT_MODE
#include "hostap_conf.h"
#include "mac_string.h"
extern UWORD8 saved_psk[40] ;
#endif

//zhuyg add for npi
#include "trout_share_mem.h"

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#include "ps_timer.h"
#define LOCK_TURE    1
#define LOCK_FALSE   0
#endif
#ifdef TROUT_WIFI_NPI
//zhuyg add
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/err.h> 
#include <linux/kthread.h>
#include "mac_init.h"
#include "event_manager.h"
#include "trout_wifi_npi.h"
#endif


#define ITMIW_DEBUD_PRINT
#ifdef ITMIW_DEBUD_PRINT
#define PRINTK_ITMIW(args...)  printk(args)
#else  
#define PRINTK_ITMIW(args...)
#endif 

#define ITMIW_FUNC_ENTER PRINTK_ITMIW(">===== chenq_itm %s enter =====>\n",__FUNCTION__)
#define ITMIW_FUNC_EXIT  PRINTK_ITMIW("<===== chenq_itm %s exit  =====<\n",__FUNCTION__)

//chenq add for wapi 2012-09-19
#ifdef MAC_WAPI_SUPP
#define ITM_WIFI_SECURITY_WAPI_ENABLE 1
#ifndef IW_AUTH_WAPI_VERSION_1
#define IW_AUTH_WAPI_VERSION_1		0x00000008
#endif

#ifndef IW_AUTH_KEY_MGMT_WAPI_PSK
#define IW_AUTH_KEY_MGMT_WAPI_PSK	0x04
#endif

#ifndef IW_AUTH_WAPI_ENABLED
#define IW_AUTH_WAPI_ENABLED		0x20
#endif

#ifndef IW_ENCODE_ALG_SM4
#define IW_ENCODE_ALG_SM4			0x20
#endif

#ifndef WLAN_CIPHER_SUITE_SMS4
#define WLAN_CIPHER_SUITE_SMS4			0x4
#endif

#endif

#define WIFI_WAPI_KEY_TYPE_PAIRWISE 1
#define WIFI_WAPI_KEY_TYPE_GROUP 0

#ifdef BSS_ACCESS_POINT_MODE
#define ITM_SUPPORT_WEXT_AP 1
#endif

#define ANDROID_BUILD 1
#define ITM_ONLY_NOTES 1
#define ITM_AP_ROUTER_THROUGHPUT_MEASUREMENTS 1
#define ITM_IW_ENCODE_ALG_WPAUSERKEY 8
 
#define ITM_CONFIG_BUF_SIZE 0x7FF

enum ANDROID_WIFI_CMD {
	ANDROID_WIFI_CMD_START = 0,	//Turn on Wi-Fi hardware	//OK if successful
	ANDROID_WIFI_CMD_STOP,		//Turn off Wi-Fi hardwoare	//OK if successful
	ANDROID_WIFI_CMD_RSSI,		//Return received signal strength indicator in dbm for current AP	//<ssid> rssi xx 
	ANDROID_WIFI_CMD_LINKSPEED,	//Return link speed in MBPS	//LinkSpeed xx 
	ANDROID_WIFI_CMD_MACADDR,	//Return mac address of the station	//Macaddr = xx:xx:xx:xx:xx:xx 
	ANDROID_WIFI_CMD_SCAN_ACTIVE,	//Set scan type to active	//OK if successful
	ANDROID_WIFI_CMD_SCAN_PASSIVE,	//Set scan type to passive	//OK if successfu
	ANDROID_WIFI_CMD_COUNTRY,
	ANDROID_WIFI_CMD_DHCP_STATUS,
	ANDROID_WIFI_CMD_MAX
};

const char *android_wifi_cmd_str[ANDROID_WIFI_CMD_MAX] = {
	"START",
	"STOP",
	"RSSI",
	"LINKSPEED",
	"MACADDR",
	"SCAN-ACTIVE",
	"SCAN-PASSIVE",
	"COUNTRY",
	"DHCP_STATUS", 
};


#define ITM_MAC_FMT "%02x:%02x:%02x:%02x:%02x:%02x"

#define ITM_MAC_ARG(x) ((u8*)(x))[0],((u8*)(x))[1],((u8*)(x))[2],((u8*)(x))[3],((u8*)(x))[4],((u8*)(x))[5]



#ifdef COMBO_SCAN
// chenq add for combo scan 2013-03-12
#define WEXT_CSCAN_AMOUNT 9
#define WEXT_CSCAN_BUF_LEN		360
#define WEXT_CSCAN_HEADER		"CSCAN S\x01\x00\x00S\x00"
#define WEXT_CSCAN_HEADER_SIZE		12
#define WEXT_CSCAN_SSID_SECTION		'S'
#define WEXT_CSCAN_CHANNEL_SECTION	'C'
#define WEXT_CSCAN_NPROBE_SECTION	'N'
#define WEXT_CSCAN_ACTV_DWELL_SECTION	'A'
#define WEXT_CSCAN_PASV_DWELL_SECTION	'P'
#define WEXT_CSCAN_HOME_DWELL_SECTION	'H'
#define WEXT_CSCAN_TYPE_SECTION		'T'
#else
#define WEXT_CSCAN_HEADER		"CSCAN S\x01\x00\x00S\x00"
#define WEXT_CSCAN_HEADER_SIZE		12
#endif

extern mac_struct_t g_mac;
extern void clear_tx_barrier(void);
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
extern void sta_sleep_disconnected(void);
#endif
extern int is_scanlist_report2ui;

#ifdef IBSS_BSS_STATION_MODE
extern BOOL_T g_keep_connection;
extern int itm_scan_flag;
//chenq add 2012-10-29
extern struct task_struct * itm_scan_task;
extern BOOL_T g_BusyTraffic;
extern BOOL_T g_UI_cmd_ps;
#endif

// add by Ke.Li at 2013-04-11 for itm_config read ram
#ifdef DEBUG_MODE
extern void display_trout_ram(WORD32 start_addr, WORD32 read_len);
#endif
// end add by Ke.Li at 2013-04-11 for itm_config read ram
//chenq add
unsigned char * g_itm_config_buf = NULL;
struct iw_statistics g_itm_iw_statistics;

UWORD8 g_psk_value[40];

//chenq add for wpa/rsn ie buf
UWORD8 g_wpa_rsn_ie_buf_3[MAX_SITES_FOR_SCAN][512] = {{0},};

//chenq add for wapi
/* WAPI Key */
typedef struct
{
    UWORD8                          unicastKey;
    /* If non zero, then unicast key otherwise group key */
    UWORD8                          keyIndex;
    UWORD8                          keyRsc[16];
    UWORD8                          authenticator;
    /* If non zero, then authenticator otherwise supplicant */
    UWORD8                          address[6];
    UWORD8                          key[32];
} wapi_key_t;


typedef struct
{
    UWORD8       rate_index;
    UWORD8       rate;
	UWORD8       point5;
} rate_table_t;

#ifdef NMAC_1X1_MODE
    rate_table_t rate_table[20] = {
	{0x00,1,0},//"1"
	{0x01,2,0},//"2"
	{0x02,5,1},//"5.5"
	{0x0B,6,0},//"6"
	{0x80,6,1},//"6.5"
	{0x0F,9,0},//"9"
	{0x03,11,0},//"11"
	{0x0A,12,0},//"12"
	{0x81,13,0},//"13"
	{0x0E,18,0},//"18"
	{0x82,19,1},//"19.5"
	{0x09,24,0},//"24"
	{0x83,26,0},//"26"
	{0x0D,36,0},//"36"
	{0x84,39,0},//"39"
	{0x08,48,0},//"48"
	{0x85,52,0},//"52"
	{0x0C,54,0},//"54"
	{0x86,58,1},//"58.5"
	{0x87,65,0}//"65"
	};
#else /* NMAC_1X1_MODE */
    rate_table_t rate_table[28] = {
	{0x00,1,0},//"1"
	{0x01,2,0},//"2"
	{0x02,5,1},//"5.5"
	{0x0B,6,0},//"6"
	{0x80,6,1},//"6.5"
	{0x0F,9,0},//"9"
	{0x03,11,0},//"11"
	{0x0A,12,0},//"12"
	{0x81,13,0},//"13"
	{0x88,13,0},//"13"
	{0x0E,18,0},//"18"
	{0x82,19,1},//"19.5"
	{0x09,24,0},//"24"
	{0x83,26,0},//"26"
	{0x89,26,0},//"26"
	{0x0D,36,0},//"36"
	{0x84,39,0},//"39"
	{0x8A,39,0},//"39"
	{0x08,48,0},//"48"
	{0x85,52,0},//"52"
	{0x8B,52,0},//"52"
	{0x0C,54,0},//"54"
	{0x86,58,1},//"58.5"
	{0x87,65,0},//"65"
	{0x8C,78,0},//"78"
    {0x8D,104,0},//"104"
    {0x8E,117,0},//"117"
    {0x8F,130,0}//"130"
	};
#endif /* NMAC_1X1_MODE */

/*
 * ---------------------------------------------------------------------------
 *  wext_freq_to_channel
 *  channel_to_mhz
 *
 *      These functions convert between channel number and frequency.
 *
 *  Arguments:
 *      ch      Channel number, as defined in 802.11 specs
 *      m, e    Mantissa and exponent as provided by wireless extension.
 *
 *  Returns:
 *      channel or frequency (in MHz) value
 * ---------------------------------------------------------------------------
 */
static int
wext_freq_to_channel(int m, int e)
{
    int mhz;

    mhz = m;
    while (e < 6) {
        mhz /= 10;
        e++;
    }
    while (e > 6) {
        mhz *= 10;
        e--;
    }

    if (mhz >= 5000) {
        return ((mhz - 5000) / 5);
    }

    if (mhz == 2482) {
        return 14;
    }

    if (mhz >= 2407) {
        return ((mhz - 2407) / 5);
    }

    return 0;
} /* wext_freq_to_channel() */

static inline int 
itm_chan_2_freq(int channel)
{
    if( (channel > 0) && ( channel < 14/*12*/) )
    {
        return /*2047*/2407 + (channel * 5);
    }
    //else if( channel == 14 )
    //{
        //return 2484;
    //}
    else
    {
        return -1;
    }
}

static int
channel_to_mhz(int ch,int dot11a)
{
	if(ch == 0) return 0;
	if(ch > 200) return 0;

	/* 5G */
	if(dot11a)
	{
		return (5000 + (5 * ch));
	}

	/* 2.4G */
	if(ch ==14)
	{
		return 2484;
	}

	if((ch < 14) && (ch > 0))
	{
		return (2407 + (5 * ch));
	}

	return 0;
}

#if 0
static int itm_is_cckrates_included(u8 *rate)
{	
		u32	i = 0;			

		while(rate[i]!=0)
		{		
			if  (  (((rate[i]) & 0x7f) == 2)	|| (((rate[i]) & 0x7f) == 4) ||		
			(((rate[i]) & 0x7f) == 11)  || (((rate[i]) & 0x7f) == 22) )		
			return 1;	
			i++;
		}
		
		return 0;
}
#endif

#if 0
static int itm_is_cckratesonly_included(u8 *rate)
{
	u32 i = 0;


	while(rate[i]!=0)
	{
			if  (  (((rate[i]) & 0x7f) != 2) && (((rate[i]) & 0x7f) != 4) &&
				(((rate[i]) & 0x7f) != 11)  && (((rate[i]) & 0x7f) != 22) )

			return 0;		

			i++;
	}
	
	return 1;

}
#endif

static inline int
itm_iwe_stream_add_value(struct iw_request_info *info, char *stream, char *start,
                        char *stop, struct iw_event *piwe, int len)
{
    char *new_start;

    new_start = iwe_stream_add_value(
                                     info,
                                     stream, start, stop, piwe, len);
    if (unlikely(new_start == start)) {
        return -E2BIG;
    }

    return new_start - start;
}

#ifdef IBSS_BSS_STATION_MODE
/*------------------------------------------------------------------*/
/*
 * Translate scan data returned from the card to a card independent
 * format that the Wireless Tools will understand - Jean II
 */
 //chenq mod do ap list merge logic 2013-08-28
static inline char *itm_translate_scan(struct net_device *dev,
					struct iw_request_info *info,
					char *current_ev,
					char *end_buf,
					bss_link_dscr_t *lbss)
{
	struct iw_event		iwe;		/* Temporary buffer */
	UWORD8 capabilities;
	char *			current_val;	/* For rates */
	bss_dscr_t * bss = NULL;

    //u8 bw_40MHz=0, short_GI=0;
    UWORD8 supp_rates[MAX_RATES_SUPPORTED] = {0};
    //u8 mcs_rate=0;
    UWORD8 max_rate=0;
    int i = 0;
    int r = 0;
	
	int signal, noise, snr;

	 bss = lbss->bss_curr;

	/* First entry *MUST* be the AP MAC address */
	iwe.cmd = SIOCGIWAP;
	iwe.u.ap_addr.sa_family = ARPHRD_ETHER;

	memcpy(iwe.u.ap_addr.sa_data, bss->bssid, ETH_ALEN);

	current_ev = iwe_stream_add_event(info, current_ev, end_buf,
					  &iwe, IW_EV_ADDR_LEN);
	if(!current_ev)
		return NULL;
	
	/* Other entries will be displayed in the order we give them */

	/* Add the ESSID */
   if(bss->ssid[MAX_SSID_LEN-1]) 
   	iwe.u.data.length = MAX_SSID_LEN;
   else
       iwe.u.data.length = strlen(bss->ssid);
       
	if(iwe.u.data.length > 32)
		iwe.u.data.length = 32;
	iwe.cmd = SIOCGIWESSID;
	iwe.u.data.flags = 1;
	current_ev = iwe_stream_add_point(info, current_ev, end_buf,
					  &iwe, bss->ssid);
	if(!current_ev)
		return NULL;
	PRINTK_ITMIW("bss->ssid:%s\n",bss->ssid);
	//PRINTK_ITMIW("bss->bssid: %02x:%02x:%02x:%02x:%02x:%02x\n",
		//bss->bssid[0],bss->bssid[1],bss->bssid[2],bss->bssid[3],bss->bssid[4],bss->bssid[5]);

    /* Add mode */
    /*in itm 
    BSS STA   0
    IBSS STA  1
    AP    2
    */
	iwe.cmd = SIOCGIWMODE;
	capabilities = bss->bss_type;
    switch(capabilities)
    {
        case IBSS_TYPE_STA:
            iwe.u.mode = IW_MODE_ADHOC;
            break;
        case    AP_TYPE_AP:
            iwe.u.mode = IW_MODE_MASTER;
            break;
        default:
            iwe.u.mode = 0;
            break;
    }         
    if( iwe.u.mode == 0 )
	{
	    current_ev = iwe_stream_add_event(info, current_ev, end_buf,
					  &iwe, IW_EV_UINT_LEN);
		if(!current_ev)
			return NULL;
    }
       
	/* Add frequency */
	iwe.cmd = SIOCGIWFREQ;
	iwe.u.freq.m = bss->channel;
	iwe.u.freq.m = itm_chan_2_freq(iwe.u.freq.m) * 100000;
	iwe.u.freq.e = 1;
	current_ev = iwe_stream_add_event(info, current_ev, end_buf,
					  &iwe, IW_EV_FREQ_LEN);
	if(!current_ev)
		return NULL;

	/* Add quality statistics */
	iwe.cmd = IWEVQUAL;

	signal = -(0xFF - bss->rssi);
    /* Clip range of snr */
    snr    = (bss->snr > 0) ? bss->snr : 0; /* In dB relative, from 0 - 255 */
    snr    = (snr < 255) ? snr : 255;
    noise  = signal - snr;

    /* Clip range of signal */
    signal = (signal < 63) ? signal : 63;
    signal = (signal > -192) ? signal : -192;

    /* Clip range of noise */
    noise = (noise < 63) ? noise : 63;
    noise = (noise > -192) ? noise : -192;

    /* Make u8 */
    signal = ( signal < 0 ) ? signal + 0x100 : signal;
    noise = ( noise < 0 ) ? noise + 0x100 : noise;

    iwe.u.qual.level   = (UWORD8)signal; /* -192 : 63 */
    iwe.u.qual.noise   = (UWORD8)noise;  /* -192 : 63 */
    iwe.u.qual.qual    = snr;         /* 0 : 255 */
    iwe.u.qual.updated = 0;

#if WIRELESS_EXT > 16
    iwe.u.qual.updated |= IW_QUAL_LEVEL_UPDATED |
                          IW_QUAL_NOISE_UPDATED |
                          IW_QUAL_QUAL_UPDATED;
#if WIRELESS_EXT > 18
    iwe.u.qual.updated |= IW_QUAL_DBM;
#endif
#endif
	current_ev = iwe_stream_add_event(info, current_ev, end_buf,
					  &iwe, IW_EV_QUAL_LEN);
	if(!current_ev)
		return NULL;

	/* Add encryption capability */
	iwe.cmd = SIOCGIWENCODE;
	if(bss->cap_info)
		iwe.u.data.flags = IW_ENCODE_ENABLED | IW_ENCODE_NOKEY;
	else
		iwe.u.data.flags = IW_ENCODE_DISABLED;
	iwe.u.data.length = 0;
	iwe.len = IW_EV_POINT_LEN + iwe.u.data.length;
	current_ev = iwe_stream_add_point(info, current_ev, end_buf,
					  &iwe, "");//bss->ssid);
	if(!current_ev)
		return NULL;

    /* set rates */
    i = -1;
    while( ++i < MAX_RATES_SUPPORTED )
    {
       if(bss->supp_rates[i] == 0x00)
       {
            break;
       }
       else
	   #if 1
       if(bss->supp_rates[i] == 5)
       {
            supp_rates[i] = 11;
       }
       else
       {
            supp_rates[i] = bss->supp_rates[i] * 2;
       }
	   #else
	   {
	   		supp_rates[i] = bss->supp_rates[i];
	   }
	   #endif

       if( supp_rates[i] > max_rate )
       {
            max_rate = supp_rates[i];
       }
    }

    /* Rate : stuffing multiple values in a single event require a bit
	 * more of magic - Jean II */
	current_val = current_ev + IW_EV_LCP_LEN;//iwe_stream_lcp_len(info);


    memset(&iwe,0x00,sizeof(iwe));
    iwe.cmd = SIOCGIWRATE;
    /* Those two flags are ignored... */
    iwe.u.bitrate.fixed = iwe.u.bitrate.disabled = 0;
    
    for (i = 0; i < MAX_RATES_SUPPORTED; i++) 
    {
        if (supp_rates[i] == 0) {
            break;
        }
        /* Bit rate given in 500 kb/s units (+ 0x80) */
		if(supp_rates[i] == 5)
			iwe.u.bitrate.value = 5.5 
* 500000;
		else
        	iwe.u.bitrate.value = ( supp_rates[i]/2 * 500000);
        /* Add new value to event */
        if( (r = itm_iwe_stream_add_value(
                        info, current_ev,current_val,
                        end_buf, &iwe, IW_EV_PARAM_LEN)) < 0 )
        {
            return NULL;
        }
        else
        {
            current_val +=r;
        }
    }

    /* Check if we added any rates event */
    if ((current_val - current_ev) > IW_EV_LCP_LEN) {
        current_ev = current_val;
    }
    
    
#if 0
    //parsing HT_CAP_IE
	if(bss->ht_capable > 0)
	{	
	    mcs_rate = (bss->coex_mgmt_supp) ? 1:0;
		bw_40MHz = (bss->supp_chwidth) ? 1:0;
		short_GI = (bss->sec_ch_offset) ? 1:0;
	}

	/* Add the protocol name */
	iwe.cmd = SIOCGIWNAME;
	if (itm_is_cckratesonly_included(supp_rates) == 1)		
	{
		if(bss->ht_capable > 0)
			snprintf(iwe.u.name, 16, "IEEE 802.11bn");
		else
		snprintf(iwe.u.name, 16, "IEEE 802.11b");
	}	
	else if (itm_is_cckrates_included(supp_rates) == 1)	
	{
		if(bss->ht_capable > 0 )
			snprintf(iwe.u.name, 16, "IEEE 802.11bgn");
		else
		snprintf(iwe.u.name, 16, "IEEE 802.11bg");
	}	
	else
	{
		if(bss->ht_capable > 0)
			snprintf(iwe.u.name, 16, "IEEE 802.11gn");
	    else
		    snprintf(iwe.u.name, 16, "IEEE 802.11g");
	}	

	current_ev = iwe_stream_add_event(info, current_ev,end_buf, 
                                        &iwe, IW_EV_CHAR_LEN);
	
	if(bss->ht_capable > 0)
	{
		if(mcs_rate > 0)//MCS15
		{
			max_rate = (bw_40MHz) ? ((short_GI)?300:270):((short_GI)?144:130);
			
		}
		else//default MCS7
		{
			max_rate = (bw_40MHz) ? ((short_GI)?150:135):((short_GI)?72:65);
		}

		max_rate = max_rate*2;//Mbps/2;		
	}

	iwe.cmd = SIOCGIWRATE;
	iwe.u.bitrate.fixed = iwe.u.bitrate.disabled = 0;
	iwe.u.bitrate.value = max_rate * 500000;
	current_ev = iwe_stream_add_event(info, current_ev, end_buf, 
                                        &iwe, IW_EV_PARAM_LEN);    
#endif    
       
	 /* Put WPA/RSN Information Elements into the event stream */
     //if( bss->dot11i_info > WEP128 )
    {
        char * buf;
        int len = 0;
		//UWORD16 * ie_len = (UWORD16 *)(g_wpa_rsn_ie_buf_3[bss_index]);
		UWORD16 * ie_len = (UWORD16 *)(lbss->generic_ie);

		buf = &(lbss->generic_ie[2]);
		//len = g_wpa_rsn_ie_buf_3[bss_index][0];
		len = *ie_len;
		
        if(len != 0)
        {
        	#if 0
			int i = 0;
			int j = 0;
			int cnt = 0;
		
        	while(i < len)
        	{
				printk("element %02x\n",buf[i]);
				i++;
				printk("element len %02x\n",buf[i]);
				cnt = buf[i];
				i++;
				printk("element data:");
				for(j = 0; j < cnt; j++,i++)
				{
					printk("%02x ",buf[i]);
				}
				printk("\n");
			}
			#endif
			
            memset(&iwe, 0, sizeof(iwe));
            iwe.cmd = IWEVGENIE;
            iwe.u.data.length = len;
            current_ev = iwe_stream_add_point(info, current_ev, 
                                                end_buf, &iwe,buf);
			if(!current_ev)
				return NULL;
        }    
	}

	return current_ev;
}


UWORD8 *
get_scan_ap_info(bss_dscr_t * bss_dscr,UWORD8 * srcbuf)
{
       
       /* Getting SSID */
        memcpy(bss_dscr->ssid,srcbuf, MAX_SSID_LEN);
	    //printk("****** bss_dscr->ssid:%s\n",bss_dscr->ssid);
		bss_dscr->ssid[MAX_SSID_LEN-1] = 0x00;
        srcbuf += MAX_SSID_LEN;
   
        /* Getting BSS Type Information */
        bss_dscr->bss_type = *srcbuf;
		srcbuf++;

        /* Getting Channel Information */
        bss_dscr->channel = *srcbuf;
        srcbuf++;

        /* Getting Capability Information */
		/* leon liu Stripped & BIT0 for cap_info
		 * bss_dscr->cap_info = *srcbuf & BIT0;
		 */
		bss_dscr->cap_info = *srcbuf;
		#ifdef  MAC_802_11I
        /* Getting 11i mode information */
        bss_dscr->dot11i_info = *srcbuf;
		#endif  /* MAC_802_11I */
        srcbuf++;

		#ifdef  MAC_802_11I
        /* Getting rsn cap for wpa supplicant */
		
        bss_dscr->rsn_cap[0] = *srcbuf;
		srcbuf++;
		
		bss_dscr->rsn_cap[1] = *srcbuf;
		srcbuf++;
		
		#endif  /* MAC_802_11I */

        /* Getting BSSID Information */
        memcpy(bss_dscr->bssid,srcbuf,6);
        srcbuf += 6;
		//printk("****** bss_dscr->bssid:%02x:%02x:%02x:%02x:%02x:%02x\n",
			//bss_dscr->bssid[0],bss_dscr->bssid[1],bss_dscr->bssid[2],
			//bss_dscr->bssid[3],bss_dscr->bssid[4],bss_dscr->bssid[5]);


        /* Getting RSSI Information */
        bss_dscr->rssi = *srcbuf;
        srcbuf++;

		//chenq add snr Information
		bss_dscr->snr = *srcbuf;
        srcbuf++;

        //chenq add (MAX_RATES_SUPPORTED) byte for supp rates
        /* Getting supp rates*/
        memset(bss_dscr->supp_rates,0x00,MAX_RATES_SUPPORTED + 1);
        memcpy(bss_dscr->supp_rates,srcbuf,MAX_RATES_SUPPORTED);
        srcbuf += MAX_RATES_SUPPORTED;

        /* Getting mac_802_11n Information */ 
        bss_dscr->ht_capable = *srcbuf;
        srcbuf++;
        
        bss_dscr->supp_chwidth = *srcbuf;
        srcbuf++;
            
        bss_dscr->sta_chwidth = *srcbuf;
        srcbuf++;
        
        bss_dscr->sec_ch_offset = *srcbuf;
        srcbuf++;
        
        bss_dscr->coex_mgmt_supp = *srcbuf;
        srcbuf++;

        /* Setting Reserved byte */
        srcbuf++;
        
        return srcbuf;

}
#endif

#if 0
static int
itm_set_Scan_Type(UWORD8 type)
{
	int ret = -1;
	UWORD8 * host_req  = NULL;
	UWORD8 * trout_rsp = NULL;
	UWORD16  trout_rsp_len = 0;

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if( host_req == NULL)
	{
		goto out1;
	}
	
	host_req[0] = WID_SCAN_TYPE & 0xFF;
	host_req[1] = (WID_SCAN_TYPE  & 0xFF00) >> 8;
	host_req[2] = 1;
	host_req[3] = type;

	trout_rsp = config_if_for_iw(&g_mac,host_req,WID_CHAR_CFG_LEN,'W',&trout_rsp_len);

	if( (trout_rsp == NULL) || (trout_rsp_len < 1) )
	{
		goto out2;
	}

	ret = 0;
	PRINTK_ITMIW("itm_set_Scan_Type fuc : %d \n",type);

	out2:
	pkt_mem_free(trout_rsp);
	out1:
	//kfree(trout_rsp);	
	return ret;


}
#endif 

static int
itm_set_Auth_Type(UWORD8 type)
{
	int ret = -1;
	UWORD8 * host_req  = NULL;
	UWORD16  trout_rsp_len = 0;
	int len = 0;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if( host_req == NULL)
	{
		return -ENOMEM;
	}
	
	host_req[len+0] = WID_AUTH_TYPE  & 0xFF;
	host_req[len+1] = (WID_AUTH_TYPE  & 0xFF00) >> 8;
	host_req[len+2] = 1;
	host_req[len+3] = type;
	len += WID_CHAR_CFG_LEN;

	host_req[len+0] = WID_RESET & 0xFF;
	host_req[len+1] = (WID_RESET & 0xFF00) >> 8;
	host_req[len+2] = 1;
	host_req[len+3] = 0; //DONT_RESET
	len += WID_CHAR_CFG_LEN;
	

	config_if_for_iw(&g_mac,host_req,len,'W',&trout_rsp_len);

	if( trout_rsp_len != 1 )
	{
		goto out1;
	}

	ret = 0;
	PRINTK_ITMIW("itm_set_Auth_Type fuc : %d \n",type);

	out1:
	//kfree(host_req);
	return ret;


}

static int
itm_get_Auth_Type(void)
{
	int ret = -1;
	UWORD8 * host_req  = NULL;
	UWORD8 * trout_rsp = NULL;
	UWORD16  trout_rsp_len = 0;
	UWORD8    auth_type = 0;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if( host_req == NULL)
	{
		return -ENOMEM;
	}
	
	host_req[0] = WID_AUTH_TYPE & 0xFF;
	host_req[1] = (WID_AUTH_TYPE & 0xFF00) >> 8;

	trout_rsp = config_if_for_iw(&g_mac,host_req,2,'Q',&trout_rsp_len);

	if( trout_rsp == NULL )
	{
		goto out1;
	}

	trout_rsp_len -= MSG_HDR_LEN;
	if( get_trour_rsp_data(&auth_type,(UWORD16)sizeof(UWORD8),
					       &trout_rsp[MSG_HDR_LEN],trout_rsp_len,
					       WID_CHAR) == 0 )
	{
		goto out2;
	}

	ret = auth_type & 0xFF;
	PRINTK_ITMIW("itm_get_Auth_Type fuc : %d \n",auth_type);

    out2:
	pkt_mem_free(trout_rsp);
	out1:
	//kfree(host_req);
	return ret;
}


static int
itm_set_Encryption_Type(UWORD8 type)
{
	int ret = -1;
	UWORD8 * host_req  = NULL;
	UWORD16  trout_rsp_len = 0;
	int len = 0;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if( host_req == NULL)
	{
		return -ENOMEM;
	}
	
	host_req[0] = WID_11I_MODE & 0xFF;
	host_req[1] = (WID_11I_MODE & 0xFF00) >> 8;
	host_req[2] = 1;
	host_req[3] = type;
	len += WID_CHAR_CFG_LEN;

	host_req[len+0] = WID_RESET & 0xFF;
	host_req[len+1] = (WID_RESET & 0xFF00) >> 8;
	host_req[len+2] = 1;
	host_req[len+3] = 0; //DONT_RESET
	len += WID_CHAR_CFG_LEN;

	config_if_for_iw(&g_mac,host_req,len,'W',&trout_rsp_len);

	if( trout_rsp_len != 1 )
	{
		goto out1;
	}

	g_psk_value[39] = 0;
	
	ret = 0;
	PRINTK_ITMIW("itm_set_Encryption_Type fuc : %02x \n",type);
	out1:
	//kfree(host_req);
	return ret;


}

static int
itm_get_Encryption_Type(void)
{
	int ret = -1;
	UWORD8 * host_req  = NULL;
	UWORD8 * trout_rsp = NULL;
	UWORD16  trout_rsp_len = 0;
	UWORD8    encryp_type = 0;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if( host_req == NULL)
	{
		return -ENOMEM;
	}
	
	host_req[0] = WID_11I_MODE & 0xFF;
	host_req[1] = (WID_11I_MODE & 0xFF00) >> 8;

	trout_rsp = config_if_for_iw(&g_mac,host_req,2,'Q',&trout_rsp_len);

	if( trout_rsp == NULL )
	{
		goto out1;
	}

	trout_rsp_len -= MSG_HDR_LEN;
	if( get_trour_rsp_data(&encryp_type,(UWORD16)sizeof(UWORD8),
					       &trout_rsp[MSG_HDR_LEN],trout_rsp_len,
					       WID_CHAR) == 0 )
	{
		goto out2;
	}

	ret = encryp_type & 0xFF;
	PRINTK_ITMIW("itm_get_Encryption_Type fuc : %02x \n",encryp_type);

	out2:
	pkt_mem_free(trout_rsp);
	out1:
	//kfree(host_req);
	return ret;
}

		int
itm_set_wep_key_index(int index)
{
	int ret = -1;
	UWORD8 * host_req  = NULL;
	UWORD16  trout_rsp_len = 0;
	int len = 0;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	PRINTK_ITMIW("itm_set_wep_key_index fuc\n");

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if( host_req == NULL )
	{
		return -ENOMEM;
	}
	
	host_req[0] = WID_KEY_ID & 0xFF;
	host_req[1] = (WID_KEY_ID & 0xFF00) >> 8;
	host_req[2] = 1;
	host_req[3] = index & 0xFF;
	len +=  WID_CHAR_CFG_LEN;

	host_req[len+0] = WID_RESET & 0xFF;
	host_req[len+1] = (WID_RESET & 0xFF00) >> 8;
	host_req[len+2] = 1;
	host_req[len+3] = 0; //DONT_RESET
	len += WID_CHAR_CFG_LEN;

	config_if_for_iw(&g_mac,host_req,len,'W',&trout_rsp_len);

	if( trout_rsp_len != 1 )
	{
		goto out1;
	}

	ret = 0;
	
	out1:
	PRINTK_ITMIW("itm_set_wep_key_index fuc : %d \n", ( ret == 0) ? index : ret);
	//kfree(host_req);
	return ret;


}

static int
itm_get_wep_key_index(void)
{
	int ret = -1;
	UWORD8 * host_req  = NULL;
	UWORD8 * trout_rsp = NULL;
	UWORD16  trout_rsp_len = 0;
	UWORD8    key_index = 0;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if( host_req == NULL)
	{
		return -ENOMEM;
	}
	
	host_req[0] = WID_KEY_ID & 0xFF;
	host_req[1] = (WID_KEY_ID & 0xFF00) >> 8;

	trout_rsp = config_if_for_iw(&g_mac,host_req,2,'Q',&trout_rsp_len);

	if( trout_rsp == NULL )
	{
		goto out1;
	}

	trout_rsp_len -= MSG_HDR_LEN;
	if( get_trour_rsp_data(&key_index,(UWORD16)sizeof(UWORD8),
					       &trout_rsp[MSG_HDR_LEN],trout_rsp_len,
					       WID_CHAR) == 0 )
	{
		goto out2;
	}

	ret = key_index & 0xFF;
	PRINTK_ITMIW("itm_get_wep_key_index fuc : %d \n",ret);

	out2:
	pkt_mem_free(trout_rsp);
	out1:
	//kfree(host_req);
	return ret;
}

#if 0
static int
itm_set_wep_key_value(UWORD8 * pkey,UWORD8 keylen)
{
	int ret = -1;
	UWORD8 * host_req  = NULL;
	UWORD16  trout_rsp_len = 0;
	UWORD8 * ppkey = pkey;
	int i = 0;
	int len = 0;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if( host_req == NULL)
	{
		return -ENOMEM;
	}
	
	host_req[0] = WID_WEP_KEY_VALUE & 0xFF;
	host_req[1] = (WID_WEP_KEY_VALUE & 0xFF00) >> 8;
	host_req[2] = keylen * 2;

	memcpy(&host_req[3],pkey,keylen);
	for( i = 0 ; i < keylen ; i++)
	{
		host_req[3+2*i] = hex_2_char((UWORD8)( (*ppkey & 0xF0) >> 4));
		host_req[3+2*i + 1] = hex_2_char((UWORD8)( *ppkey & 0x0F));

		ppkey++;
	}

	len += WID_HEAD_LEN+keylen*2;

	host_req[len+0] = WID_RESET & 0xFF;
	host_req[len+1] = (WID_RESET & 0xFF00) >> 8;
	host_req[len+2] = 1;
	host_req[len+3] = 0; //DONT_RESET
	len += WID_CHAR_CFG_LEN;

	config_if_for_iw(&g_mac,host_req,len,'W',&trout_rsp_len);

	if( trout_rsp_len != 1 )
	{
		goto out1;
	}

	ret = keylen;
	PRINTK_ITMIW("itm_set_wep_key_value fuc : %s \n",pkey);

	out1:
	//kfree(host_req);
	return ret;


}
#endif

static int
itm_get_wep_key_value(UWORD8 * pkey,UWORD8 keylen)
{
	int ret = -1;
	UWORD8 * host_req  = NULL;
	UWORD8 * trout_rsp = NULL;
	UWORD16  trout_rsp_len = 0;
	int len = 0;
	UWORD8   key_value[LARGE_KEY_SIZE * 2] = {0};

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if( host_req == NULL)
	{
		return -ENOMEM;
	}
	
	host_req[0] = WID_WEP_KEY_VALUE & 0xFF;
	host_req[1] = (WID_WEP_KEY_VALUE & 0xFF00) >> 8;

	trout_rsp = config_if_for_iw(&g_mac,host_req,2,'Q',&trout_rsp_len);

	if( trout_rsp == NULL )
	{
		goto out1;
	}

	trout_rsp_len -= MSG_HDR_LEN;
	if( (len = get_trour_rsp_data(key_value,(UWORD16)(sizeof(key_value)),
					       &trout_rsp[MSG_HDR_LEN],trout_rsp_len,
					       WID_STR)) == 0 )
	{
		goto out2;
	}

	ret = len/2;
	str_2_hex(key_value,ret);
	memcpy(pkey,key_value,ret);
	
	PRINTK_ITMIW("itm_get_wep_key_value fuc : %s \n",pkey);

	out2:
	pkt_mem_free(trout_rsp);
	out1:
	//kfree(host_req);
	return ret;
}

//zhou mod
//static int
int
itm_set_power_type(UWORD8 type)
{
	int ret = -1;
	UWORD8 * host_req  = NULL;
	UWORD16  trout_rsp_len = 0;
	int len = 0;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_NO_ACCESS);

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if( host_req == NULL)
	{
		return -ENOMEM;
	}
	
	host_req[len+0] = WID_POWER_MANAGEMENT & 0xFF;
	host_req[len+1] = (WID_POWER_MANAGEMENT & 0xFF00) >> 8;
	host_req[len+2] = 1;
	host_req[len+3] = type;
	len += WID_CHAR_CFG_LEN;

	config_if_for_iw(&g_mac,host_req,len,'W',&trout_rsp_len);

	if( trout_rsp_len != 1 )
	{
		goto out1;
	}

	ret = 0;
	PRINTK_ITMIW("itm_set_power_type fuc : %d \n",type & 0xFF);

	out1:
	//kfree(host_req);
	return ret;

}

static int
itm_get_power_type(void)
{
	int ret = -1;
	UWORD8 * host_req  = NULL;
	UWORD8 * trout_rsp = NULL;
	UWORD16  trout_rsp_len = 0;
	UWORD8    power_type = 0;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_NO_ACCESS);

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if( host_req == NULL)
	{
		return -ENOMEM;
	}
	
	host_req[0] = WID_POWER_MANAGEMENT & 0xFF;
	host_req[1] = (WID_POWER_MANAGEMENT & 0xFF00) >> 8;

	trout_rsp = config_if_for_iw(&g_mac,host_req,2,'Q',&trout_rsp_len);

	if( trout_rsp == NULL )
	{
		goto out1;
	}

	trout_rsp_len -= MSG_HDR_LEN;
	if( get_trour_rsp_data(&power_type,(UWORD16)(sizeof(power_type)),
					       &trout_rsp[MSG_HDR_LEN],trout_rsp_len,
					       WID_CHAR) == 0 )
	{
		goto out2;
	}
	
	PRINTK_ITMIW("itm_get_power_type fuc : %d \n",power_type);
	ret = power_type & 0xFF;
	
	out2:
	pkt_mem_free(trout_rsp);
	out1:
	//kfree(host_req);
	return ret;
}

#if 0
static int
itm_set_power_period(UWORD8 period)
{
	int ret = -1;
	UWORD8 * host_req  = NULL;
	UWORD16  trout_rsp_len = 0;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if( host_req == NULL)
	{
		return -ENOMEM;
	}
	
	host_req[0] = WID_LISTEN_INTERVAL & 0xFF;
	host_req[1] = (WID_LISTEN_INTERVAL & 0xFF00) >> 8;
	host_req[2] = 1;
	host_req[3] = period;

	config_if_for_iw(&g_mac,host_req,WID_CHAR_CFG_LEN,'W',&trout_rsp_len);

	if( trout_rsp_len != 1 )
	{
		goto out1;
	}

	ret = 0;
	PRINTK_ITMIW("itm_set_power_period fuc : %d \n",period & 0xFF);

	out1:
	//kfree(host_req);
	return ret;


}
#endif

static int
itm_get_power_period(void)
{
	int ret = -1;
	UWORD8 * host_req  = NULL;
	UWORD8 * trout_rsp = NULL;
	UWORD16  trout_rsp_len = 0;
	UWORD8    power_period = 0;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if( host_req == NULL)
	{
		return -ENOMEM;
	}
	
	host_req[0] = WID_LISTEN_INTERVAL & 0xFF;
	host_req[1] = (WID_LISTEN_INTERVAL & 0xFF00) >> 8;

	trout_rsp = config_if_for_iw(&g_mac,host_req,2,'Q',&trout_rsp_len);

	if( trout_rsp == NULL )
	{
		goto out1;
	}

	trout_rsp_len -= MSG_HDR_LEN;
	if( get_trour_rsp_data(&power_period,(UWORD16)(sizeof(power_period)),
					       &trout_rsp[MSG_HDR_LEN],trout_rsp_len,
					       WID_CHAR) == 0 )
	{
		goto out2;
	}
	
	PRINTK_ITMIW("itm_get_power_period fuc : %d \n",power_period);
	ret = power_period & 0xFF;
	
	out2:
	pkt_mem_free(trout_rsp);
	out1:
	//kfree(host_req);	
	return ret;
}

#if 0
static int
itm_set_BSS_type(UWORD8 type)
{
    /* in itm,we don't set */
    return 0;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);
#if 0    
	int ret = -1;
	UWORD8 * host_req  = NULL;
	UWORD16  trout_rsp_len = 0;

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if( host_req == NULL)
	{
		return -ENOMEM;
	}
	
	host_req[0] = WID_BSS_TYPE & 0xFF;
	host_req[1] = (WID_BSS_TYPE & 0xFF00) >> 8;
	host_req[2] = 1;
	host_req[3] = type;

	trout_rsp = config_if_for_iw(&g_mac,host_req,WID_CHAR_CFG_LEN,'W',&trout_rsp_len);

	if( trout_rsp_len != 1 )
	{
		goto out1;
	}

	ret = 0;
	PRINTK_ITMIW("itm_set_BSS_type fuc : %d \n",type & 0xFF);

	out1:
	//kfree(host_req);
	return ret;
#endif

}
#endif

static int
itm_get_BSS_type(void)
{
	int ret = -1;
	UWORD8 * host_req  = NULL;
	UWORD8 * trout_rsp = NULL;
	UWORD16  trout_rsp_len = 0;
	UWORD8    bss_type = 0;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if( host_req == NULL)
	{
		return -ENOMEM;
	}
	
	host_req[0] = WID_BSS_TYPE & 0xFF;
	host_req[1] = (WID_BSS_TYPE & 0xFF00) >> 8;

	trout_rsp = config_if_for_iw(&g_mac,host_req,2,'Q',&trout_rsp_len);

	if( trout_rsp == NULL )
	{
		goto out1;
	}

	trout_rsp_len -= MSG_HDR_LEN;
	if( get_trour_rsp_data(&bss_type,(UWORD16)(sizeof(bss_type)),
					       &trout_rsp[MSG_HDR_LEN],trout_rsp_len,
					       WID_CHAR) == 0 )
	{
		goto out2;
	}
	
	PRINTK_ITMIW("itm_get_BSS_type fuc : %d \n",bss_type);
	ret = bss_type & 0xFF;
	
	out2:
	pkt_mem_free(trout_rsp);
	out1:
	//kfree(host_req);	
	return ret;
}

#if 0
static int
itm_set_device_mode(UWORD8 mode)
{
	int ret = -1;
	UWORD8 * host_req  = NULL;
	UWORD16  trout_rsp_len = 0;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if( host_req == NULL)
	{
		return -ENOMEM;
	}
	
	host_req[0] = WID_BSS_TYPE & 0xFF;
	host_req[1] = (WID_BSS_TYPE & 0xFF00) >> 8;
	host_req[2] = 1;
	host_req[3] = mode;

	trout_rsp = config_if_for_iw(&g_mac,host_req,WID_CHAR_CFG_LEN,'W',&trout_rsp_len);

	if( trout_rsp_len != 1 )
	{
		goto out1;
	}

	ret = 0;
	PRINTK_ITMIW("itm_set_device_mode fuc : %d \n",mode & 0xFF);

	out1:
	//kfree(host_req);
	return ret;

}
#endif

static int
itm_get_device_mode(void)
{
	int ret = -1;
	UWORD8 * host_req  = NULL;
	UWORD8 * trout_rsp = NULL;
	UWORD16  trout_rsp_len = 0;
	UWORD8    mode = 0;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if( host_req == NULL)
	{
		return -ENOMEM;
	}
	
	host_req[0] = WID_DEVICE_MODE & 0xFF;
	host_req[1] = (WID_DEVICE_MODE & 0xFF00) >> 8;

	trout_rsp = config_if_for_iw(&g_mac,host_req,2,'Q',&trout_rsp_len);

	if( trout_rsp == NULL )
	{
		goto out1;
	}

	trout_rsp_len -= MSG_HDR_LEN;
	if( get_trour_rsp_data(&mode,(UWORD16)(sizeof(mode)),
					       &trout_rsp[MSG_HDR_LEN],trout_rsp_len,
					       WID_CHAR) == 0 )
	{
		goto out2;
	}
	
	PRINTK_ITMIW("itm_get_device_mode fuc : %d \n",mode);
	ret = mode & 0xFF;
	
	out2:
	pkt_mem_free(trout_rsp);
	out1:
	//kfree(host_req);
	return ret;
}

#if 1
/*leon liu added for setting caculated PSK*/
static int itm_set_psk(UWORD8 *psk, UWORD8 psklen)
{
		int ret = 0;
		UWORD8 * host_req  = NULL;
		UWORD16  trout_rsp_len = 0;
		int i=0;
		int len = 0;

		CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

		PRINTK_ITMIW("itm_set_psk for caculated PSK fuc \n");

#ifdef	BSS_ACCESS_POINT_MODE
		phostap_conf_t conf = hostap_conf_load(NULL);
		if(conf){
			if(hexstr2bin(conf->wpa_psk,saved_psk,conf->len/2)){
				printk("[hostap_conf]	hexstr2bin error\n");
			}
			psklen = conf->len/2;
			hostap_conf_destry(conf);
			psk = saved_psk;
		}else{
			printk("hosta_conf_load error\n");
		}
#endif

		host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

		if( host_req == NULL)
		{
				return -ENOMEM;
		}

		host_req[len+0] = WID_11I_PSK_VALUE  & 0xFF;
		host_req[len+1] = (WID_11I_PSK_VALUE  & 0xFF00) >> 8;
		host_req[len+2] = psklen;
		memcpy(&host_req[len+3],psk,psklen);
		len += psklen + WID_HEAD_LEN;

		host_req[len+0] = WID_RESET & 0xFF;
		host_req[len+1] = (WID_RESET & 0xFF00) >> 8;
		host_req[len+2] = 1;
		host_req[len+3] = 0; //DONT_RESET
		len += WID_CHAR_CFG_LEN;

		config_if_for_iw(&g_mac,host_req,len,'W',&trout_rsp_len);

		if( trout_rsp_len != 1 )
		{
				ret = -EINVAL;
				goto out1;
		}

		for(i=0;i<psklen;i++)
		{
				PRINTK_ITMIW("0x%02x ",psk[i]);
		}
		PRINTK_ITMIW("\n");

out1:
		//kfree(host_req);
		return ret;

}
static int
itm_set_psk_value(UWORD8 * psk,UWORD8 psklen)
{
	int ret = 0;
	UWORD8 * host_req  = NULL;
	UWORD16  trout_rsp_len = 0;
    int i=0;
	int len = 0;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	PRINTK_ITMIW("itm_set_psk_value fuc \n");

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if( host_req == NULL)
	{
		return -ENOMEM;
	}
	
	host_req[len+0] = WID_11I_PSK  & 0xFF;
	host_req[len+1] = (WID_11I_PSK  & 0xFF00) >> 8;
	host_req[len+2] = psklen;
	memcpy(&host_req[len+3],psk,psklen);
	len += psklen + WID_HEAD_LEN;

	host_req[len+0] = WID_RESET & 0xFF;
	host_req[len+1] = (WID_RESET & 0xFF00) >> 8;
	host_req[len+2] = 1;
	host_req[len+3] = 0; //DONT_RESET
	len += WID_CHAR_CFG_LEN;

	config_if_for_iw(&g_mac,host_req,len,'W',&trout_rsp_len);

	if( trout_rsp_len != 1 )
	{
		ret = -EINVAL;
		goto out1;
	}

    PRINTK_ITMIW("itm_set_psk_value fuc set psk:");
    for(i=0;i<psklen;i++)
    {
        PRINTK_ITMIW("0x%02x ",psk[i]);
    }
    PRINTK_ITMIW("\n");

	out1:
	//kfree(host_req);
	return ret;
}
#endif


static int itm_get_bssid(UWORD8 * bssid);

int itm_add_wpa_wpa2_value(UWORD8 * s_addr,
							   UWORD8 * key_data,UWORD8 key_len,UWORD8 key_type,
							   UWORD8 key_id,UWORD8 * key_rsc)
{
	int ret = 0;
	UWORD8 * host_req  = NULL;
	UWORD16  trout_rsp_len = 0;
    //int i=0;
	int len = 0;
	static BOOL_T start_pm_timer = BFALSE;
	UWORD8 tmp_32byte_all0[32] = {0};

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	PRINTK_ITMIW("itm_add_wpa_wpa2_value fuc %s\n",(!key_type) ? "GTK" : "PTK");

	if( itm_get_bssid(s_addr) < 0 )
	{
		return -1;
	}

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if( host_req == NULL)
	{
		return -ENOMEM;
	}

	if(key_type == 1)
	{
	/*----------------------------------------*/
    /*    STA Addr  | KeyLength |   Key       */
    /*----------------------------------------*/
    /*       6      |     1     |  KeyLength  */
    /*----------------------------------------*/

    /*---------------------------------------------------------*/
    /*                      key                                */
    /*---------------------------------------------------------*/
    /* Temporal Key    | Rx Micheal Key    |   Tx Micheal Key  */
    /*---------------------------------------------------------*/
    /*    16 bytes     |      8 bytes      |       8 bytes     */
    /*---------------------------------------------------------*/

		host_req[len++] = WID_ADD_PTK  & 0xFF;
		host_req[len++] = (WID_ADD_PTK  & 0xFF00) >> 8;
		host_req[len++] = 6 + 1 + key_len;
		
		memcpy(&host_req[len],s_addr,6);
		len += 6;

		host_req[len++] = key_len;

		memcpy(&host_req[len],key_data,key_len);
		len += key_len;
		
	}
	else if(key_type == 0)
	{
	/*---------------------------------------------------------*/
    /*    STA Addr  | KeyRSC | KeyID | KeyLength |   Key       */
    /*---------------------------------------------------------*/
    /*       6      |   8    |   1   |     1     |  KeyLength  */
    /*---------------------------------------------------------*/

    /*-------------------------------------*/
    /*                      key            */
    /*-------------------------------------*/
    /* Temporal Key    | Rx Micheal Key    */
    /*-------------------------------------*/
    /*    16 bytes     |      8 bytes      */
    /*-------------------------------------*/
	
		host_req[len++] = WID_ADD_RX_GTK  & 0xFF;
		host_req[len++] = (WID_ADD_RX_GTK  & 0xFF00) >> 8;
		host_req[len++] = 6 + 8 + 1 + 1 + key_len;

		memcpy(&host_req[len],s_addr,6);
		len += 6;

		memcpy(&host_req[len],key_rsc,8);
		len += 8;
		
		host_req[len++] = key_id;

		host_req[len++] = key_len;

		memcpy(&host_req[len],key_data,key_len);
		len += key_len;

	}
	else
	{
		return -1;
	}

	if(key_len < 16)
	{
		ret = -EINVAL;
		goto out1;
	}

	config_if_for_iw(&g_mac,host_req,len,'W',&trout_rsp_len);

	if( trout_rsp_len != 1 )
	{
		ret = -EINVAL;
		goto out1;
	}

    PRINTK_ITMIW("itm_add_wpa_wpa2_value fuc end \n");
	#if 0
	for(i=0;i<psklen;i++)
    {
        PRINTK_ITMIW("0x%02x ",psk[i]);
    }
    PRINTK_ITMIW("\n");
	#endif

	//chenq add for start_activity_timer(); 2013-01-07
	#ifdef IBSS_BSS_STATION_MODE
	if(!key_type)
	{
		if(start_pm_timer == BFALSE)
		{
			start_activity_timer();
			start_pm_timer = BTRUE;
			//chenq add for ext wpa_wpa2_hs 2013-04-16
			g_wpa_wpa2_hs_complete = BTRUE;
		}	
	}
	else
	{
		start_pm_timer = BFALSE;

		//chenq add for ext wpa_wpa2_rehs 2013-12-05
		if(g_wpa_wpa2_rehs == BFALSE)
		{
			if(memcmp(tmp_32byte_all0,g_wpa_wpa2_ptk_key,32) == 0)
			{//first fime set ptk
				memcpy(g_wpa_wpa2_ptk_key,key_data,key_len);
			}
			else if(memcmp(key_data,g_wpa_wpa2_ptk_key,key_len) != 0)
			{//second time set pkt, rehs
				g_wpa_wpa2_rehs = BTRUE;
			}
			//we will not care other time,after set g_wpa_wpa2_rehs BTRUE
		}
	}
	#endif
	
	out1:
	//kfree(host_req);
	return ret;
}


#if 0 //BSS_ACCESS_POINT_MODE

static int
itm_set_channel(int channel_id)
{
	int ret = 0;
	UWORD8 * host_req  = NULL;
	UWORD16  trout_rsp_len = 0;
	int len = 0;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	PRINTK_ITMIW("itm_set_channel fuc\n");

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);
	if( host_req == NULL )
	{
		return -ENOMEM;
	}

	host_req[len+0] = WID_PRIMARY_CHANNEL & 0xFF;
	host_req[len+1] = (WID_PRIMARY_CHANNEL & 0xFF00) >> 8;
	host_req[len+2] = 1;
	host_req[len+3] = channel_id & 0xFF;
	len += WID_CHAR_CFG_LEN;

	host_req[len+0] = WID_RESET & 0xFF;
	host_req[len+1] = (WID_RESET & 0xFF00) >> 8;
	host_req[len+2] = 1;
	host_req[len+3] = 0; //DONT_RESET
	len += WID_CHAR_CFG_LEN;
		
	config_if_for_iw(&g_mac,host_req,len,
								'W',&trout_rsp_len);

    if( trout_rsp_len != 1 )
    {
    	ret = -EINVAL;
    	goto out1;
    }

    PRINTK_ITMIW("itm_siwfreq fuc set channel:%d\n",channel_id);
	   
	out1:
	//kfree(host_req);	
	return ret;
}

#if 0
static int
itm_set_preamble(int preamble_type)
{
	int ret = 0;
	UWORD8 * host_req  = NULL;
	UWORD16  trout_rsp_len = 0;
	int len = 0;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	PRINTK_ITMIW("itm_set_preamble fuc\n");

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);
	if( host_req == NULL )
	{
		return -ENOMEM;
	}

	host_req[len+0] = WID_PREAMBLE & 0xFF;
	host_req[len+1] = (WID_PREAMBLE & 0xFF00) >> 8;
	host_req[len+2] = 1;
	host_req[len+3] = preamble_type & 0xFF;
	len += WID_CHAR_CFG_LEN;

	host_req[len+0] = WID_RESET & 0xFF;
	host_req[len+1] = (WID_RESET & 0xFF00) >> 8;
	host_req[len+2] = 1;
	host_req[len+3] = 0; //DONT_RESET
	len += WID_CHAR_CFG_LEN;
		
	config_if_for_iw(&g_mac,host_req,len,
								'W',&trout_rsp_len);

    if( trout_rsp_len != 1 )
    {
    	ret = -EINVAL;
    	goto out1;
    }

    PRINTK_ITMIW("itm_set_preamble fuc set preamble_type:%d\n",preamble_type);
	   
	out1:
	//kfree(host_req);
	return ret;
}
#endif

static int
itm_set_beaconInterval(int beaconInterval)
{
	int ret = 0;
	UWORD8 * host_req  = NULL;
	UWORD16  trout_rsp_len = 0;
	int len = 0;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	PRINTK_ITMIW("itm_set_beaconInterval fuc\n");

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);
	if( host_req == NULL )
	{
		return -ENOMEM;
	}

    host_req[len+0] = WID_BEACON_INTERVAL & 0xFF;
    host_req[len+1] = (WID_BEACON_INTERVAL & 0xFF00) >> 8;
	host_req[len+2] = (WID_SHORT + 1) & 0xFF;

	host_req[3] = beaconInterval & 0xFF;
	host_req[4] = (beaconInterval & 0xFF00) >> 8;
	len += WID_SHORT_CFG_LEN;

	host_req[len+0] = WID_RESET & 0xFF;
	host_req[len+1] = (WID_RESET & 0xFF00) >> 8;
	host_req[len+2] = 1;
	host_req[len+3] = 0; //DONT_RESET
	len += WID_CHAR_CFG_LEN;

	config_if_for_iw(&g_mac,host_req,len,
							   'W',&trout_rsp_len);

    if( trout_rsp_len != 1 )
    {
    	ret = -EINVAL;
    	goto out1;
    }

    PRINTK_ITMIW("itm_set_beaconInterval fuc set beaconInterval:%d\n",beaconInterval);
	   
	out1:
	//kfree(host_req);
	return ret;
}


static int
itm_set_dtim_period(int dtim_period)
{
	int ret = 0;
	UWORD8 * host_req  = NULL;
	UWORD16  trout_rsp_len = 0;
	int len = 0;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	PRINTK_ITMIW("itm_set_dtim_period fuc\n");

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);
	if( host_req == NULL )
	{
		return -ENOMEM;
	}

	host_req[len+0] = WID_DTIM_PERIOD & 0xFF;
	host_req[len+1] = (WID_DTIM_PERIOD & 0xFF00) >> 8;
	host_req[len+2] = 1;
	host_req[len+3] = dtim_period & 0xFF;
	len += WID_CHAR_CFG_LEN;

	host_req[len+0] = WID_RESET & 0xFF;
	host_req[len+1] = (WID_RESET & 0xFF00) >> 8;
	host_req[len+2] = 1;
	host_req[len+3] = 0; //DONT_RESET
	len += WID_CHAR_CFG_LEN;

	
		
	config_if_for_iw(&g_mac,host_req,len,
								'W',&trout_rsp_len);

    if( trout_rsp_len != 1 )
    {
    	ret = -EINVAL;
    	goto out1;
    }

    PRINTK_ITMIW("itm_set_dtim_period fuc set itm_set_dtim_period:%d\n",dtim_period);
	   
	out1:
	//kfree(host_req);	
	return ret;
}

static int
itm_set_wmmEnable(int wmm_enable)
{
	int ret = 0;
	UWORD8 * host_req  = NULL;
	UWORD16  trout_rsp_len = 0;
	int len = 0;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	PRINTK_ITMIW("itm_set_wmmEnable fuc\n");
	
	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);
	if( host_req == NULL )
	{
		return -ENOMEM;
	}
	
	host_req[len+0] = WID_QOS_ENABLE & 0xFF;
	host_req[len+1] = (WID_QOS_ENABLE & 0xFF00) >> 8;
	host_req[len+2] = 1;
	host_req[len+3] = wmm_enable & 0xFF;
	len += WID_CHAR_CFG_LEN;

	host_req[len+0] = WID_RESET & 0xFF;
	host_req[len+1] = (WID_RESET & 0xFF00) >> 8;
	host_req[len+2] = 1;
	host_req[len+3] = 0; //DONT_RESET
	len += WID_CHAR_CFG_LEN;
		
	config_if_for_iw(&g_mac,host_req,len,
								'W',&trout_rsp_len);

    if( trout_rsp_len != 1 )
    {
    	ret = -EINVAL;
    	goto out1;
    }

    PRINTK_ITMIW("itm_set_wmmEnable fuc set itm_set_wmmEnable:%d\n",wmm_enable);
	   
	out1:
	//kfree(host_req);
	return ret;
}

static int
itm_set_11g_op_mode(int op_mode)
{
	int ret = 0;
	UWORD8 * host_req  = NULL;
	UWORD16  trout_rsp_len = 0;
	int len = 0;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	PRINTK_ITMIW("itm_set_11g_op_mode fuc\n");

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);
	if( host_req == NULL )
	{
		return -ENOMEM;
	}

	host_req[len+0] = WID_11G_OPERATING_MODE & 0xFF;
	host_req[len+1] = (WID_11G_OPERATING_MODE & 0xFF00) >> 8;
	host_req[len+2] = 1;
	host_req[len+3] = op_mode & 0xFF;
	len += WID_CHAR_CFG_LEN;

	host_req[len+0] = WID_RESET & 0xFF;
	host_req[len+1] = (WID_RESET & 0xFF00) >> 8;
	host_req[len+2] = 1;
	host_req[len+3] = 0; //DONT_RESET
	len += WID_CHAR_CFG_LEN;
		
	config_if_for_iw(&g_mac,host_req,len,
								'W',&trout_rsp_len);

    if( trout_rsp_len != 1 )
    {
    	ret = -EINVAL;
    	goto out1;
    }

    PRINTK_ITMIW("itm_set_11g_op_mode fuc set itm_set_11g_op_mode:%d\n",op_mode);
	   
	out1:
	//kfree(host_req);	
	return ret;
}

static int
set_11n_enable(int n_enable)
{
	int ret = 0;
	UWORD8 * host_req  = NULL;
	UWORD16  trout_rsp_len = 0;
	int len = 0;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	PRINTK_ITMIW("set_11n_enable fuc\n");

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);
	if( host_req == NULL )
	{
		return -ENOMEM;
	}

	host_req[len+0] = WID_11N_ENABLE & 0xFF;
	host_req[len+1] = (WID_11N_ENABLE & 0xFF00) >> 8;
	host_req[len+2] = 1;
	host_req[len+3] = n_enable & 0xFF;
	len += WID_CHAR_CFG_LEN;

	host_req[len+0] = WID_RESET & 0xFF;
	host_req[len+1] = (WID_RESET & 0xFF00) >> 8;
	host_req[len+2] = 1;
	host_req[len+3] = 0; //DONT_RESET
	len += WID_CHAR_CFG_LEN;
		
	config_if_for_iw(&g_mac,host_req,len,
								'W',&trout_rsp_len);

    if( trout_rsp_len != 1 )
    {
    	ret = -EINVAL;
    	goto out1;
    }

    PRINTK_ITMIW("set_11n_enable fuc set set_11n_enable:%d\n",n_enable);
	   
	out1:
	//kfree(host_req);	
	return ret;
}
#endif

static int 
itm_set_bssid(UWORD8 * addr)
{
	int ret = 0;
	int len = 0;
	UWORD8 * host_req  = NULL;
	UWORD16  trout_rsp_len = 0;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if(host_req == NULL)
	{
		return -ENOMEM;
	}

	host_req[len++] = WID_BSSID & 0xFF;
	host_req[len++] = (WID_BSSID & 0xFF00) >> 8;
	host_req[len++] = 6;
	memcpy(&host_req[len],addr,6);
	len += 6;

	host_req[len] = WID_RESET & 0xFF;
	host_req[len+1] = (WID_RESET & 0xFF00) >> 8;
	host_req[len+2] = 1;
	host_req[len+3] = 0;//DONT_RESET
	len += WID_CHAR_CFG_LEN;

	config_if_for_iw(&g_mac,host_req,len,'W',&trout_rsp_len);

	if( trout_rsp_len != 1 )
	{
		ret = -EINVAL;
		goto out1;
	}

    PRINTK_ITMIW("itm_set_bssid fuc set mac:%02x:%02x:%02x:%02x:%02x:%02x \n",
                    addr[0],addr[1],addr[2],
                    addr[3],addr[4],addr[5]);
	   
	out1:
	//kfree(host_req);
	return ret;
}

extern UWORD8 itm_get_max_ch_no(void);

static int 
itm_set_essid(UWORD8 * essid,int essid_len)
{
	int ret = 0;
	UWORD8 * host_req  = NULL;
	UWORD16  trout_rsp_len = 0;
	//int index = 0;
	int len = 0;
	static UWORD8 ap_name[MAX_SSID_LEN] = {0};

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if(host_req == NULL)
	{
		return -ENOMEM;
	}	

#ifdef IBSS_BSS_STATION_MODE
	PRINTK_ITMIW("domain: get_prim_chnl_num() = %d, itm_get_max_ch_no() = %d\n",
		get_prim_chnl_num(),itm_get_max_ch_no());
	//caisf add, 20130505
    /* Do not connect to the ap which channel is out of max channel number*/
	/* Config essid will earlier than config primary channel! */
    if(get_prim_chnl_num() > itm_get_max_ch_no())
    {
		ret = -EINVAL;
		PRINTK_ITMIW("This ap's(%s) channel is out of max channel number range!\n",
			essid);
		goto out1;
    }
#endif

    #ifdef TROUT_WIFI_NPI
	g_npi_scan_flag = 1;
    printk("%s-%d, g_npi_scan_flag = %d\n",__FUNCTION__,__LINE__,g_npi_scan_flag);
    #endif

	if( (essid != NULL) && (essid_len > 0) )
	{
		if( (essid_len) > ( MAX_SSID_LEN - 1) ) 
			essid_len = MAX_SSID_LEN - 1;

		host_req[len] = WID_SSID & 0xFF;
		host_req[len+1] = (WID_SSID & 0xFF00) >> 8;
		host_req[len+2] = essid_len & 0xFF;
		memcpy(&host_req[len+3],essid,essid_len);
		len += WID_HEAD_LEN + essid_len;

		#if 1
		/*compare the ssid*/
		host_req[len] = WID_BCAST_SSID & 0xFF;
    	host_req[len+1] = (WID_BCAST_SSID & 0xFF00) >> 8;
		host_req[len+2] = 1; 
		host_req[len+3] = 0; // 1:not compare ssid
		len += WID_CHAR_CFG_LEN;
		#endif
	}
	else
	{
		#if 0
		host_req[len] = WID_BSSID & 0xFF;
		host_req[len+1] = (WID_BSSID & 0xFF00) >> 8;
		host_req[len+2] = 0x06;
		memset(&host_req[len+3],0x00,6);
		len += WID_HEAD_LEN + 6;
		#endif

		host_req[len] = WID_SSID & 0xFF;
		host_req[len+1] = (WID_SSID & 0xFF00) >> 8;
		host_req[len+2] = 0x01;
		host_req[len+3] = 0x00;
		len += WID_CHAR_CFG_LEN;

		#if 1
		/*compare the ssid*/
		host_req[len] = WID_BCAST_SSID & 0xFF;
    	host_req[len+1] = (WID_BCAST_SSID & 0xFF00) >> 8;
		host_req[len+2] = 1; 
		host_req[len+3] = 1; // 1:not compare ssid
		len += WID_CHAR_CFG_LEN;
		#endif
	}
	
	//len += WID_HEAD_LEN + essid_len;

	#if 0 //chenq for test
	host_req[len] = WID_11I_MODE & 0xFF;
	host_req[len+1] = (WID_11I_MODE & 0xFF00) >> 8;
	host_req[len+2] = 0x01;
	host_req[len+3] = 0x31;
	len += WID_CHAR_CFG_LEN;

	host_req[len] = WID_11I_PSK & 0xFF;
	host_req[len+1] = (WID_11I_PSK & 0xFF00) >> 8;
	host_req[len+2] = 8;
	memcpy(&host_req[len+3],"87654321",8);
	len += WID_CHAR_CFG_LEN + 7;
	#endif
	
	if( (memcmp(ap_name,essid,essid_len) == 0) 
		#ifdef IBSS_BSS_STATION_MODE 
		&& ((get_mac_state() == ENABLED) || (g_keep_connection == BTRUE))
		&& (essid != NULL) 
		#endif
	  )
	{
		PRINTK_ITMIW("<%s> is already config,and wifi enable,we will return!\n",ap_name);
		goto out1;
	}
	/*
	else
	{
		memcpy(ap_name,essid,essid_len);
	}
	*/

	config_if_for_iw(&g_mac,host_req,len,'W',&trout_rsp_len);

	if( trout_rsp_len != 1 )
	{
		ret = -EINVAL;
		goto out1;
	}

	if(essid)
	{
		int i = 0;

		PRINTK_ITMIW("itm_set_essid fuc set essid <");
		for( i = 0 ; i < essid_len ; i ++)
			PRINTK_ITMIW("%c",essid[i]);
		PRINTK_ITMIW(">\n");

		//PRINTK_ITMIW("itm_set_essid fuc set essid <%s>  \n", essid );
	}	
	else
		PRINTK_ITMIW("itm_set_essid fuc clear essid \n");
    
	out1:
	//kfree(host_req);
	return ret;
}	

/*leon liu exported itm_get_essid to global*/
int 
itm_get_essid(UWORD8 * essid,int essid_len)
{
	int ret = 0;
	UWORD8 * host_req  = NULL;
	UWORD8 * trout_rsp = NULL;
	UWORD16  trout_rsp_len = 0;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if(host_req == NULL)
	{
		return -ENOMEM;
	}

	host_req[0] = WID_SSID & 0xFF;
	host_req[1] = (WID_SSID & 0xFF00) >> 8;
	trout_rsp = config_if_for_iw(&g_mac,host_req,2,'Q',&trout_rsp_len);

	if( trout_rsp == NULL )
	{
		ret = -EINVAL;
		goto out1;
	}

	trout_rsp_len -= MSG_HDR_LEN;

	if(trout_rsp_len == 0)
	{
	    goto out2;
	}

	if((ret = get_trour_rsp_data(essid,essid_len,&trout_rsp[MSG_HDR_LEN],
					      trout_rsp_len,WID_STR))  ==  0)
	{	
		ret = -EINVAL;
		goto out2;
	}

	out2:
	pkt_mem_free(trout_rsp);
	out1:
	//kfree(host_req);
	return ret;

}
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
void sleep_disconnected(void)
{
	pr_info("%s   enter sleep ############# 1\n", __func__);
	UWORD8 cur_essid[IW_ESSID_MAX_SIZE+1] = {0};
	if (get_mac_state() != ENABLED && g_keep_connection == BFALSE
			&& !itm_get_essid(cur_essid,IW_ESSID_MAX_SIZE) && (BOOL_T)atomic_read(&g_mac_reset_done) == BTRUE
			&& g_wifi_suspend_status == wifi_suspend_nosuspend
			#ifdef CONFIG_CFG80211
			&& g_mac_dev->ieee80211_ptr->sme_state == CFG80211_SME_IDLE
			#endif
	   ){
		pr_info("%s   enter sleep ############# 2\n", __func__);
		sta_sleep_disconnected();
	}
}
EXPORT_SYMBOL(sleep_disconnected);
#endif
static int 
itm_get_rssi(int * rssi)
{
	int ret = 0;
	UWORD8 * host_req  = NULL;
	UWORD8 * trout_rsp = NULL;
	UWORD16  trout_rsp_len = 0;
	WORD8  rssi_db = 0;
	int signal = 0;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	PRINTK_ITMIW("itm_get_rssi fuc\n");

	#ifdef IBSS_BSS_STATION_MODE
	if( ( get_mac_state() != ENABLED ) && (g_keep_connection == BFALSE) )
	{
		return -1;
	}
	#else
	if( get_mac_state() != ENABLED )
	{
		return -1;
	}
	#endif

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if(host_req == NULL)
	{
		return -1;
	}

	host_req[0] = WID_RSSI & 0xFF;
	host_req[1] = (WID_RSSI & 0xFF00) >> 8;
	trout_rsp = config_if_for_iw(&g_mac,host_req,2,'Q',&trout_rsp_len);

	if( trout_rsp == NULL )
	{
		PRINTK_ITMIW("trout_rsp == NULL\n");
		return -1;
	}

	trout_rsp_len -= MSG_HDR_LEN;
	if((ret = get_trour_rsp_data((UWORD8*)(&rssi_db),sizeof(UWORD8),&trout_rsp[MSG_HDR_LEN],
					      trout_rsp_len,WID_CHAR))  ==  0)
	{	
		pkt_mem_free(trout_rsp);
		PRINTK_ITMIW("get_trour_rsp_data == fail\n");
		return -1;
	}
	pkt_mem_free(trout_rsp);

	signal = -(0xFF - rssi_db);
	signal = (signal < 63) ? signal : 63;
    signal = (signal > -192) ? signal : -192;

	*rssi = signal;

	return 0;

}

static int itm_get_rate(void)
{
	int ret = 0;
	UWORD8 * host_req  = NULL;
	UWORD8 * trout_rsp = NULL;
	UWORD16  trout_rsp_len = 0;
	UWORD8 rate=0;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if(host_req == NULL)
	{
		return -ENOMEM;
	}

	host_req[0] = WID_CURRENT_TX_RATE & 0xFF;
	host_req[1] = (WID_CURRENT_TX_RATE & 0xFF00) >> 8;

	trout_rsp = config_if_for_iw(&g_mac,host_req,2,'Q',&trout_rsp_len);

	if( trout_rsp == NULL )
	{
		ret = -EINVAL;
		goto out1;
	}

	trout_rsp_len -= MSG_HDR_LEN;
	if(get_trour_rsp_data(&rate,sizeof(rate),&trout_rsp[MSG_HDR_LEN],
					      trout_rsp_len,WID_CHAR)  ==  0)
	{
		ret = -EINVAL;
		goto out2;
	}

	if(rate == 0)
	{
		rate = 1;
	}

	out2:
	pkt_mem_free(trout_rsp);
	out1:
	//kfree(host_req);
	return (ret < 0) ? ret : rate;
}

static int itm_get_bssid(UWORD8 * bssid)
{
	int ret = 0;
	UWORD8 * host_req  = NULL;
	UWORD8 * trout_rsp = NULL;
	UWORD16  trout_rsp_len = 0;
	UWORD8   ap_addr[6] = {0};

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if(host_req == NULL)
	{
		return -ENOMEM;
	}

	host_req[0] = 	WID_BSSID & 0xFF;
	host_req[1] = (WID_BSSID & 0xFF00) >> 8;
	
	trout_rsp = config_if_for_iw(&g_mac,host_req,2,'Q',&trout_rsp_len);

	if( trout_rsp == NULL )
	{
		ret = -EINVAL;
		goto out1;
	}

	trout_rsp_len -= MSG_HDR_LEN;
	if(get_trour_rsp_data(ap_addr,sizeof(ap_addr),
		                              &trout_rsp[MSG_HDR_LEN],trout_rsp_len,
		                              WID_STR) == 0 )
	{
		ret = -EINVAL;
		goto out2;
	}

	memcpy(bssid,ap_addr,sizeof(ap_addr));

	{
		int i = 0 ;
		for( i = 0 ; i < 6 ; i++)
		{
			printk("%02x ",ap_addr[i]);
		}
		printk("\n");
	}

	out2:
	pkt_mem_free(trout_rsp);
	out1:
	//kfree(host_req);	
	return ret;
}


/* yangke exported itm_get_devmac to global*/
int itm_get_devmac(UWORD8 * dev_mac)
{
	int ret = 0;
	UWORD8 * host_req  = NULL;
	UWORD8 * trout_rsp = NULL;
	UWORD16  trout_rsp_len = 0;
	UWORD8   sta_addr[6] = {0};

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if(host_req == NULL)
	{
		return -ENOMEM;
	}

	host_req[0] = 	WID_MAC_ADDR & 0xFF;
	host_req[1] = (WID_MAC_ADDR & 0xFF00) >> 8;
	
	trout_rsp = config_if_for_iw(&g_mac,host_req,2,'Q',&trout_rsp_len);

	if( trout_rsp == NULL )
	{
		ret = -EINVAL;
		goto out1;
	}

	trout_rsp_len -= MSG_HDR_LEN;
	if(get_trour_rsp_data(sta_addr,sizeof(sta_addr),
		                              &trout_rsp[MSG_HDR_LEN],trout_rsp_len,
		                              WID_STR) == 0 )
	{
		ret = -EINVAL;
		goto out2;
	}

	memcpy(dev_mac,sta_addr,sizeof(sta_addr));

	{
		int i = 0 ;
		for( i = 0 ; i < 6 ; i++)
		{
			printk("%02x ",sta_addr[i]);
		}
		printk("\n");
	}

	out2:
	pkt_mem_free(trout_rsp);
	out1:
	//kfree(host_req);	
	return ret;
}

//caisf add get current domain max channel number 2013-05-05
UWORD8 itm_get_max_ch_no(void)
{
	return get_scan_req_channel_no();
}

//caisf mod 2012-12-27
static int itm_set_domain(int flag, int scan_ch_no)
{
	int ret = 0;
	UWORD8 * host_req  = NULL;
	UWORD16  trout_rsp_len = 0;
    int len = 0;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_NO_ACCESS);

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if(host_req == NULL)
	{
		return -ENOMEM;
	}


#ifdef MAC_MULTIDOMAIN
	// caisf mod 0502
    if(!get_multi_domain_supp())
    {
    	PRINTK_ITMIW("get_multi_domain_supp(): current not support.\n");
		goto out1;
    }
	
    if(flag == 1)
    {
        if(!mget_MultiDomainCapabilityEnabled())
        {
        	host_req[len+0] = WID_ENABLE_MULTI_DOMAIN & 0xFF;
        	host_req[len+1] = (WID_ENABLE_MULTI_DOMAIN & 0xFF00) >> 8;
        	host_req[len+2] = 1;
        	host_req[len+3] = flag;
        	len += WID_CHAR_CFG_LEN;
        }
    }
    else
    {
        if(mget_MultiDomainCapabilityEnabled())
        {
        	host_req[len+0] = WID_ENABLE_MULTI_DOMAIN & 0xFF;
        	host_req[len+1] = (WID_ENABLE_MULTI_DOMAIN & 0xFF00) >> 8;
        	host_req[len+2] = 1;
        	host_req[len+3] = flag;
        	len += WID_CHAR_CFG_LEN;
        }
    }
#endif

    // caisf add for multi-domain support
    if(itm_get_max_ch_no()!=scan_ch_no && (scan_ch_no >= 11))
    {
    	host_req[len+0] = WID_SCAN_REQ_CHANNEL_NO & 0xFF;
    	host_req[len+1] = (WID_SCAN_REQ_CHANNEL_NO & 0xFF00) >> 8;
    	host_req[len+2] = 1;
    	host_req[len+3] = scan_ch_no & 0xFF;
    	len += WID_CHAR_CFG_LEN;

    	/* clear channel to channel 1 */
		host_req[len+0] = WID_PRIMARY_CHANNEL & 0xFF;
		host_req[len+1] = (WID_PRIMARY_CHANNEL & 0xFF00) >> 8;
		host_req[len+2] = 1;
		host_req[len+3] = 1;
		len += WID_CHAR_CFG_LEN;

    	/* clear ssid */
		host_req[len] = WID_SSID & 0xFF;
		host_req[len+1] = (WID_SSID & 0xFF00) >> 8;
		host_req[len+2] = 0x01;
		host_req[len+3] = 0x00;
		len += WID_CHAR_CFG_LEN;
		
		//xuan yang, 2013.7.06, set disc flag.
		send_disconnect_flg(1);

    }
	else
	{
    	PRINTK_ITMIW("The scan channel nubmer is already configured, current=%d, req=%d.\n", 
			itm_get_max_ch_no(), scan_ch_no);
		goto out1;
	}


	config_if_for_iw(&g_mac,host_req,len,'W',&trout_rsp_len);

	if( trout_rsp_len != 1 )
	{
		ret = -EINVAL;
		goto out1;
	}
	   
	out1:
	//kfree(host_req);
	return ret;
}

static int 
itm_remove_wep_key(void)
{
	int ret = 0;
	UWORD8 * host_req  = NULL;
	UWORD16  trout_rsp_len = 0;
	int len=0;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if(host_req == NULL)
	{
		return -ENOMEM;
	}

	host_req[0] = WID_REMOVE_WEP_KEY & 0xFF;
	host_req[1] = (WID_REMOVE_WEP_KEY & 0xFF00) >> 8;
	
	if( (len = strlen("0") ) >( MAX_SSID_LEN - 1) )
	{
		len = MAX_SSID_LEN - 1;
	}
	host_req[2] = len & 0xFF;
	memcpy(&host_req[3],"0",len);
	host_req[3+len] = 0x00;
	
	len+=WID_HEAD_LEN;

	config_if_for_iw(&g_mac,host_req,len,'W',&trout_rsp_len);

	if( trout_rsp_len != 1 )
	{
		ret = -EINVAL;
		goto out1;
	}

    PRINTK_ITMIW("itm_remove_wep_key\n");
    
	out1:
	//kfree(host_req);
	return ret;

}		

		int 
itm_add_wep_key(UWORD8 * key,int key_len,int key_index)
{
	int ret = 0;
	UWORD8 * host_req  = NULL;
	UWORD16  trout_rsp_len = 0;
	int len=0;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if(host_req == NULL)
	{
		return -ENOMEM;
	}

	host_req[0] = WID_ADD_WEP_KEY & 0xFF;
	host_req[1] = (WID_ADD_WEP_KEY & 0xFF00) >> 8;

    len = key_len + 1 + 1;
	host_req[2] = len & 0xFF;
    host_req[3] = key_index & 0xFF;
    host_req[4] = key_len & 0xFF;
	memcpy(&host_req[5],key,key_len);
	host_req[5+len] = 0x00;
	
	len+=WID_HEAD_LEN;

	config_if_for_iw(&g_mac,host_req,len,'W',&trout_rsp_len);

	if( trout_rsp_len != 1 )
	{
		ret = -EINVAL;
		goto out1;
	}

    PRINTK_ITMIW("itm_add_wep_key\n");
    
	out1:
	//kfree(host_req);
	return ret;

}		

static void indicate_custom_event(char *msg)
{
	WORD8 *buff;
	union iwreq_data wrqu;

	if (strlen(msg) > IW_CUSTOM_MAX) 
	{
		PRINTK_ITMIW("%s strlen(msg):%u > IW_CUSTOM_MAX:%u\n", __FUNCTION__ ,strlen(msg), IW_CUSTOM_MAX);
		return;
	}

	buff = g_itm_config_buf;
	if(!buff)
	{
		return;
	}
	
	memcpy(buff, msg, strlen(msg));
		
	memset(&wrqu,0,sizeof(wrqu));
	wrqu.data.length = strlen(msg);

	PRINTK_ITMIW("%s %s\n", __FUNCTION__, msg);	
		/*leon liu added judgement for WEXT*/
#ifdef CONFIG_TROUT_WEXT
		wireless_send_event(g_mac_dev, IWEVCUSTOM, &wrqu, buff);
#endif
}
#ifdef ITM_WIFI_SECURITY_WAPI_ENABLE
/*leon liu modified, export itm_set_wapimode to global*/
int
itm_set_wapimode(int mode)
{
	UWORD8 * host_req  = NULL;
	UWORD16  trout_rsp_len = 0;
	UWORD8 enable = 0;
	int len = 0;
	int ret = 0;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_NO_ACCESS);	
		
    PRINTK_ITMIW("itm_iwprivsconfwapi \n");
	
	enable = mode;
/*jiangtao.yi add CONFIG_CFG80211 for bug228447.*/
#ifndef CONFIG_CFG80211
	if(!enable)
	{
		PRINTK_ITMIW("disable wapi\n");
		return 0;
	}
#endif
	/*make wid pkt*/
	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);
	if(host_req == NULL)
	{
		return -ENOMEM;
	}
	host_req[len+0] = WID_11I_MODE & 0xFF;
    host_req[len+1] = (WID_11I_MODE & 0xFF00) >> 8;
	host_req[len+2] = 1; 
	host_req[len+3] = 0x00; 
	len += WID_CHAR_CFG_LEN;

    host_req[len+0] = WID_AUTH_TYPE & 0xFF;
    host_req[len+1] = (WID_AUTH_TYPE & 0xFF00) >> 8;
	host_req[len+2] = 1; 
	host_req[len+3] = 1; 
	len += WID_CHAR_CFG_LEN;
	
	host_req[len+0] = WID_WAPI_MODE & 0xFF;
    host_req[len+1] = (WID_WAPI_MODE & 0xFF00) >> 8;
	host_req[len+2] = 1; 
	if(enable)
	{
		host_req[len+3] = 1; 
	}
	else
	{
		host_req[len+3] = 0; 
	}
	len += WID_CHAR_CFG_LEN;

	host_req[len+0] = WID_RESET & 0xFF;
	host_req[len+1] = (WID_RESET & 0xFF00) >> 8;
	host_req[len+2] = 1;
	host_req[len+3] = 0; //DONT_RESET
	len += WID_CHAR_CFG_LEN;

	config_if_for_iw(&g_mac,host_req,len,'W',&trout_rsp_len);

	if( trout_rsp_len != 1 )
	{
		ret = -EINVAL;
		goto out1;
	}

    //PRINTK_ITMIW("itm_iwprivsconfwapi mode %s \n",(enable) ? "TV_TRUE" : "TV_FALSE");
    
	out1:
	//kfree(host_req);
	return ret;
	
}
         
/*leon liu exported iwprivswapikey to global*/
int
iwprivswapikey(wapi_key_t * inKey)
{
	UWORD8 * host_req  = NULL;
	UWORD16  trout_rsp_len = 0;
	int len = 0;
	int ret = 0;
	int i   = 0;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_NO_ACCESS);

	PRINTK_ITMIW("itm_iwprivswapikey \n");

	/*make wid pkt*/
	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if(host_req == NULL)
	{
		return -ENOMEM;
	}

/*************************************************************************/
/*                        set_wapi_key wid Format                        */
/* --------------------------------------------------------------------  */
/* |PAIRWISE/group|key index|BSSID|RSC|key(rxtx + mic)|                  */
/* --------------------------------------------------------------------  */
/* | 1            |1        |6    |16 |32             |                  */
/* --------------------------------------------------------------------  */
/*                                                                       */
/*************************************************************************/

	host_req[len++] = WID_WAPI_KEY & 0xFF;
    host_req[len++] = (WID_WAPI_KEY & 0xFF00) >> 8;
	host_req[len++] = 1 + 1 + 6 + 16 + 32;

    if (inKey->unicastKey) 
	{
        PRINTK_ITMIW("WIFI_WAPI_KEY_TYPE_PAIRWISE\n");
		host_req[len++] = 1;
    } 
	else 
    {
        PRINTK_ITMIW("WIFI_WAPI_KEY_TYPE_GROUP\n");
		host_req[len++] = 0;
    }

    PRINTK_ITMIW("inKey->keyIndex %d\n",inKey->keyIndex);
	host_req[len++] = inKey->keyIndex;

	PRINTK_ITMIW("address\n");
	for( i = 0 ; i < 6 ; i++)
	{
		PRINTK_ITMIW("%02x ",inKey->address[i]);	
	}
	memcpy(&host_req[len],inKey->address,6);
	len += 6;
	PRINTK_ITMIW("\n");

    /* memcpy(key.keyRsc, inKey.keyRsc, 16); */
	PRINTK_ITMIW("keyRsc\n");
    for( i = 0; i < 16; i++ )
    {
		host_req[len+i] = inKey->keyRsc[i];
    	PRINTK_ITMIW("%02x ",inKey->keyRsc[i]);
    }
	len += 16;
	PRINTK_ITMIW("\n");

	PRINTK_ITMIW("key\n");
	for( i = 0 ; i < 32 ; i++)
	{
		PRINTK_ITMIW("%02x ",inKey->key[i]);
	}
	memcpy(&host_req[len],inKey->key,32);
	len += 32;
	PRINTK_ITMIW("\n");

	len += WID_HEAD_LEN;

	config_if_for_iw(&g_mac,host_req,len,'W',&trout_rsp_len);

	if( trout_rsp_len != 1 )
	{
		ret = -EINVAL;
		goto out1;
	}
    
	out1:
	//kfree(host_req);
	return ret;

}
#endif

static int 
itm_set_channel(int channel)
{
	UWORD8 * host_req  = NULL;
	UWORD16  trout_rsp_len = 0;
	int len = 0;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if(host_req == NULL)
	{
		return -ENOMEM;
	}

	host_req[len+0] = WID_PRIMARY_CHANNEL & 0xFF;
	host_req[len+1] = (WID_PRIMARY_CHANNEL & 0xFF00) >> 8;
	host_req[len+2] = 1;
	host_req[len+3] = channel & 0xFF;
	len += WID_CHAR_CFG_LEN;

	host_req[len+0] = WID_RESET & 0xFF;
	host_req[len+1] = (WID_RESET & 0xFF00) >> 8;
	host_req[len+2] = 1;
	host_req[len+3] = 0; //DONT_RESET
	len += WID_CHAR_CFG_LEN;
		
	config_if_for_iw(&g_mac,host_req,len,'W',&trout_rsp_len);

	if( trout_rsp_len != 1 )
	{
		return -EINVAL;
	}

	PRINTK_ITMIW("itm_siwfreq fuc set channel:%d\n",channel);
	return 0;
}

static int itm_set_scan(struct net_device *dev,
	                       union iwreq_data *awrq,
				           char *extra)
{
#ifdef IBSS_BSS_STATION_MODE
	int ret = 0;
	UWORD8 * host_req  = NULL;
	UWORD16  trout_rsp_len = 0;
	//struct  iw_point * data = ( struct iw_point * )wrqu;
	//int i = 0;
	int len = 0;
	int scan_type = 0;
	struct in_device *my_ip_ptr = dev->ip_ptr;

	UWORD8 cur_essid[IW_ESSID_MAX_SIZE+1] = {0};
	int cur_essid_len = 0;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);	
	
	PRINTK_ITMIW("itm_set_scan fuc\n");

	if( ( get_mac_state() == ENABLED ) || (g_keep_connection == BTRUE) )
	{
		if(my_ip_ptr != NULL)
		{
			struct in_ifaddr *my_ifa_list = my_ip_ptr->ifa_list;
			if(my_ifa_list != NULL)
			{
				goto CHECK_SCAN;
			}
		}

		PRINTK_ITMIW("skip scan op <ip addr less> \n");
		//chenq mod 2013-05-23
		is_scanlist_report2ui = 2;
		send_mac_status(MAC_SCAN_CMP);
		return 0;
		//return -EAGAIN;
	}

	cur_essid_len = itm_get_essid(cur_essid,IW_ESSID_MAX_SIZE);
	if(cur_essid_len)
	{
		PRINTK_ITMIW("skip scan op <%s> \n",
			(cur_essid_len < 0) ? "get cur essid err":"wifi linking" );
		return -EAGAIN;
	}

CHECK_SCAN:
	if( itm_scan_flag == 1 )
	{
		PRINTK_ITMIW("skip scan op <already in scan> \n");
		//chenq add 2013-06-09
		is_scanlist_report2ui = 2;
		send_mac_status(MAC_SCAN_CMP);
		return 0;
	}
	else if(g_BusyTraffic == BTRUE)
	{
		PRINTK_ITMIW("skip scan op <BusyTraffic> \n");
		//chenq add 2013-06-09
		is_scanlist_report2ui = 2;
		send_mac_status(MAC_SCAN_CMP);
		return 0;
	}

//chenq add for combo scan 2013-04-11
#ifdef COMBO_SCAN
	{
		// scan
		int combo_scan_len = awrq->data.length - WEXT_CSCAN_HEADER_SIZE;
		char *pos = extra+WEXT_CSCAN_HEADER_SIZE;
		char section;
		char sec_len;
		g_ap_combo_scan_index = 0;
		g_ap_combo_scan_cnt = 0;

		while( (combo_scan_len >= 1) && (g_ap_combo_scan_cnt < MAX_AP_COMBO_SCAN) )
		{
			section = *(pos++); combo_scan_len-=1;

			switch(section) 
			{
				case WEXT_CSCAN_SSID_SECTION:
					if(combo_scan_len < 1) 
					{
						len = 0;
						break;
					}
					
					sec_len = *(pos++); combo_scan_len-=1;

					if(sec_len>0 && sec_len<=combo_scan_len)
					{
						memcpy(g_ap_combo_scan_list[g_ap_combo_scan_cnt],pos,min(sec_len,MAX_SSID_LEN-1));
						g_ap_combo_scan_list[g_ap_combo_scan_cnt][min(sec_len,MAX_SSID_LEN-1)] = 0x00;
						PRINTK_ITMIW("%s COMBO_SCAN with specific ssid:%s, %d\n", __FUNCTION__
							, g_ap_combo_scan_list[g_ap_combo_scan_cnt],min(sec_len,MAX_SSID_LEN-1));
						g_ap_combo_scan_cnt++;
					}
					
					pos+=sec_len; combo_scan_len-=sec_len;
					break;
					
				case WEXT_CSCAN_CHANNEL_SECTION:
					//PRINTK_ITMIW("WEXT_CSCAN_CHANNEL_SECTION\n");
					pos+=1; combo_scan_len-=1;
					break;
				case WEXT_CSCAN_ACTV_DWELL_SECTION:
					//PRINTK_ITMIW("WEXT_CSCAN_ACTV_DWELL_SECTION\n");
					pos+=2; combo_scan_len-=2;
					break;
				case WEXT_CSCAN_PASV_DWELL_SECTION:
					//PRINTK_ITMIW("WEXT_CSCAN_PASV_DWELL_SECTION\n");
					pos+=2; combo_scan_len-=2;					
					break;
				case WEXT_CSCAN_HOME_DWELL_SECTION:
					//PRINTK_ITMIW("WEXT_CSCAN_HOME_DWELL_SECTION\n");
					pos+=2; combo_scan_len-=2;
					break;
				case WEXT_CSCAN_TYPE_SECTION:
					//PRINTK_ITMIW("WEXT_CSCAN_TYPE_SECTION\n");
					pos+=1; combo_scan_len-=1;
					break;
				#if 0
				case WEXT_CSCAN_NPROBE_SECTION:
					PRINTK_ITMIW("WEXT_CSCAN_NPROBE_SECTION\n");
					break;
				#endif
				
				default:
					//PRINTK_ITMIW("Unknown CSCAN section %c\n", section);
					combo_scan_len = 0; // stop parsing
			}
		    //PRINTK_ITMIW("len:%d\n", len);
		}
	}
#endif

	scan_type = 1;

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if(host_req == NULL)
	{
		return -ENOMEM;
	}

	host_req[len+0] = WID_BCAST_SSID & 0xFF;
    host_req[len+1] = (WID_BCAST_SSID & 0xFF00) >> 8;
	host_req[len+2] = 1; 
	host_req[len+3] = 0; // 0:not compare ssid
	len += WID_CHAR_CFG_LEN;

    host_req[len+0] = WID_SCAN_TYPE & 0xFF;
    host_req[len+1] = (WID_SCAN_TYPE & 0xFF00) >> 8;
	host_req[len+2] = 1; 
	host_req[len+3] = scan_type & 0xFF; // 1:  Active Scanning   0:pass
	len += WID_CHAR_CFG_LEN;
	
	host_req[len+0] = WID_START_SCAN_REQ & 0xFF;
	host_req[len+1] = (WID_START_SCAN_REQ & 0xFF00) >> 8;
	host_req[len+2] = 1;
	host_req[len+3] = 1; //trigger scanning
	len += WID_CHAR_CFG_LEN;

	host_req[len+0] = WID_RESET & 0xFF;
	host_req[len+1] = (WID_RESET & 0xFF00) >> 8;
	host_req[len+2] = 1;
	host_req[len+3] = 0;//( (g_mac.state == ENABLED) || (g_keep_connection == BTRUE) ) ? 0 : 2; //DONT_RESET
	len += WID_CHAR_CFG_LEN;

	#if 0 //chenq mask 20120903
	host_req[len+0] = WID_SITE_SURVEY & 0xFF;
	host_req[len+1] = (WID_SITE_SURVEY & 0xFF00) >> 8;
	host_req[len+2] = 1; 
	host_req[len+3] = 1; //all channel
	len += WID_CHAR_CFG_LEN;
	#endif

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifdef WIFI_SLEEP_POLICY
	//wake_lock(&scan_ap_lock); /*Keep awake when scan ap, by caisf 20130929*/
	//pr_info("%s-%d: acquire wake_lock %s\n", __func__, __LINE__, scan_ap_lock.name);
#endif
#endif

	config_if_for_iw(&g_mac,host_req,len,'W',&trout_rsp_len);

	if( trout_rsp_len != 1 )
	{
		ret = -EINVAL;
		goto out1;
	}
	
	out1:
	//kfree(host_req);
	if(itm_scan_flag == 1)
	{
		if(!ret)
		{
			is_scanlist_report2ui = 1;
			//itm_scan_task = current;
			//set_current_state(TASK_INTERRUPTIBLE); 
			//printk("chenq debug itm_siwscan %p\n",itm_scan_task);
			//schedule();
			//is_scanlist_report2ui = 0;
		}
		else
		{
			//chenq add 2013-06-09ss
			is_scanlist_report2ui = 2;
			send_mac_status(MAC_SCAN_CMP);
			ret = 0;//-EBUSY;
		}	
	}
	else
	{
		//chenq add 2013-06-09
		is_scanlist_report2ui = 2;
		send_mac_status(MAC_SCAN_CMP);
		ret = 0;//-EINVAL;
	}
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifdef WIFI_SLEEP_POLICY
	//wake_unlock(&scan_ap_lock); /*Keep awake when scan ap, by caisf 20130929*/
	//pr_info("%s-%d: release wake_lock %s\n", __func__, __LINE__, scan_ap_lock.name);
#endif
#endif

	PRINTK_ITMIW("itm_set_scan fuc end code:%d\n",ret);
	return ret;

	#else //ap mode

	PRINTK_ITMIW("itm_set_scan fuc\n");

	return -ESRCH;
	#endif
}

#define iw_handler_fuc_list 0

static int dummy(struct net_device *dev, struct iw_request_info *a,
		 union iwreq_data *wrqu, char *b)
{
	return -1;	
}


/*------------------------------------------------------------------*/
/*
 * Standard Wireless Handler :  Commit pending changes to driver
 */
static int itm_siwcommit(struct net_device *	dev,
			  struct iw_request_info *info,
			  union iwreq_data *	wrqu,  char * extra)
{
	PRINTK_ITMIW("itm_siwcommit fuc\n");
	return 0;
}




/*------------------------------------------------------------------*/
/*
 * Standard Wireless Handler :  get name == wireless protocol
 */
static int itm_giwname( struct net_device * dev,
			  struct iw_request_info *info,
			  union iwreq_data *  wrqu,  char *  extra ) 
{
	int ret =0;
	UWORD8 * host_req  = NULL;
	UWORD8 * trout_rsp = NULL;
	UWORD16  trout_rsp_len = 0;
	UWORD16  user_channel = 0;
	char *name = wrqu->name;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	PRINTK_ITMIW("itm_giwname fuc\n");
	
	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

    if(host_req == NULL)
    {
    	return -ENOMEM;
    }
	
	host_req[0] = 	WID_USER_PREF_CHANNEL & 0xFF;
	host_req[1] = (WID_USER_PREF_CHANNEL & 0xFF00) >> 8;

	trout_rsp = config_if_for_iw(&g_mac,host_req,2,'Q',&trout_rsp_len);

	if( trout_rsp == NULL )
	{
		ret = -EINVAL;
		goto out1;
	}

	trout_rsp_len -= MSG_HDR_LEN;
	if( get_trour_rsp_data((UWORD8 *)(&user_channel),sizeof(user_channel),
						&trout_rsp[MSG_HDR_LEN],trout_rsp_len,
						WID_SHORT) == 0)
	{
		ret = -EINVAL;
		goto out2;
	}

	if (  user_channel & 0x8000  )
	{
		/* 5G */
		//data->length = strlen("IEEE 802.11-a");
		strcpy(name, "IEEE 802.11a");		
	} 
	else 
	{
		/* 2.4G */
		//data->length = strlen("IEEE 802.11-bgn");
		strcpy(name, "IEEE 802.11bgn");
	}
	
	out2:
	pkt_mem_free(trout_rsp);
	out1:
	//kfree(host_req);
	return ret;
}
 

/*------------------------------------------------------------------*/
/*
 * Standard Wireless Handler :  set channel/frequency (Hz) 
 */

static int itm_siwfreq(struct net_device *	dev,
			  struct iw_request_info *info,
			  union iwreq_data *	wrqu,  char * extra)
{
	int channel=0;
	struct iw_freq * freq = (struct iw_freq *)wrqu;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	PRINTK_ITMIW("itm_siwfreq fuc\n");

	if ((freq->e == 0) && (freq->m <= 1000)) 
	{
		channel = freq->m;
	} 
	else 
	{
		channel = wext_freq_to_channel(freq->m, freq->e);
	}

	return itm_set_channel(channel);

}
 
/*------------------------------------------------------------------*/
/*
 * Standard Wireless Handler :  get channel/frequency (Hz)
 */
static int itm_giwfreq(struct net_device *	dev,
			  struct iw_request_info *info,
			   union iwreq_data * wrqu,  char * extra)
{
	int ret = 0;
	UWORD8 * host_req  = NULL;
	UWORD8 * trout_rsp = NULL;
	UWORD16  trout_rsp_len = 0;
	UWORD8  channel=0;
	struct iw_freq * freq = (struct iw_freq *)wrqu;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	PRINTK_ITMIW("itm_giwfreq fuc\n");

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

    if(host_req == NULL)
    {
		return -ENOMEM;
    }

	host_req[0] = 	WID_PRIMARY_CHANNEL & 0xFF;
	host_req[1] = (WID_PRIMARY_CHANNEL & 0xFF00) >> 8;
	
	trout_rsp = config_if_for_iw(&g_mac,host_req,2,'Q',&trout_rsp_len);

	if( trout_rsp == NULL )
	{
		ret = -EINVAL;
		goto out1;
	}

	trout_rsp_len -= MSG_HDR_LEN;
	if( get_trour_rsp_data(&channel,sizeof(channel),
						&trout_rsp[MSG_HDR_LEN],trout_rsp_len,
						WID_CHAR) == 0)
	{
		ret = -EINVAL;
		goto out2;
	}
	freq->m  = itm_chan_2_freq(channel) * 100000 ;
	freq->e  = 1;
	freq->i  = channel;
	
	out2:
	pkt_mem_free(trout_rsp);
	out1:
	//kfree(host_req);
	return ret;
}
 
/*------------------------------------------------------------------*/
/*
 * Standard Wireless Handler :  set operation mode
 */
static int itm_siwmode(struct net_device *	dev,
			  struct iw_request_info *info,
			   union iwreq_data * wrqu,  char * extra)
{
	UWORD8 mode = wrqu->mode;
    //UWORD8 device_mode = 0;
    //UWORD8 clear_essid= 0x00;
    //UWORD8 clear_bssid[]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

	PRINTK_ITMIW("itm_siwmode fuc\n");
    PRINTK_ITMIW("itm_siwmode fuc set mode = %d\n",mode);
	
	return 0;
}

/*------------------------------------------------------------------*/
/*
 * Standard Wireless Handler : get operation mode
 */

static int itm_giwmode(struct net_device *	dev,
			  struct iw_request_info *info,
			   union iwreq_data * wrqu,  char * extra)
{
	UWORD8 device_mode=0;
    UWORD8 bss_type=0;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	PRINTK_ITMIW("itm_giwmode fuc\n");

    if( (device_mode = itm_get_device_mode() & 0xFF) < 0 )
    {
        return -EINVAL;
    }

    if( (bss_type = itm_get_BSS_type() & 0xFF) < 0 )
    {
        return -EINVAL;
    }
	
    PRINTK_ITMIW("itm_giwmode: Exisitng mode = %d\n",
                device_mode);
    
    switch(device_mode)
    {
        case DEVICE_MODE_BSS_STA:
        case DEVICE_MODE_P2P_Client:
           wrqu->mode = IW_MODE_INFRA;
           break;
        case DEVICE_MODE_Access_Point:
        case DEVICE_MODE_P2P_GO:
            wrqu->mode = IW_MODE_MASTER;
            break;
        case DEVICE_MODE_IBSS_STA:
            wrqu->mode = IW_MODE_ADHOC;
            break;
        case DEVICE_MODE_P2P_Device:
        case DEVICE_MODE_Not_Configured:

            switch(bss_type) 
            {
                case IBSS_TYPE_STA:
                    wrqu->mode = IW_MODE_ADHOC;
                    break;
                case BSS_TYPE_STA:
                    wrqu->mode = IW_MODE_INFRA;
                    break;
					
				#ifdef BSS_ACCESS_POINT_MODE
				case AP_TYPE_AP:
					wrqu->mode = IW_MODE_MASTER;
                    break;
				#endif
				
                default:
                    wrqu->mode = IW_MODE_AUTO;
                    PRINTK_ITMIW("Unknown IW MODE value.\n");
            }

            break;
        default:
            wrqu->mode = IW_MODE_AUTO;
            PRINTK_ITMIW("Unknown IW MODE value.\n");

    }

    PRINTK_ITMIW("itm_giwmode fuc get mode = %d\n",wrqu->mode);
    return 0;
}

static int itm_giwsens(struct net_device *dev, 
			     struct iw_request_info *info, 
			     union iwreq_data *wrqu, char *extra)
{

	int ret  = 0;
	int rssi = 0;
	/*
	*  For rockchip platform's wpa_driver_wext_get_rssi
	*/
	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);	
	
	PRINTK_ITMIW("itm_giwsens fuc\n");

	ret = itm_get_rssi(&rssi);
	
	if(ret < 0)
	{
		wrqu->sens.value = 0;
		wrqu->sens.fixed = 0;	/* no auto select */
		wrqu->sens.disabled = 1;
	}
	else
	{
		wrqu->sens.value= rssi;
		wrqu->sens.fixed = 0; /* no auto select */ 
	}

	return 0;
}

#if 0 //not use
/*------------------------------------------------------------------*/
/*
 * Standard Wireless Handler : Set range of parameters
 */
static int itm_siwrange(struct net_device *	dev,
			  struct iw_request_info *info,
			   union iwreq_data * wrqu,  char * extra)
{
	/*
	int ret = 0;
	UWORD8 * host_req  = NULL;
	UWORD8 * trout_rsp = NULL;
	UWORD16  trout_rsp_len = 0;
	*/

	PRINTK_ITMIW("itm_siwrange fuc\n");
	
	return 0;
}
#endif
 
/*------------------------------------------------------------------*/
/*
 * Standard Wireless Handler : Get range of parameters
 */
static int itm_giwrange(struct net_device *	dev,
			  struct iw_request_info *info,
			   union iwreq_data * wrqu,  char * extra)
{
	    struct iw_point *dwrq = &wrqu->data;
	    struct iw_range *range = (struct iw_range *) extra;
	    int i;

		CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	    PRINTK_ITMIW("itm_giwrange fuc\n");

	    dwrq->length = sizeof(struct iw_range);
	    memset(range, 0, sizeof(*range));

		//chenq add 2012-12-25
		/* ~5 Mb/s real (802.11b) */
		range->throughput = 5 * 1000 * 1000;  
		
	    range->min_nwid = 0x0000;
	    range->max_nwid = 0x0000;

	    /*
	     * Don't report the frequency/channel table, then the channel
	     * number returned elsewhere will be printed as a channel number.
	     */

	    /* Ranges of values reported in quality structs */
	    range->max_qual.qual  = 40;         /* Max expected qual value */
	    range->max_qual.level = -120;       /* Noise floor in dBm */
	    range->max_qual.noise = -120;       /* Noise floor in dBm */
		//chenq add 2012-12-25
		range->max_qual.updated = 7;


	    /* space for IW_MAX_BITRATES (8 up to WE15, 32 later) */
	    i = 0;
#if WIRELESS_EXT > 15
	    range->bitrate[i++] =   2 * 500000;
	    range->bitrate[i++] =   4 * 500000;
	    range->bitrate[i++] =  11 * 500000;
	    range->bitrate[i++] =  22 * 500000;
	    range->bitrate[i++] =  12 * 500000;
	    range->bitrate[i++] =  18 * 500000;
	    range->bitrate[i++] =  24 * 500000;
	    range->bitrate[i++] =  36 * 500000;
	    range->bitrate[i++] =  48 * 500000;
	    range->bitrate[i++] =  72 * 500000;
	    range->bitrate[i++] =  96 * 500000;
	    range->bitrate[i++] = 108 * 500000;
#else
	    range->bitrate[i++] =   2 * 500000;
	    range->bitrate[i++] =   4 * 500000;
	    range->bitrate[i++] =  11 * 500000;
	    range->bitrate[i++] =  22 * 500000;
	    range->bitrate[i++] =  24 * 500000;
	    range->bitrate[i++] =  48 * 500000;
	    range->bitrate[i++] =  96 * 500000;
	    range->bitrate[i++] = 108 * 500000;
#endif /* WIRELESS_EXT < 16 */
	    range->num_bitrates = i;

		//chenq add 2012-12-25
		range->min_frag = 256U;
		range->max_frag = 7936U;
		//chenq add 2012-12-25
		range->pm_capa = 0;

	    range->max_encoding_tokens = NUM_WEPKEYS;
	    range->num_encoding_sizes = 2;
	    range->encoding_size[0] = 5;
	    range->encoding_size[1] = 13;

	    range->we_version_source = 20;
	    range->we_version_compiled = WIRELESS_EXT;

	    /* Number of channels available in h/w */
	    range->num_channels = 14;
	    /* Number of entries in freq[] array */
	    range->num_frequency = 14;
	    for (i = 0; i < range->num_frequency; i++) {
	        int chan = i + 1;
	        range->freq[i].i = chan;
	        range->freq[i].m = channel_to_mhz(chan, 0);
	        range->freq[i].e = 6;
	    }

#if WIRELESS_EXT > 16
	    /* Event capability (kernel + driver) */
	    range->event_capa[0] = (IW_EVENT_CAPA_K_0 |
	                            IW_EVENT_CAPA_MASK(SIOCGIWTHRSPY) |
	                            IW_EVENT_CAPA_MASK(SIOCGIWAP) |
	                            IW_EVENT_CAPA_MASK(SIOCGIWSCAN));
	    range->event_capa[1] = IW_EVENT_CAPA_K_1;
	    range->event_capa[4] = (IW_EVENT_CAPA_MASK(IWEVTXDROP) |
	                            IW_EVENT_CAPA_MASK(IWEVCUSTOM) |
	                            IW_EVENT_CAPA_MASK(IWEVREGISTERED) |
	                            IW_EVENT_CAPA_MASK(IWEVEXPIRED));
#endif /* WIRELESS_EXT > 16 */

#if WIRELESS_EXT > 17
#ifdef EXT_SUPP_11i  //wxb add: external 11i
	    range->enc_capa = IW_ENC_CAPA_WPA | IW_ENC_CAPA_WPA2 |
	                      IW_ENC_CAPA_CIPHER_TKIP | IW_ENC_CAPA_CIPHER_CCMP
	                      /*| IW_ENC_CAPA_4WAY_HANDSHAKE*/;
#else /* ifdef EXT_SUPP_11i */  //wxb add: internal 11i
	    range->enc_capa = IW_ENC_CAPA_WPA | IW_ENC_CAPA_WPA2 |
	                      IW_ENC_CAPA_CIPHER_TKIP | IW_ENC_CAPA_CIPHER_CCMP
	                      | IW_ENC_CAPA_4WAY_HANDSHAKE;
#endif /* ifdef EXT_SUPP_11i */
#endif /* WIRELESS_EXT > 17 */

#ifdef IW_SCAN_CAPA_ESSID //WIRELESS_EXT > 21
	range->scan_capa = IW_SCAN_CAPA_ESSID | IW_SCAN_CAPA_TYPE |IW_SCAN_CAPA_BSSID|
					IW_SCAN_CAPA_CHANNEL|IW_SCAN_CAPA_MODE|IW_SCAN_CAPA_RATE;
#endif
	
	return 0;
}

/*junbinwang add for set dhcp or dhcp renew to driver. 20131223*/
int dhcp_status = 0;

int itm_get_dhcp_status()
{
	return dhcp_status;
}

void itm_set_dhcp_status(int status)
{
	dhcp_status = status;
	return;
}
static int itm_siwpriv(struct net_device *dev,
				struct iw_request_info *info,
				union iwreq_data *awrq,
				char *extra)
{
	int ret = 0;
	int len = 0;
	char *ext = NULL;
	int i;
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifndef WIFI_SLEEP_POLICY
	//xuan.yang, 2013-09-17, declare  trout_private varibles
	struct trout_private *stp;
#endif
#endif

	struct iw_point *dwrq = (struct iw_point*)awrq;

	//xuanyang 2013.4.24 supplicant timeout handle
//	CHECK_MAC_RESET_IN_IW_HANDLER;

	PRINTK_ITMIW("itm_siwpriv fuc\n");
	//PRINTK_ITMIW("len = %d\n",dwrq->length);

	len = (dwrq->length > 1024) ? 1024 : dwrq->length;
	ext = g_itm_config_buf;
	if ( ext == NULL )
	{
		return -ENOMEM;
	}
	
	if (copy_from_user(ext, dwrq->pointer, len)) 
	{
		return -EFAULT;
	}

	//added for wps2.0
	if(dwrq->flags == 0x8766 && len > 8)
	{
		BUG_ON(1);
		
		goto FREE_EXT;
		
	}
	
	if(	len >= WEXT_CSCAN_HEADER_SIZE
		&& memcmp(ext, WEXT_CSCAN_HEADER, WEXT_CSCAN_HEADER_SIZE) == 0 )
	{
		/*wangjunbin modify for CR179292 2013-06-21*/
		//xuanyang 2013.4.24 supplicant timeout handle
		//CHECK_MAC_RESET_IN_IW_HANDLER;

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifndef WIFI_SLEEP_POLICY
		//xuan.yang, 2013-09-17, check current running space(wid, suspend, resume and so on)
		stp = netdev_priv(dev);
		mutex_lock(&stp->cur_run_mutex);
		if (g_wifi_suspend_status == wifi_suspend_suspend) {
			pr_info("We can't do %s now: g_wifi_suspend_status = %d\n", __func__, g_wifi_suspend_status);
			mutex_unlock(&stp->cur_run_mutex);
			return 0;
		}
		if ( 0 == stp->resume_complete_state) {
			pr_info("We can't do %s now: g_wifi_suspend_status = %d\n", __func__, g_wifi_suspend_status);
			mutex_unlock(&stp->cur_run_mutex);
			return 0;
		}
		printk("%s get cur run mutex ------------ yx \n", __FUNCTION__);
#endif
#endif

		ret = itm_set_scan(dev,awrq,ext);
        if(ret != 0)
        {
            is_scanlist_report2ui = 2;
            send_mac_status(MAC_SCAN_CMP);
            ret = 0;
        }

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifndef WIFI_SLEEP_POLICY
		mutex_unlock(&stp->cur_run_mutex);
		printk("%s release cur run mutex ------------ yx \n", __FUNCTION__);
#endif
#endif
		goto FREE_EXT;
	}
	
#ifdef ANDROID_BUILD
	PRINTK_ITMIW("itm_siwpriv: %s req=%s\n", dev->name, ext);

	/*junbinwang add for set dhcp or dhcp renew to driver. 20131223*/
	if(0 == strncmp(ext , android_wifi_cmd_str[ANDROID_WIFI_CMD_DHCP_STATUS], 
			strlen(android_wifi_cmd_str[ANDROID_WIFI_CMD_DHCP_STATUS])) )
	{
		if(0 == strncmp(ext , "DHCP_STATUS 1",  strlen("DHCP_STATUS 1")))
		{
			printk("[wjb] set dhcp status 1\n");
			itm_set_dhcp_status(1);
		}
		else
		{
			printk("[wjb] set dhcp status 0\n");
			itm_set_dhcp_status(0);
		}

		printk("[wjb]get dhcp status %d\n", itm_get_dhcp_status());
		sprintf(ext, "OK");
		if (copy_to_user(dwrq->pointer, ext, min(dwrq->length, (u16)(strlen(ext)+1)) ) )
			return -EFAULT;
		return 0;
	}
	for(i=0; i<len; i++) {
		if(*(ext+i)>='a' && *(ext+i)<='z' ) {
			*(ext+i)+= 'A'-'a';
		}
	}

	for(i=0 ; i<ANDROID_WIFI_CMD_MAX; i++)
		if(0 == strncmp(ext , android_wifi_cmd_str[i], strlen(android_wifi_cmd_str[i])) )
			break;

	switch(i) {
		case ANDROID_WIFI_CMD_START :
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifdef WIFI_SLEEP_POLICY
			if(g_wifi_suspend_status == wifi_suspend_suspend) { // deep sleep
				g_wifi_suspend_status = wifi_suspend_nosuspend;
			}
#endif
#endif

#if 0
			//xuanyang 2013.4.24 supplicant timeout handle
			CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);
			/* wake netif if needed by zhao */
			if(netif_queue_stopped(dev))
				netif_wake_queue(dev);
            #ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
			trout_sdio_can_sleep(BFALSE); /*notify SDIO could not suspend, by keguang 20130330*/ 
			if(g_wifi_suspend_status == wifi_suspend_suspend) // deep sleep
			{
				sta_wakeup();
			}
            #endif 
#endif
			indicate_custom_event("START");
			break;
		case ANDROID_WIFI_CMD_STOP :
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifdef WIFI_SLEEP_POLICY
			pr_info("======== STOP ========\n");
			//xuanyang 2013.4.24 supplicant timeout handle
			CHECK_MAC_RESET_IN_IW_HANDLER(LPM_NO_ACCESS);

			/* we need stop netif and wait for all packets transmitted by zhao */
			if(dev && !netif_queue_stopped(dev))
				netif_stop_queue(dev);
			//Comment by zhao.zhang
			//wait_for_tx_finsh();
			restart_mac_plus(&g_mac, 0);
			sta_sleep();
			/*clear_tx_barrier();*/
#endif
#endif
#if 0
			//xuanyang 2013.4.24 supplicant timeout handle
			CHECK_MAC_RESET_IN_IW_HANDLER;
			
			/* we need stop netif and wait for all packets transmitted by zhao */
			if(dev && !netif_queue_stopped(dev))
				netif_stop_queue(dev);
			//Comment by zhao.zhang
			//wait_for_tx_finsh();
            #ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
			sta_sleep();
			trout_sdio_can_sleep(BTRUE); /*notify SDIO could suspend, by keguang 20130329*/ 
            #endif
			clear_tx_barrier();
#endif
			indicate_custom_event("STOP");
			break;

		case ANDROID_WIFI_CMD_RSSI :
			{
				UWORD8 essid[IW_ESSID_MAX_SIZE + 1] = {0};
				int rssi = 0;
				
				//xuanyang 2013.4.24 supplicant timeout handle
				if( (BOOL_T)atomic_read(&g_mac_reset_done) == BFALSE )
				{
					PRINTK_ITMIW("%s not do,in reset process\n",__func__);
					sprintf(ext, "OK");
					break;
				}

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifdef WIFI_SLEEP_POLICY
				if (mutex_is_locked(&suspend_mutex) || (g_wifi_suspend_status != wifi_suspend_nosuspend)) {
					pr_info("We can't do %s during suspending, g_wifi_suspend_status = %d\n", __func__, g_wifi_suspend_status);
					break;
				}
#endif
#endif
				#ifdef IBSS_BSS_STATION_MODE
				//if( ( get_mac_state() != ENABLED ) && (g_keep_connection == BFALSE) )
				if( ( get_mac_state() < WAIT_JOIN ) && (g_keep_connection == BFALSE) && (get_mac_state() != ENABLED) )
				{
					sprintf(ext, "OK");
					break;
				}

				if(itm_get_essid(essid,IW_ESSID_MAX_SIZE) < 0)
				{
					sprintf(ext, "OK");
					break;
				}
				#endif
				
				if(itm_get_rssi(&rssi) < 0)
				{
					sprintf(ext, "OK");
					break;
				}

				sprintf(ext, "%s rssi %d", essid, rssi);
			}
			break;
		case ANDROID_WIFI_CMD_LINKSPEED :
			{
				//union iwreq_data wrqd;
				int mbps;

				//xuanyang 2013.4.24 supplicant timeout handle
				if( (BOOL_T)atomic_read(&g_mac_reset_done) == BFALSE )
				{
					PRINTK_ITMIW("%s not do,in reset process\n",__func__);
					sprintf(ext, "LINKSPEED 0");
					break;
				}

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifdef WIFI_SLEEP_POLICY
				if (mutex_is_locked(&suspend_mutex) || (g_wifi_suspend_status != wifi_suspend_nosuspend)) {
					pr_info("We can't do %s during suspending, g_wifi_suspend_status = %d\n", __func__, g_wifi_suspend_status);
					break;
				}
#endif
#endif
				mbps = itm_get_rate();
		
				if( mbps < 0 )
				{
					sprintf(ext, "LINKSPEED 0");
				}
				else if(IS_RATE_MCS(mbps) == BFALSE)
				{
					sprintf(ext, "LINKSPEED %d",mbps);
				}
				else
				{
					int j=0;
					for( j = 0; j < sizeof(rate_table)/sizeof(rate_table_t); j++ )
					{
						if(rate_table[j].rate_index == mbps)
						{
							sprintf(ext, "LINKSPEED %d", rate_table[j].rate);
							if(rate_table[j].point5)
							{
								strcat(ext,".5");
							}
							break;
						}
					}

					if( j >= sizeof(rate_table)/sizeof(rate_table_t) )
					{
						sprintf(ext, "LINKSPEED 0");
					}
				}				
			}
			break;
		case ANDROID_WIFI_CMD_MACADDR :
			{
				UWORD8 dev_addr[6] = {0};
			
				//xuanyang 2013.4.24 supplicant timeout handle
				if( (BOOL_T)atomic_read(&g_mac_reset_done) == BFALSE )
				{
					PRINTK_ITMIW("%s not do,in reset process\n",__func__);
				    sprintf(ext, "MACADDR = " ITM_MAC_FMT, ITM_MAC_ARG(mget_StationID()));
					break;
				}
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifdef WIFI_SLEEP_POLICY
				if (mutex_is_locked(&suspend_mutex) || (g_wifi_suspend_status != wifi_suspend_nosuspend)) {
					pr_info("We can't do %s during suspending, g_wifi_suspend_status = %d\n", __func__, g_wifi_suspend_status);
					break;
				}
#endif
#endif
				itm_get_devmac(dev_addr);
			
				sprintf(ext, "MACADDR = " ITM_MAC_FMT, ITM_MAC_ARG(dev_addr));
			}
			break;
		case ANDROID_WIFI_CMD_SCAN_ACTIVE :
			{
				sprintf(ext, "OK");
			}
			break;
		case ANDROID_WIFI_CMD_SCAN_PASSIVE :
			{
				sprintf(ext, "OK");
			}
			break;

		case ANDROID_WIFI_CMD_COUNTRY :
			{
				char country_code[10];
				int channel_plan = 0;
				int scan_channel_no = 0;
				//union iwreq_data wrqd;
				//int ret_inner;


				if(strcmp("COUNTRY", ext) == 0)
				{
					printk("cmd country_code get max channel number\n");
					
					sprintf(ext, "SCAN-CHANNELS = %u", itm_get_max_ch_no());
					break;
				}

				
				sscanf(ext,"%*s %s",country_code);
				
				if(0 == strcmp(country_code, "US")){
					channel_plan = 0;
                    scan_channel_no = 11;
				}else if(0 == strcmp(country_code, "EU")){
					channel_plan = 1;
                    scan_channel_no = 13;
                }else if(0 == strcmp(country_code, "JP")){
					channel_plan = 1;
                    scan_channel_no = 14;
				}else{
					PRINTK_ITMIW("%s unknown country_code:%s, set to RT_CHANNEL_DOMAIN_FCC\n", __FUNCTION__, country_code);
                    scan_channel_no = 11;
                }

				// 0502 caisf mod for support different scan channel numbers
#if 0
                channel_plan = 1;
				if( itm_set_domain(channel_plan, scan_channel_no) < 0 )
				{
					PRINTK_ITMIW("%s rtw_wx_set_channel_plan error\n", __FUNCTION__);
				}
#endif
				// reply current domain max channel number 
				sprintf(ext, "SCAN-CHANNELS = %u", itm_get_max_ch_no());
			}
			break;
			
		default :
			sprintf(ext, "OK");
		
	}

	if (copy_to_user(dwrq->pointer, ext, min(dwrq->length, (u16)(strlen(ext)+1)) ) )
		ret = -EFAULT;
#endif //end of CONFIG_ANDROID

FREE_EXT:
	return ret;
	
}

/*------------------------------------------------------------------*/
/*
 * Standard Wireless Handler : set access point MAC addresses
 */

static int itm_siwap(struct net_device *	dev,
			  struct iw_request_info *info,
			   union iwreq_data * wrqu,  char * extra)
{
	UWORD8   ap_addr[6] = {0x00,0x00,0x00,0x00,0x00,0x00};
	int flag = 0;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	PRINTK_ITMIW("itm_siwap fuc\n");
	PRINTK_ITMIW("itm_siwap fuc ap_addr.sa_family:%d\n",wrqu->ap_addr.sa_family);

	if (wrqu->ap_addr.sa_family != ARPHRD_ETHER) 
	{
       	return -EINVAL;
    }

	//flag = memcmp(ap_addr,wrqu->ap_addr.sa_data,6);
	memcpy(ap_addr,wrqu->ap_addr.sa_data,6);

	PRINTK_ITMIW("itm_siwap: asked for %02X:%02X:%02X:%02X:%02X:%02X\n",
                (u8)ap_addr[0],
                (u8)ap_addr[1],
                (u8)ap_addr[2],
                (u8)ap_addr[3],
                (u8)ap_addr[4],
                (u8)ap_addr[5]);

	//PRINTK_ITMIW("itm_siwap: will not do this return\n");
	//return 0;

	if(flag == 0)
	{
		PRINTK_ITMIW("will set ssid\n");
    	if( itm_set_bssid(ap_addr) < 0 )	
    	{
        	return -EINVAL;
    	}
    	else 
    	{
        	return 0;
    	}
	}
	else
	{
		PRINTK_ITMIW("will not set ssid,driver will do this\n");
		return 0;
	}
}

/*------------------------------------------------------------------*/
/*
 * Standard Wireless Handler : get access point MAC addresses
 */
static int itm_giwap(struct net_device *	dev,
			  struct iw_request_info *info,
			   union iwreq_data * wrqu,  char * extra)
{
	int ret = 0;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);	
	
	PRINTK_ITMIW("itm_giwap fuc\n");

	if((ret = itm_get_bssid((UWORD8 * )(wrqu->ap_addr.sa_data))) >= 0)	
	{
		wrqu->ap_addr.sa_family = ARPHRD_ETHER;
	}

	return ret;
}

 

/* WPA : IEEE 802.11 MLME requests */
/*------------------------------------------------------------------*/
/*
 * Standard Wireless Handler :  request MLME operation
 */
static int itm_siwmlme(struct net_device *	dev,
			  struct iw_request_info *info,
			   union iwreq_data * wrqu,  char * extra)
{
	//int ret = 0;
	//UWORD8 * host_req  = NULL;
	//UWORD16  trout_rsp_len = 0;
	//UWORD8  dis_value = 1;
	struct iw_mlme * mlme = (struct iw_mlme *) extra;
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifndef WIFI_SLEEP_POLICY
	struct trout_private *stp;
#endif
#endif

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_NO_ACCESS);

	PRINTK_ITMIW("itm_siwmlme fuc\n");

	//return 0; //NULL chenq mod 0801
	PRINTK_ITMIW("itm_siwmlme mlme->code = %d\n",mlme->reason_code);

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifndef WIFI_SLEEP_POLICY
	//xuan.yang, 2013-09-17, get run metux lock
	stp = netdev_priv(dev);
    mutex_lock(&stp->cur_run_mutex);

	//check suspend state
	if (g_wifi_suspend_status == wifi_suspend_suspend) {
		pr_info("We can't do %s now: g_wifi_suspend_status = %d\n", __func__, g_wifi_suspend_status);
		mutex_unlock(&stp->cur_run_mutex);
		return -EBUSY;
	}

	//check resume state
	if (0 == stp->resume_complete_state) {
		pr_info("We can't do %s now: g_wifi_suspend_status = %d\n", __func__, g_wifi_suspend_status);
		mutex_unlock(&stp->cur_run_mutex);
		return -EBUSY;
	}
	printk("%s get cur run mutex ------------ yx \n", __FUNCTION__);
#endif
#endif

	switch (mlme->cmd) 
	{
		case IW_MLME_DEAUTH:			
				/*  do disassoc action*/
				break;
				
		case IW_MLME_DISASSOC:			
				/*  do disassoc action*/		
				break;
		default:
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifndef WIFI_SLEEP_POLICY
			mutex_unlock(&stp->cur_run_mutex);
			printk("%s release cur run mutex ------------ yx \n", __FUNCTION__);
#endif
#endif
			return -EOPNOTSUPP;
	}

	//reset_mac(&g_mac, BTRUE);

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
        if(get_ps_state() == STA_DOZE)
        {
            pr_info("-------------------God!!!----------------------");
           sta_awake(LOCK_TURE);
           /*set_ps_state(STA_ACTIVE);*/
        }
#endif
#ifdef IBSS_BSS_STATION_MODE
	if( itm_scan_flag == 1 )
	{
		PRINTK_ITMIW("itm_siwmlme now is scanning \n");
	    restart_mac_plus(&g_mac, 0);
		is_scanlist_report2ui = 2;
	    send_mac_status(MAC_SCAN_CMP);
	}
	else
	{
		restart_mac_plus(&g_mac, 0);
	}
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifndef WIFI_SLEEP_POLICY
	mutex_unlock(&stp->cur_run_mutex);
	printk("%s release cur run mutex ------------ yx \n", __FUNCTION__);
#endif
#endif
	
	return 0;
#else

	restart_mac_plus(&g_mac, 0);

	return 0;
#endif
	#if 0
	if(itm_set_essid(NULL,0) < 0)
	{
		return -1;
	}
	else
	{
		send_mac_status(MAC_CTL_OK);
		return 0;
	}

	
	/*  do disassoc action*/	
	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if( host_req == NULL )
	{
		return -ENOMEM;
	}
	
	host_req[0] = WID_DISCONNECT & 0xFF;
	host_req[1] = (WID_DISCONNECT & 0xFF00) >> 8;
	host_req[2] = 1;
	host_req[3] = dis_value;

	config_if_for_iw(&g_mac,host_req,WID_CHAR_CFG_LEN,
								'W',&trout_rsp_len);

	if( trout_rsp_len != 1 )
	{
		ret = -EINVAL;
		goto out1;
	}
	
	out1:
	//kfree(host_req);
	return ret;
	#endif
}


/*------------------------------------------------------------------*/
/*
 * Standard Wireless Handler : trigger scanning (list cells)
 */

static int itm_siwscan(struct net_device *	dev,
			  struct iw_request_info *info,
			   union iwreq_data * wrqu,  char * extra)
{
#ifdef IBSS_BSS_STATION_MODE
	int ret = 0;
	UWORD8 * host_req  = NULL;
	UWORD16  trout_rsp_len = 0;
	//struct  iw_point * data = ( struct iw_point * )wrqu;
	int i = 0;
	int len = 0;
	int scan_type = 0;
#if WIRELESS_EXT > 17
    struct iw_point *data = &wrqu->data;
    struct iw_scan_req *req = (struct iw_scan_req *) extra;
#endif
	struct in_device *my_ip_ptr = dev->ip_ptr;

	UWORD8 cur_essid[IW_ESSID_MAX_SIZE+1] = {0};
	int cur_essid_len = 0;

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifndef WIFI_SLEEP_POLICY
	//xuan.yang, 2013-09-17, declare  trout_private varibles
	struct trout_private *stp;
#endif
#endif

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	PRINTK_ITMIW("itm_siwscan fuc\n");

    PRINTK_ITMIW("req->num_channels = %d,req->num_channels = %d,scan_type = %d\n",
                 req->num_channels,req->num_channels,
                 (data->flags & IW_SCAN_THIS_ESSID) ? 1 : 0);

	if( ( get_mac_state() == ENABLED ) || (g_keep_connection == BTRUE) )
	{
		if(my_ip_ptr != NULL)
		{
			struct in_ifaddr *my_ifa_list = my_ip_ptr->ifa_list;
			if(my_ifa_list != NULL)
			{
				goto CHECK_SCAN;
			}
		}

		PRINTK_ITMIW("skip scan op <ip addr less> \n");
		//chenq mod 2013-05-23
		return 0;
		//return -EAGAIN;
	}

	cur_essid_len = itm_get_essid(cur_essid,IW_ESSID_MAX_SIZE);
	if(cur_essid_len)
	{
		PRINTK_ITMIW("skip scan op <%s> \n",
			(cur_essid_len < 0) ? "get cur essid err":"wifi linking" );
		return -EAGAIN;
	}

CHECK_SCAN:
	if( itm_scan_flag == 1 )
	{
		PRINTK_ITMIW("skip scan op <already in scan> \n");
		return 0;
	}
	else if(g_BusyTraffic == BTRUE)
	{
		PRINTK_ITMIW("skip scan op <BusyTraffic> \n");
		return 0;
	}


    if (req && (data->flags & IW_SCAN_THIS_ESSID))
    {
    	PRINTK_ITMIW("itm_siwscan fuc:set essid:");
        for( i = 0;i < req->essid_len;i++)
        	PRINTK_ITMIW("%c",req->essid[i]);

		PRINTK_ITMIW("\n");

		PRINTK_ITMIW("we will not config in this hander !!!\n");

		#if 0
		if( itm_set_essid( (UWORD8*)req->essid,(int)req->essid_len ) < 0 )
    	{
        	return -EINVAL;
    	}
    	else 
    	{
        	scan_type = 1;
    	}
		#else
		scan_type = 1;
		#endif
    }
	else
	{
		//PRINTK_ITMIW("itm_siwscan fuc:clear essid \n");
		//if( itm_set_essid( NULL,0 ) < 0 )
		if(0)
    	{
        	return -EINVAL;
    	}
    	else 
    	{
        	scan_type = 1;
    	}
	}

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if(host_req == NULL)
	{
		return -ENOMEM;
	}

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifndef WIFI_SLEEP_POLICY
	//xuan.yang, 2013-09-17, check current running space(wid, suspend, resume and so on)
	stp = netdev_priv(dev);
    mutex_lock(&stp->cur_run_mutex);
	if (g_wifi_suspend_status == wifi_suspend_suspend) {
		pr_info("We can't do %s now: g_wifi_suspend_status = %d\n", __func__, g_wifi_suspend_status);
		mutex_unlock(&stp->cur_run_mutex);
		return -EBUSY;
	}
	if ( 0 == stp->resume_complete_state) {
		pr_info("We can't do %s now: g_wifi_suspend_status = %d\n", __func__, g_wifi_suspend_status);
		mutex_unlock(&stp->cur_run_mutex);
		return -EBUSY;
	}
	printk("%s get cur run mutex ------------ yx \n", __FUNCTION__);
#endif
#endif

#if 1

#if 0
	host_req[len+0] = WID_BCAST_SSID & 0xFF;
    host_req[len+1] = (WID_BCAST_SSID & 0xFF00) >> 8;
	host_req[len+2] = 1; 
	host_req[len+3] = 1; // 1:not compare ssid
	len += WID_CHAR_CFG_LEN;
#endif

    host_req[len+0] = WID_SCAN_TYPE & 0xFF;
    host_req[len+1] = (WID_SCAN_TYPE & 0xFF00) >> 8;
	host_req[len+2] = 1; 
	host_req[len+3] = scan_type & 0xFF; // 1:  Active Scanning   0:pass
	len += WID_CHAR_CFG_LEN;

#if 0 //chenq mask 20120903	
	host_req[len+0] = WID_SITE_SURVEY & 0xFF;
	host_req[len+1] = (WID_SITE_SURVEY & 0xFF00) >> 8;
	host_req[len+2] = 1; 
	host_req[len+3] = 1; //all channel
	len += WID_CHAR_CFG_LEN;
#endif
	
#if 0    
	trout_rsp = config_if_for_iw(&g_mac,host_req,WID_CHAR_CFG_LEN,'W',&trout_rsp_len);

	if( (trout_rsp == NULL) || (trout_rsp_len < 1) )
	{
		ret = -EINVAL;
		goto out1;
	}
#endif    
#endif
	
	host_req[len+0] = WID_START_SCAN_REQ & 0xFF;
	host_req[len+1] = (WID_START_SCAN_REQ & 0xFF00) >> 8;
	host_req[len+2] = 1;
	host_req[len+3] = 1; //trigger scanning
	len += WID_CHAR_CFG_LEN;

	host_req[len+0] = WID_RESET & 0xFF;
	host_req[len+1] = (WID_RESET & 0xFF00) >> 8;
	host_req[len+2] = 1;
	host_req[len+3] = 0;//( (g_mac.state == ENABLED) || (g_keep_connection == BTRUE) ) ? 0 : 2; //DONT_RESET
	len += WID_CHAR_CFG_LEN;

	config_if_for_iw(&g_mac,host_req,len,'W',&trout_rsp_len);

	if( trout_rsp_len != 1 )
	{
		ret = -EINVAL;
		goto out1;
	}
	
	out1:
	//kfree(host_req);
	if(itm_scan_flag == 1)
	{
		if(!ret)
		{
			is_scanlist_report2ui = 1;
			//itm_scan_task = current;
			//set_current_state(TASK_INTERRUPTIBLE); 
			//printk("chenq debug itm_siwscan %p\n",itm_scan_task);
			//schedule();
			//is_scanlist_report2ui = 0;

			//chenq debug
			//msleep(500);
		}
		else
		{
			ret = -EBUSY;
		}	
	}
	else
	{
		ret = -EINVAL;
	}

        if(ret == 0)
            msleep(1500);

	PRINTK_ITMIW("itm_siwscan fuc end code:%d\n",ret);
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifndef WIFI_SLEEP_POLICY
	mutex_unlock(&stp->cur_run_mutex);
		printk("%s release cur run mutex ------------ yx \n", __FUNCTION__);
#endif
#endif
	return ret;

	#else //ap mode

	PRINTK_ITMIW("itm_siwscan fuc\n");

	return -ESRCH;
	#endif
}

/*------------------------------------------------------------------*/
/*
 * Standard Wireless Handler : get scanning results
 */
//chenq mod do ap list merge logic 2013-08-28
static int itm_giwscan(struct net_device *	dev,
			  struct iw_request_info *info,
			   union iwreq_data * wrqu,  char * extra)
{
	#ifdef IBSS_BSS_STATION_MODE

    char *ev = extra;
	int i =0;
	int ret = 0;
	struct iw_point * dwrq = &wrqu->data;
	bss_link_dscr_t* bss = NULL;
		
	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);	
	
	PRINTK_ITMIW("itm_giwscan fuc (sta mode)\n");

	if( itm_scan_task != NULL )
	{
		wake_up_process(itm_scan_task);
		itm_scan_task = NULL;
	}

	if(is_scanlist_report2ui == 1)
	{
		msleep(1000);
		return -E2BIG;
		//return -EAGAIN;
	}
	
	bss = g_user_getscan_aplist;
	
	if(NULL == g_user_getscan_aplist )
	{
		TROUT_DBG4("linklist is null\n");
		return -E2BIG;
	}

	while( bss != NULL )
	{
	    ev = itm_translate_scan(dev, info, ev,extra + dwrq->length,
	                                            bss);
		if(!ev)
		{
			break;
		}

	    if( (extra + dwrq->length - ev) <= IW_EV_ADDR_LEN )
	    {
	        /*Ask user space to try again with a bigger buffer */
	        ret = -E2BIG;
	        //goto out2;
	        return ret;
	    }
		
	    i++;
	    bss=bss->bss_next;

		//chenq add ,,,too big (max 16 ap info)2013-02-08
		//if(i >= 16)
			//break;
	}

	PRINTK_ITMIW("itm_giwscan fuc report to UI %d ap info\n",i);
	dwrq->length = ev - extra;
	dwrq->flags = 0;

	return ret;

	#else//ap mode

	PRINTK_ITMIW("itm_giwscan fuc (ap mode)\n"); 
	return -ESRCH;
	
	#endif
}		

/*------------------------------------------------------------------*/
/*
 * Standard Wireless Handler : set ESSID (network name)
 */
static int itm_siwessid(struct net_device *	dev,
			  struct iw_request_info *info,
			   union iwreq_data * wrqu,  char * extra)
{
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifndef WIFI_SLEEP_POLICY
	struct trout_private *stp;
#endif
#endif
	struct iw_point * data = (struct iw_point *)wrqu;
	int len=0;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifndef WIFI_SLEEP_POLICY
	//xuan.yang, 2013-09-17, get run metux log
	stp = netdev_priv(dev);
    mutex_lock(&stp->cur_run_mutex);

	//check suspend state
	if (g_wifi_suspend_status == wifi_suspend_suspend) {
		pr_info("We can't do %s now: g_wifi_suspend_status = %d\n", __func__, g_wifi_suspend_status);
		mutex_unlock(&stp->cur_run_mutex);
		return -EBUSY;
	}

	//check resume state
	if (0 == stp->resume_complete_state) {
		pr_info("We can't do %s now: g_wifi_suspend_status = %d\n", __func__, g_wifi_suspend_status);
		mutex_unlock(&stp->cur_run_mutex);
		return -EBUSY;
	}
	printk("%s get cur run mutex ------------ yx \n", __FUNCTION__);
#endif
#endif

	PRINTK_ITMIW("\n\n\n");
    PRINTK_ITMIW("itm_siwessid fuc\n");
    //PRINTK_ITMIW("data->flags = 0x%x\n",data->flags);
    //for(len = 0;len < data->length;len++)
        //PRINTK_ITMIW("%02x,",*(extra+len) & 0xFF);
    
    //PRINTK_ITMIW("\n");
    
    for(len = 0;len < data->length;len++)
        PRINTK_ITMIW("%c,",*(extra+len) & 0xFF);

    //PRINTK_ITMIW("\n");
	//PRINTK_ITMIW("itm_siwessid fuc %d \n",data->length);
	PRINTK_ITMIW("\n\n\n");

	#if 0
	if( itm_set_Scan_Type( (UWORD8)1 ) < 0 )
    {
        return -EINVAL;
    }
    else 
    {
        PRINTK_ITMIW("itm_siwessid fuc set scan type 1 \n");
    }
	#endif
	

    if( itm_set_essid( (UWORD8*)extra,(int)data->length ) < 0 )
    {
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifndef WIFI_SLEEP_POLICY
		mutex_unlock(&stp->cur_run_mutex);
		printk("%s release cur run mutex ------------ yx \n", __FUNCTION__);
#endif
#endif
        return -EINVAL;
    }
    else 
    {
        //PRINTK_ITMIW("itm_siwessid fuc set essid <%s>  \n",extra);
    }

	return 0;
}		
    
/*------------------------------------------------------------------*/
/*
 * Standard Wireless Handler : get ESSID
 */
static int itm_giwessid(struct net_device *	dev,
			  struct iw_request_info *info,
			   union iwreq_data * wrqu,  char * extra)
{
	int ret = 0;
	struct iw_point * data = (struct iw_point *)wrqu;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	PRINTK_ITMIW("itm_giwessid fuc \n");

	ret = itm_get_essid(extra,IW_ESSID_MAX_SIZE);

	if(ret >= 0)
	{
		data->length = ret;
		memcpy(data->pointer,extra,data->length);
		data->flags = 1;
	}
	
	return ret;
}

/*------------------------------------------------------------------*/
/*
 * Standard Wireless Handler : set default bit rate (bps)
 */

static int itm_giwnick(struct net_device *	dev,
			  struct iw_request_info *info,
			   union iwreq_data * wrqu,  char * extra)
{
	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_NO_ACCESS);

	PRINTK_ITMIW("itm_giwnick fuc\n");

	if(extra)
	{
		wrqu->data.length = 14;
		wrqu->data.flags = 1;
		memcpy(extra, "<WIFI@ITM>", 10);
	}

	return 0;
}




#define TROUT_COMMON_BASE   (0x4000U << 2)
#define TCBA	TROUT_COMMON_BASE
#define TCBE	(TCBA + (0xFF << 2))

#define TROUT_SYS_BASE		(0x0)

void dump_comm_regs(struct file *filp)
{
	unsigned int i,v;
	int size = 0;
	char buff[100] = {0};

	mem_set(buff, 0, 100);
	size = sprintf(buff,"%s","************COMMON REGS*****************\n");
	filp->f_op->write(filp, buff, size, &filp->f_pos);
	for(i = 0; i < 0xff; i++)
	{
		v = convert_to_le(host_read_trout_reg((UWORD32)(TCBA + (i << 2))));
		mem_set(buff, 0, 100);
		size = sprintf(buff,"0x%04X: 0x%08X,\n", i, v);
		filp->f_op->write(filp,buff,size,&filp->f_pos);
	}
}

void dump_sys_regs(struct file *filp)
{
	unsigned int i,v;
	char buff[100] = {0};
	int size;

	mem_set(buff, 0, 100);
	size = sprintf(buff,"%s","************SYS REGS*****************\n");
	filp->f_op->write(filp,buff,size,&filp->f_pos);
	for(i = 0; i < 0xff; i++)
	{
		v = convert_to_le(host_read_trout_reg((UWORD32)((i << 2))));
		mem_set(buff, 0, 100);
		size = sprintf(buff,"0x%04X: 0x%08X,\n", i, v);
		filp->f_op->write(filp,buff,size,&filp->f_pos);
	}
}


void dump_rf_regs(struct file *filp)
{
	unsigned int i, v;
	int size = 0;
	char buff[100] = {0};

	mem_set(buff, 0, 100);
	size = sprintf(buff,"%s","************RF REGS*****************\n");
	size = filp->f_op->write(filp,buff,size,&filp->f_pos);
	for(i = 0; i < 0x4EF; i++)
	{
		v = convert_to_le(host_read_trout_reg((UWORD32)((0x1000 + i) << 2)));
		mem_set(buff, 0, 100);
		size = sprintf(buff,"0x%04X: 0x%08X,\n", i, v);
		size = filp->f_op->write(filp,buff,size,&filp->f_pos);
	}
}

static void write_regs_file(UWORD32 startAdr, UWORD32 endAdr,
								struct file *filp)
{
	UWORD32 i = 0;
	UWORD32 v = 0;
	char buff[100];
	int size;
	
    for (i = startAdr; i <= endAdr;)
    {
	   mem_set(buff, 0, 100);
	   v = host_read_trout_reg(i);
	   size = sprintf(buff,"0x%04x: %08x,\n", i, v);
	   filp->f_op->write(filp,buff,size,&filp->f_pos);
	   i = i + 4;
    }
}

static void dump_mac_regs(struct file *filp)
{
	//UWORD32 i = 0;
	//UWORD32 v = 0;
	int size = 0;
	char buff[100];
	
    TROUT_FUNC_ENTER;

    mem_set(buff, 0, 100);
	size = sprintf(buff,"%s","MAC General Registers \n");
	size = filp->f_op->write(filp,buff,size,&filp->f_pos);
	write_regs_file(rMAC_PA_VER,rMAC_EXTENDED_PA_CON,filp);

	mem_set(buff, 0, 100);
	size = sprintf(buff,"%s","MAC Reception Registers \n");
	size = filp->f_op->write(filp,buff,size,&filp->f_pos);
	write_regs_file(rMAC_RX_FRAME_FILTER,rMAC_MAX_RX_BUFFER_LEN,filp);

	mem_set(buff, 0, 100);
	size = sprintf(buff,"%s","MAC EDCA Registers  \n");
	size = filp->f_op->write(filp,buff,size,&filp->f_pos);
	write_regs_file(rMAC_AIFSN,rMAC_EDCA_VI_VO_LIFETIME,filp);

	mem_set(buff, 0, 100);
	size = sprintf(buff,"%s","MAC HCCA STA Registers \n");
	size = filp->f_op->write(filp,buff,size,&filp->f_pos);
	write_regs_file(rMAC_HC_STA_PRI0_Q_PTR,rMAC_HC_STA_PRI7_Q_PTR,filp);

	mem_set(buff, 0, 100);
	size = sprintf(buff,"%s","MAC TSF Registers\n");
	size = filp->f_op->write(filp,buff,size,&filp->f_pos);
	write_regs_file(rMAC_TSF_CON,rMAC_BEACON_PHY_TX_MODE,filp);

	mem_set(buff, 0, 100);
	size = sprintf(buff,"%s","MAC  Protection And SIFS Response Registers \n");
	size = filp->f_op->write(filp,buff,size,&filp->f_pos);
	write_regs_file(rMAC_PROT_CON,rTX_NUM_TXOP_ABORT_ON_SEC_BUSY,filp);

	mem_set(buff, 0, 100);
	size = sprintf(buff,"%s","MAC Channel Access Timer Management Registers \n");
	size = filp->f_op->write(filp,buff,size,&filp->f_pos);
	write_regs_file(rMAC_SLOT_TIME,rMAC_RIFS_TIME_CONTROL_REG,filp);
	

	mem_set(buff, 0, 100);
	size = sprintf(buff,"%s","Retry Registers	\n");
	size = filp->f_op->write(filp,buff,size,&filp->f_pos);
	write_regs_file(rMAC_LONG_RETRY_LIMIT,rMAC_SHORT_RETRY_LIMIT,filp);


	mem_set(buff, 0, 100);
	size = sprintf(buff,"%s","Sequence Number and Duplicate Detection Registers	\n");
	size = filp->f_op->write(filp,buff,size,&filp->f_pos);
	write_regs_file(rMAC_SEQ_NUM_CON,rMAC_TX_SEQ_NUM,filp);

	mem_set(buff, 0, 100);
	size = sprintf(buff,"%s","PCF Registers \n");
	size = filp->f_op->write(filp,buff,size,&filp->f_pos);
	write_regs_file(rMAC_PCF_CON,rMAC_UNUSED_CFP_DUR,filp);

	
	mem_set(buff, 0, 100);
	size = sprintf(buff,"%s","Power Management Registers \n");
	size = filp->f_op->write(filp,buff,size,&filp->f_pos);
	write_regs_file(rMAC_PM_CON,rMAC_SMPS_CONTROL,filp);

	mem_set(buff, 0, 100);
	size = sprintf(buff,"%s"," MAC Interrupt Registers  \n");
	size = filp->f_op->write(filp,buff,size,&filp->f_pos);
	write_regs_file(rMAC_INT_STAT,rMAC_ERROR_MASK,filp);

	mem_set(buff, 0, 100);
	size = sprintf(buff,"%s","PHY Interface and Parameters Register \n");
	size = filp->f_op->write(filp,buff,size,&filp->f_pos);
	write_regs_file(rMAC_PHY_REG_ACCESS_CON,rMAC_ANTENNA_SET,filp);

	mem_set(buff, 0, 100);
	size = sprintf(buff,"%s","Block Ack register address \n");
	size = filp->f_op->write(filp,buff,size,&filp->f_pos);
	write_regs_file(rMAC_BA_CTRL,rMAC_BA_CBMAP_LSW,filp);

	mem_set(buff, 0, 100);
	size = sprintf(buff,"%s","HCCA AP Registers  \n");
	size = filp->f_op->write(filp,buff,size,&filp->f_pos);
	write_regs_file(rMAC_SCHEDULE_LINK_ADDR,rMAC_CAP_START_TIME,filp);

	mem_set(buff, 0, 100);
	size = sprintf(buff,"%s","Queue pointer addresses \n");
	size = filp->f_op->write(filp,buff,size,&filp->f_pos);
	write_regs_file(MAC_EDCA_PRI_BK_Q_PTR,MAC_HC_STA_PRI7_Q_PTR,filp);

}


void dump_phy_regs(struct file *filp)
{
	UWORD8 i = 0;
	UWORD32 v = 0;
	int size = 0;
	char buff[100];
	
    TROUT_FUNC_ENTER;
    write_dot11_phy_reg(0xFF, 0x00);
    mem_set(buff, 0, 100);
	size = sprintf(buff,"%s","PHY table 1\n");
	size = filp->f_op->write(filp,buff,size,&filp->f_pos);
	
    for (i = 0; i <= MAX_PHY_ADDR; i++)
    {
      read_dot11_phy_reg(i, &v);
	
	   mem_set(buff, 0, 100);
	   size = sprintf(buff,"0x%04x: %08x,\n", i, v);
	   size = filp->f_op->write(filp,buff,size,&filp->f_pos);
    }

    write_dot11_phy_reg(0xFF, 0x01);
	mem_set(buff, 0, 100);
	size = sprintf(buff,"%s","PHY table 2\n");
	size = filp->f_op->write(filp,buff,size,&filp->f_pos);
    for (i = 0; i <= MAX_PHY_ADDR2; i++)
    {
        read_dot11_phy_reg(i, &v);
	mem_set(buff, 0, 100);
	size = sprintf(buff,"0x%04x: 0x%08x,\n", i, v);
	size = filp->f_op->write(filp,buff,size,&filp->f_pos);
    }
}


void dump_allregs(unsigned long ix, unsigned long iy)
{
#if 1
	struct file *filp; 
	mm_segment_t fs;
	unsigned char filename[24] = "/data/allregs";
	static int count = 0;
	int size = 0;
	char buff[100] = {0};
	

	sprintf(&filename[13], "%d_%d.txt", ix, iy);
	printk("dump_allregs: save all reg info to file %s\n", filename);
		
	filp = filp_open(filename, O_CREAT |O_RDWR | O_TRUNC, 0644);
	if(IS_ERR(filp)){
	      printk("dump_allregs: open error...\n");
	      return;
	}
	
	fs=get_fs();
    set_fs(KERNEL_DS);
	
	size += sprintf(buff, "dump_allregs: open times = %d\n", count++);
	size = filp->f_op->write(filp, buff, size, &filp->f_pos);
	
#endif
	dump_comm_regs(filp);
	dump_sys_regs(filp);
	dump_mac_regs(filp);
	dump_phy_regs(filp);
	dump_rf_regs(filp);
	 
    filp_close(filp, NULL);
    set_fs(fs);
	
	printk("dump_allregs: dump regs  end...\n");
}




unsigned int stop_wr = 0;

void data_to_file(unsigned char *buf, unsigned long len)
{
	struct file *filp; 
	mm_segment_t fs;
	unsigned char filename[] = "/data/dl_data.txt";
	int size = 0;

	filp = filp_open(filename, O_CREAT |O_RDWR | O_TRUNC, 0644);
	if(IS_ERR(filp)){
	      printk("open dl_data: open error...\n");
	      return;
	}
	fs=get_fs();
	set_fs(KERNEL_DS);
	size = filp->f_op->write(filp, buf, len, &filp->f_pos);
	if(size != len)
		printk("data_to_file!, wtire %d Dword!\n", size);

	filp_close(filp, NULL);
	set_fs(fs);
	printk("data write to /data/dl_data.txt\n");

}

unsigned long sz = 0;
char	dl_buf[512];
unsigned long	dl_nr = 0;
void dump_dl_data(void)
{
	unsigned long	dl_data[8];
	unsigned long	idx = 0;

	switch(dl_nr)
	{
	case 0:
		memset((char *)dl_data, 0, 32);
		memset((char *)dl_buf, 0, 256);
		dl_data[idx++]  = convert_to_le(host_read_trout_reg((rMAC_FCS_FAIL_COUNT)));
		dl_data[idx++]  = convert_to_le(host_read_trout_reg((rMAC_RX_END_COUNT)));
		dl_data[idx++]  = convert_to_le(host_read_trout_reg((rMAC_RX_ERROR_END_COUNT)));
		dl_data[idx++]  = convert_to_le(host_read_trout_reg((rMAC_RX_FRAME_FILTER_COUNTER)));
		dl_data[idx++]  = convert_to_le(host_read_trout_reg((rMAC_RX_MAC_HEADER_FILTER_COUNTER)));
		dl_data[idx++]  = convert_to_le(host_read_trout_reg((rMAC_RXQ_FULL_COUNTER)));

	   	sz = sprintf(dl_buf,"start: FCS_FAIL:%08x, RX_END:%08X, RX_END_ERR:%08x, 0x8050:%08X, 0x8054:%08X, 0x8058:%08X\n", 
			dl_data[0], dl_data[1], dl_data[2], dl_data[3],  dl_data[4], dl_data[5]);
		dl_nr++;	
		break;
	case 1:
		dl_data[idx++]  = convert_to_le(host_read_trout_reg((rMAC_FCS_FAIL_COUNT)));
		dl_data[idx++]  = convert_to_le(host_read_trout_reg((rMAC_RX_END_COUNT)));
		dl_data[idx++]  = convert_to_le(host_read_trout_reg((rMAC_RX_ERROR_END_COUNT)));
		dl_data[idx++]  = convert_to_le(host_read_trout_reg((rMAC_RX_FRAME_FILTER_COUNTER)));
		dl_data[idx++]  = convert_to_le(host_read_trout_reg((rMAC_RX_MAC_HEADER_FILTER_COUNTER)));
		dl_data[idx++]  = convert_to_le(host_read_trout_reg((rMAC_RXQ_FULL_COUNTER)));
	   	sz = sprintf(&dl_buf[sz],"end: FCS_FAIL:%08x, RX_END:%08X, RX_END_ERR:%08x, 0x8050:%08X, 0x8054:%08X, 0x8058:%08X\n", 
			dl_data[0], dl_data[1], dl_data[2], dl_data[3],  dl_data[4], dl_data[5]);
		data_to_file(dl_buf, 256);
		dl_nr = 0;
		sz = 0;
		break;
	default:
		printk("dump_dl error!\n");
		break;
	}
}


static int itm_siwrate(struct net_device *	dev,
			  struct iw_request_info *info,
			   union iwreq_data * wrqu,  char * extra)
{
	int ret = 0;
	UWORD8 * host_req  = NULL;
	UWORD16  trout_rsp_len = 0;
	struct iw_param *args = &wrqu->bitrate;
	
	printk("********************************************\n");
	printk("0: no debug statistics\n");
	printk("1: show rx statistics info\n");
	printk("2: clear rx statistics info\n");
	printk("3: print recv frame detail\n");
	printk("4: show all associate STA/AP\n");
	printk("5: show mac status\n");
	printk("6: restart mac\n");
	printk("7: save all reg(COMM/SYS/MAC/PHY/RF) to local file\n");
	printk("8: stop all to write trout RAM\n");
	printk("********************************************\n");

	if(args->value == 1){
		print_rx_pkt_cnt_info();
	}
	else if(args->value == 2){
		clear_rx_pkt_cnt_info();
	}

	if(args->value < 4){
		g_rx_dbg_flag = args->value;
	}

	if(args->value == 4){
		print_all_entry();
	}

	if(args->value == 5){
		mac_status_show();
	}

	if(args->value == 6){
		restart_mac(&g_mac, 0);
	}
	
	if(args->value == 7){
		dump_allregs(0, 0);
	}
	if(args->value == 8){
		dump_dl_data();
	}
	#if 1 //chenq add for test
//#ifndef AUTORATE_FEATURE
	return 0;
//#endif
	
#if 0
        struct timespec time;	
        getnstimeofday(&time);
        PRINTK("time s=%u:ms= %u\n", time.tv_sec, (time.tv_nsec/1000/1000));

        print_test_stats(0);
#endif		
#if 0
    switch(args->value)
    {
        case 1:
            trout_rf_switch(1,0);
        break;
        
        case 2:
            trout_rf_switch(1,1);
        break;
        
        case 3:
            trout_rf_switch(2,0);
        break;   
        
        case 4:
            trout_rf_switch(2,1); // tx iq
        break;
        
        case 5:
            trout_rf_switch(3,0);
        break;
        
        case 6:
            trout_rf_switch(3,1);
        break;  
        
        case 7:
            trout_rf_switch(4,0);
        break;
        
        case 8:
            trout_rf_switch(4,1); //rx iq
        break;
        
        default:
            break;
    }
#endif
        return 0;

	#else
	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if(host_req == NULL)
	{
		return -ENOMEM;
	}

	host_req[0] = WID_CURRENT_TX_RATE & 0xFF;
	host_req[1] = (WID_CURRENT_TX_RATE & 0xFF00) >> 8;
	host_req[2] = 1;
	
	if(args->value != -1)
		host_req[3] = (args->value /1000000) & 0xFF;
	else
		host_req[3] = 0x00;

	config_if_for_iw(&g_mac,host_req,WID_CHAR_CFG_LEN,'W',&trout_rsp_len);

	if( trout_rsp_len != 1 )
	{
		ret = -EINVAL;
		goto out1;
	}
	   
	out1:
	//kfree(host_req);
	return ret;
	#endif
}		

/*------------------------------------------------------------------*/
/*
 * Standard Wireless Handler : get default bit rate (bps) 
 */
static int itm_giwrate(struct net_device *	dev,
			  struct iw_request_info *info,
			   union iwreq_data * wrqu,  char * extra)
{

	struct iw_param *args = &wrqu->bitrate;
	int rate=0;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	PRINTK_ITMIW("itm_giwrate fuc\n");

	rate = itm_get_rate();
	if(rate < 0)
	{
		PRINTK_ITMIW("itm_giwrate fuc get rate err \n");
		return rate;
	}
	else
	{
		
		if( rate < 0 )
		{
			args->value = 1000000;
		}
		else if(IS_RATE_MCS(rate) == BFALSE)
		{
			args->value = rate * 1000000;
		}
		else
		{
			int j=0;
			for( j = 0; j < sizeof(rate_table)/sizeof(rate_table_t); j++ )
			{
				if(rate_table[j].rate_index == rate)
				{
					args->value = (rate_table[j].rate * 1000000) + 
						          ((rate_table[j].point5 == 1) ? 500000 : 0);
					break;
				}
			}

			if( j >= sizeof(rate_table)/sizeof(rate_table_t) )
			{
				args->value = 1000000;
			}
		}			

		PRINTK_ITMIW("itm_giwrate fuc get rate:%d \n",rate);
		return 0;
	}
}	

/*------------------------------------------------------------------*/
/*
 * Standard Wireless Handler : set RTS/CTS threshold (bytes) 
 */



static int itm_siwrts(struct net_device *	dev,
			  struct iw_request_info *info,
			   union iwreq_data * wrqu,  char * extra)
{
	int ret = 0;
	UWORD8 * host_req  = NULL;
	UWORD16  trout_rsp_len = 0;
	int val = wrqu->rts.value;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_NO_ACCESS);

	PRINTK_ITMIW("itm_siwrts fuc \n");

	PRINTK_ITMIW("wrqu->rts.value %d \n",wrqu->rts.value);
	PRINTK_ITMIW("wrqu->rts.disabled %d \n",wrqu->rts.disabled);
	PRINTK_ITMIW("wrqu->rts.fixed %d \n",wrqu->rts.fixed);
	

	if(wrqu->rts.disabled)
	{
		val = 65535;
	}

	if( (val <256) || (val > 65535) )
	{
		return -EINVAL;
	}

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if(host_req == NULL)
	{
		return -ENOMEM;
	}

	host_req[0] = WID_RTS_THRESHOLD & 0xFF;
	host_req[1] = (WID_RTS_THRESHOLD & 0xFF00) >> 8;
	
	host_req[2] = (WID_SHORT + 1) & 0xFF;

	host_req[3] = val & 0xFF;
	host_req[4] = (val & 0xFF00) >> 8;

	config_if_for_iw(&g_mac,host_req,(UWORD16)WID_SHORT_CFG_LEN,
							   'W',&trout_rsp_len);

	if( trout_rsp_len != 1 )
	{
		ret = -EINVAL;
		goto out1;
	}
	   
	out1:
	//kfree(host_req);
	return ret;
}	

/*------------------------------------------------------------------*/
/*
 * Standard Wireless Handler : get RTS/CTS threshold (bytes)
 */
static int itm_giwrts(struct net_device *	dev,
			  struct iw_request_info *info,
			   union iwreq_data * wrqu,  char * extra)
{
	int ret = 0;
	UWORD8 * host_req  = NULL;
	UWORD8 * trout_rsp = NULL;
	UWORD16  trout_rsp_len = 0;
	UWORD16  rts=0;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	PRINTK_ITMIW("itm_giwrts fuc\n");

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if(host_req == NULL)
	{
		return -ENOMEM;
	}

	host_req[0] = WID_RTS_THRESHOLD & 0xFF;
	host_req[1] = (WID_RTS_THRESHOLD & 0xFF00) >> 8;
	
	trout_rsp = config_if_for_iw(&g_mac,host_req,2,'Q',&trout_rsp_len);

	if( trout_rsp == NULL )
	{
		ret = -EINVAL;
		goto out1;
	}

	trout_rsp_len -= MSG_HDR_LEN;
	if(get_trour_rsp_data((UWORD8 *)(&rts),sizeof(rts),&trout_rsp[MSG_HDR_LEN],
					      trout_rsp_len,WID_SHORT)  ==  0)
	{
		ret = -EINVAL;
		goto out2;
	}

	PRINTK_ITMIW("itm_giwrts fuc %d \n",rts);
	
	wrqu->rts.value = rts & 0xFFFF;
	wrqu->rts.disabled = ( (rts & 0xFFFF)  == 65535);
	wrqu->rts.fixed = 0;

	out2:
	pkt_mem_free(trout_rsp);
	out1:
	//kfree(host_req);
	return ret;
}		

/*------------------------------------------------------------------*/
/*
 * Standard Wireless Handler : set fragmentation thr (bytes)
 */
static int itm_siwfrag(struct net_device *	dev,
			  struct iw_request_info *info,
			   union iwreq_data * wrqu,  char * extra)
{
	int ret = 0;
	UWORD8 * host_req  = NULL;
	UWORD16  trout_rsp_len = 0;
	UWORD16  val = wrqu->frag.value;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_NO_ACCESS);

	PRINTK_ITMIW("itm_siwfrag fuc\n");

	PRINTK_ITMIW("wrqu->frag.value %d \n",wrqu->frag.value);
	PRINTK_ITMIW("wrqu->frag.disabled %d \n",wrqu->frag.disabled);
	PRINTK_ITMIW("wrqu->frag.fixed %d \n",wrqu->frag.fixed);

	if(wrqu->frag.disabled)
	{
		val = 2346;
	}

	if( (val < 256) || (val > 7936) )
	{
		return -EINVAL;
	}

	 host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if(host_req == NULL)
	{
		return -ENOMEM;
	}

	host_req[0] = WID_FRAG_THRESHOLD & 0xFF;
	host_req[1] = (WID_FRAG_THRESHOLD & 0xFF00) >> 8;

	host_req[2] = (WID_SHORT + 1) & 0xFF;
	
	host_req[3] = val & 0xFF;
	host_req[4] = (val & 0xFF00) >> 8;
		
	config_if_for_iw(&g_mac,host_req,WID_SHORT_CFG_LEN,'W',&trout_rsp_len);

	if( trout_rsp_len != 1 )
	{
		ret = -EINVAL;
		goto out1;
	}

	out1:
	//kfree(host_req);
	return ret;
}

/*------------------------------------------------------------------*/
/*
 * Standard Wireless Handler : get fragmentation thr (bytes)
 */
static int itm_giwfrag(struct net_device *	dev,
			  struct iw_request_info *info,
			   union iwreq_data * wrqu,  char * extra)
{
	int ret = 0;
	UWORD8 * host_req  = NULL;
	UWORD8 * trout_rsp = NULL;
	UWORD16  trout_rsp_len = 0;
	UWORD16  val = 0;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	PRINTK_ITMIW("itm_giwfrag fuc\n");

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if(host_req == NULL)
	{
		return -ENOMEM;
	}
	
	host_req[0] = WID_FRAG_THRESHOLD & 0xFF;
	host_req[1] = (WID_FRAG_THRESHOLD & 0xFF00) >> 8;
	
	trout_rsp = config_if_for_iw(&g_mac,host_req,2,'Q',&trout_rsp_len);

	if( trout_rsp == NULL )
	{
		ret = -EINVAL;
		goto out1;
	}

	trout_rsp_len -= MSG_HDR_LEN;
	if( get_trour_rsp_data((UWORD8 *)(&val),sizeof(val),&trout_rsp[MSG_HDR_LEN],
						trout_rsp_len,WID_SHORT) == 0)
	{
		ret = -EINVAL;
		goto out2;
	}

	wrqu->frag.value =  val & 0xFFFF;
    wrqu->frag.disabled = (val >= 7936);
    wrqu->frag.fixed = 1;

	out2:
	pkt_mem_free(trout_rsp);
	out1:
	//kfree(host_req);
	return ret;
	
}	

static int itm_giwretry(struct net_device *	dev,
			  struct iw_request_info *info,
			   union iwreq_data * wrqu,  char * extra)
{

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	PRINTK_ITMIW("itm_giwretry fuc\n");
	
	wrqu->retry.value = 7;
	wrqu->retry.fixed = 0;	/* no auto select */
	wrqu->retry.disabled = 1;
	
	return 0;
}

/*------------------------------------------------------------------*/
/*
 * Standard Wireless Handler : set encoding token & mode
 */
static int itm_siwencode(struct net_device *	dev,
			  struct iw_request_info *info,
			   union iwreq_data * wrqu,  char * extra)
{
	int ret = 0;
	struct iw_point * erq = &wrqu->encoding;
	UWORD8 index=0;
	int privacy = -1;
	int tmp_key_index = 0;
	int len = 0;
	UWORD16  trout_rsp_len = 0;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	PRINTK_ITMIW("itm_siwencode fuc\n");
	
	 /*
	 * Key index is encoded in the flags.
	 * 0 - use current default,
	 * 1-4 - if a key value is given set that key
	 *       if not use that key
	 */
    index = (erq->flags & IW_ENCODE_INDEX);

	if ((index < 0) || (index > 4))
	{
		return -EINVAL;
	}

	/* Check the index (none (i.e. 0) means use current) */
	/* If we do not have a previous key, use 1 as default */
	if( index == 0 )
	{
		if( (tmp_key_index = itm_get_wep_key_index()) < 0 )
		{
			return -EINVAL;
		}
		index = tmp_key_index & 0xFF ;
	}
	else
	{
		index--;
	}

	/*
     * Basic checking: do we have a key to set ?
     * The IW_ENCODE_NOKEY flag is set when no key is present (only change flags),
     * but older versions rely on sending a key id 1-4.
     */
	if(erq->length > 0)
	{
		PRINTK_ITMIW("itm_siwencode fuc key index:%d\n",index);
		PRINTK_ITMIW("itm_siwencode fuc key len:%d\n",erq->length);
		PRINTK_ITMIW("itm_siwencode fuc key:%s\n",(char *)extra);
		/* Check the size of the key */
        if ((erq->length != LARGE_KEY_SIZE) && (erq->length != SMALL_KEY_SIZE)) 
		{
            return  -EINVAL;
        }

		privacy = 1;

		/* set wep length */
		if( erq->length == SMALL_KEY_SIZE )
		{
			g_itm_config_buf[len+0] = WID_11I_MODE & 0xFF;
			g_itm_config_buf[len+1] = (WID_11I_MODE & 0xFF00) >> 8;
			g_itm_config_buf[len+2] = 1;
			g_itm_config_buf[len+3] = (UWORD8)WEP64;
			len += WID_CHAR_CFG_LEN;
			#if 0	
			if( itm_set_Encryption_Type((UWORD8)WEP64) < 0 )
			{
				return -EINVAL;
			}
			#endif
		}
		else
		{
			g_itm_config_buf[len+0] = WID_11I_MODE & 0xFF;
			g_itm_config_buf[len+1] = (WID_11I_MODE & 0xFF00) >> 8;
			g_itm_config_buf[len+2] = 1;
			g_itm_config_buf[len+3] = (UWORD8)WEP128;
			len += WID_CHAR_CFG_LEN;
			#if 0
			if( itm_set_Encryption_Type((UWORD8)WEP128) < 0 )
			{
				return -EINVAL;
			}
			#endif
		}

		/* set wep key index*/
		g_itm_config_buf[len+0] = WID_KEY_ID & 0xFF;
		g_itm_config_buf[len+1] = (WID_KEY_ID & 0xFF00) >> 8;
		g_itm_config_buf[len+2] = 1;
		g_itm_config_buf[len+3] = index;
		len += WID_CHAR_CFG_LEN;
		#if 0
		if( itm_set_wep_key_index(index) < 0 )
		{
			return -EINVAL;
		}
		#endif
		 /* Check if the key is not marked as invalid */
        if((erq->flags & IW_ENCODE_NOKEY) == 0) 
		{
			int i = 0;
			UWORD8 * ppkey = (UWORD8 *)extra;
			
			g_itm_config_buf[len+0] = WID_WEP_KEY_VALUE & 0xFF;
			g_itm_config_buf[len+1] = (WID_WEP_KEY_VALUE & 0xFF00) >> 8;
			g_itm_config_buf[len+2] = (erq->length & 0xFF) * 2;
			
			memcpy(&g_itm_config_buf[len+3],extra,erq->length);
			for( i = 0 ; i < erq->length ; i++)
			{
				g_itm_config_buf[len + 3+2*i] = hex_2_char((UWORD8)( (*ppkey & 0xF0) >> 4));
				g_itm_config_buf[len + 3+2*i + 1] = hex_2_char((UWORD8)( *ppkey & 0x0F));

				ppkey++;
			}
			len += WID_HEAD_LEN + erq->length * 2;

		}

		g_itm_config_buf[len+0] = WID_RESET & 0xFF;
		g_itm_config_buf[len+1] = (WID_RESET & 0xFF00) >> 8;
		g_itm_config_buf[len+2] = 1;
		g_itm_config_buf[len+3] = 0; //DONT_RESET
		len += WID_CHAR_CFG_LEN;

		config_if_for_iw(&g_mac,g_itm_config_buf,len,'W',&trout_rsp_len);

		if( trout_rsp_len != 1 )
		{
			return -EINVAL;
		}
		else
		{
			return 0;
		}
	}
	else
	{
	     /*
	     * No additional key data, so it must be a request to change the
	     * active key.
	     */
		if( itm_set_wep_key_index(index) < 0)
		{
			return -EINVAL;
		}
		else
		{
			privacy = 1;
		}
	}

	/* Read the flags */
	if (erq->flags & IW_ENCODE_DISABLED) 
	{
		/* disable encryption */
		PRINTK_ITMIW("itm_siwencode fuc: disable WEP encryption\n");
		privacy = 0;
		if( itm_set_Auth_Type( (UWORD8)(ITM_OPEN_SYSTEM & 0xFF) ) < 0 )
		{
			return -EINVAL;
		}
		if( itm_set_Encryption_Type((UWORD8)No_ENCRYPTION) < 0 )
		{
			return -EINVAL;
		}
	}

    if (erq->flags & IW_ENCODE_RESTRICTED) 
    {
         /* Use shared key auth */
         PRINTK_ITMIW("itm_siwencode fuc :use WEP shared-key auth\n");
		 //if( itm_set_Auth_Type( (UWORD8)(ITM_SHARED_KEY & 0xFF) ) < 0)
		 //{
		 	//return -EINVAL;
		 //}
    }
    else if (erq->flags & IW_ENCODE_OPEN) 
	{
        /* Only Wep */
        PRINTK_ITMIW("itm_siwencode fuc :use WEP open-system auth\n");
        if( itm_set_Auth_Type( (UWORD8)(ITM_OPEN_SYSTEM & 0xFF) ) < 0 )
        {
        	return -EINVAL;
        }
    }
    else	
    {
    	  /* else */
        PRINTK_ITMIW("itm_siwencode fuc :use WEP open-system auth\n");
        if( itm_set_Auth_Type( (UWORD8)(ITM_OPEN_SYSTEM & 0xFF) ) < 0 )
        {
        	return -EINVAL;
        }
    }

    /* Commit the changes to flags if needed */
#if 0	
    if (privacy != -1) {
        PRINTK_ITMIW("itm_siwencode fuc: setting WEP encryption = %d\n", privacy);
	
	 if( itm_set_Auth_Type( 
	 	(UWORD8)(ITM_WIFI_ROUTER_PORT_ACTION_8021X_PORT & 0xFF) ) < 0)
	 {
	 	PRINTK_ITMIW("itm_siwencode fuc 10\n");
	 	ret  =  -EINVAL;
		goto out1;
	 }

        /* Clear the WPA state. */
#if WIRELESS_EXT > 17
        /* \B2\CE\D5\D58810 \CF\C2\C3\E6wext.c \B4˴\A6Ӧ\B8\C3\CAǹر\D5wpa*/
        /* \B5\ABitm\C0\EF\C3潫wep wpa\D7\F6\D4\DAһ\B8\F6\BCĴ\E6\C6\F7\C0\EF\C3\E6*/
        /* \B9ʵ\B1disable wpa\A3\ACֻ\C4ܽ\AB\BC\D3\C3ܷ\BDʽ\C9\E8\D6\C3Ϊ\BF\D5*/
        //priv->wext_conf.wpa_version = IW_AUTH_WPA_VERSION_DISABLED;
        if( itm_set_Encryption_Type((UWORD8)No_ENCRYPTION) < 0 )	
	 {
	 	PRINTK_ITMIW("itm_siwencode fuc 11\n");
	 	ret  =  -EINVAL;
		goto out1;
	 }
#endif /* WIRELESS_EXT > 17 */
    }
#endif	

	return ret;
}			

/*------------------------------------------------------------------*/
/*
 * Standard Wireless Handler : get encoding token & mode
 */
static int itm_giwencode(struct net_device *	dev,
			  struct iw_request_info *info,
			   union iwreq_data * wrqu,  char * extra)
{
	int ret = 0;
	
	int auth_type = 0;
	int encry_type = 0;
	int wep_key_index = 0;
	int wep_key_len = 0;
	UWORD8 wep_key_value[LARGE_KEY_SIZE]= {0};

	struct iw_point * erq = &wrqu->encoding;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	PRINTK_ITMIW("itm_giwencode fuc\n");

	if ( (encry_type = itm_get_Encryption_Type()) < 0)
	{
		return -EINVAL;
	}
	
	if( encry_type < 0 )
	{
		return -EINVAL;
	}

/*
	if( encry_type == 0 )
	{
		erq->flags = IW_ENCODE_DISABLED;
		return 0;
	}
*/	
	switch(encry_type)
	{
		case WEP64:
			wep_key_len = 5;
			break;
		case WEP128:
			wep_key_len = 13;
			break;
		case No_ENCRYPTION:
			erq->flags = IW_ENCODE_DISABLED;
			return 0;
		default:
			break;
	}
	
	if( (wep_key_index = itm_get_wep_key_index()) < 0 )
	{
		return -EINVAL;
	}

	if( itm_get_wep_key_value(wep_key_value,LARGE_KEY_SIZE) < 0 )
	{
		return -EINVAL;
	}

	if( (auth_type = itm_get_Auth_Type()) < 0)
	{
		return -EINVAL;
	}

	wep_key_index++;

	if (auth_type == ITM_SHARED_KEY)
	{
		erq->flags = IW_ENCODE_RESTRICTED;
	}		
	else
	{
		erq->flags = IW_ENCODE_OPEN;
	}

	erq->flags |= (wep_key_index & IW_ENCODE_INDEX);
	erq->length = wep_key_len;
	memcpy(extra, wep_key_value , wep_key_len);
	//memcpy(erq->pointer, wep_key_value , wep_key_len);
	ret = 0;
	
	return ret;
}

/*------------------------------------------------------------------*/
/*
 * Standard Wireless Handler : set Power Management settings
 */
static int itm_siwpower(struct net_device *	dev,
			  struct iw_request_info *info,
			   union iwreq_data * wrqu,  char * extra)
{
	
	//struct	iw_param
	//{
	//  __s32		value;		/* The value of the parameter itself */
	//  __u8		fixed;		/* Hardware should not use auto select */
	//  __u8		disabled;	/* Disable the feature */
	//  __u16		flags;		/* Various specifc flags (if any) */
	//};
	int ret = 0;
	//int tmp_val = 0;
	struct iw_param *args = &wrqu->power;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_NO_ACCESS);

	PRINTK_ITMIW("itm_siwpower fuc\n");
	PRINTK_ITMIW("valu: %x\n",args->value);
	PRINTK_ITMIW("fixed: %02x\n",args->fixed);
	PRINTK_ITMIW("disabled: %02x\n",args->disabled);
	PRINTK_ITMIW("flags: %04x\n",args->flags);
	#ifdef IBSS_BSS_STATION_MODE
	
	if(!args->disabled)
	{
		/*  disable power  */
		#if 1
		//if( itm_set_power_type((UWORD8)( Minimum_PSPOLL_Power_Save_Mode & 0xFF)) < 0 )
		if( itm_set_power_type((UWORD8)( Maximum_PSPOLL_Power_Save_Mode & 0xFF)) < 0 )
		{
			return -EINVAL;
		}
		goto out0;
		#else
		g_UI_cmd_ps = BFALSE;
		#endif
	}
	else
	{
		/*  enable power  */
		#if 1
		if( itm_set_power_type(
			(UWORD8)(Active_Mode & 0xFF)) < 0 )
		{
			return -EINVAL;
		}
		goto out0;
		#else
		g_UI_cmd_ps = BTRUE;
		#endif
	}

	#if 0
	switch( args->flags & IW_POWER_TYPE )
	{
		case IW_POWER_PERIOD:
			tmp_val = args->value;
			while( tmp_val > 0 )
			{
				if(  (  tmp_val >= 1  ) && ( tmp_val <= 255)  )
				{
					break;
				}
				else 
				{
					tmp_val /=1000;
				}
			}
			
			if( tmp_val <= 0 )
			{
				return -EINVAL;
			}

			if( itm_set_power_period((UWORD8)(tmp_val & 0xFF)) < 0)
			{
				return -EINVAL;
			}
		default:
			break;
	}

	switch( args->flags & IW_POWER_MODE )
	{
		case IW_POWER_UNICAST_R:
			PRINTK_ITMIW("set  IW_POWER_UNICAST_R not support !\n");
			break;
		case IW_POWER_MULTICAST_R:
			PRINTK_ITMIW("set  IW_POWER_MULTICAST_R not support !\n");
			break;
		case IW_POWER_ALL_R:
			PRINTK_ITMIW("set  IW_POWER_ALL_R not support !\n");
			break;
		case IW_POWER_FORCE_S:
			PRINTK_ITMIW("set  IW_POWER_FORCE_S not support !\n");
			break;
		case IW_POWER_REPEATER:
			PRINTK_ITMIW("set  IW_POWER_REPEATER not support !\n");
			break;
		default:
			break;
	}
	#endif

	#endif/*IBSS_BSS_STATION_MODE*/
	
	out0:
	return ret;
	
}

/*------------------------------------------------------------------*/
/*
 * Standard Wireless Handler : get Power Management settings
 */
static int itm_giwpower(struct net_device *	dev,
			  struct iw_request_info *info,
			   union iwreq_data * wrqu,  char * extra)
{

	int ret = 0;
	int power_type  = 0;
	int power_period = 0;
	struct iw_param *args = &wrqu->power;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	if( (power_type = itm_get_power_type()) < 0)
	{
		return -EINVAL;
	}

	if( (power_period = itm_get_power_period()) < 0)
	{
		return -EINVAL;
	}

	args->value = power_period * 100000;
	args->disabled = (power_type == 0) ?  1 : 0;
	args->flags |= IW_POWER_PERIOD;

	args->flags |= IW_POWER_ALL_R; // ???? 

	return ret;
}


/* WPA : Generic IEEE 802.11 informatiom element (e.g., for WPA/RSN/WMM). */
/*------------------------------------------------------------------*/
/*
 * Standard Wireless Handler : set generic IE 
 */
static int itm_siwgenie(struct net_device *	dev,
			  struct iw_request_info *info,
			   union iwreq_data * wrqu,  char * extra)
{
#if 1
	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	PRINTK_ITMIW("itm_siwgenie fuc \n");

	return 0;
	
#else
	int ret = 0;
	UWORD8 * host_req  = NULL;
	UWORD16  trout_rsp_len = 0;
	int len = 0;

	PRINTK_ITMIW("itm_siwgenie fuc \n");

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if( host_req == NULL)
	{
		return -ENOMEM;
	}
	
	host_req[0] = WID_11I_PSK  & 0xFF;
	host_req[1] = (WID_11I_PSK  & 0xFF00) >> 8;
	host_req[2] = wrqu->data.length;

	memcpy(&host_req[3],extra,wrqu->data.length);
	len += WID_HEAD_LEN+wrqu->data.length;

	host_req[len+0] = WID_RESET & 0xFF;
	host_req[len+1] = (WID_RESET & 0xFF00) >> 8;
	host_req[len+2] = 1;
	host_req[len+3] = 0; //DONT_RESET
	len += WID_CHAR_CFG_LEN;

	config_if_for_iw( &g_mac,host_req,len,'W',&trout_rsp_len );

	if( trout_rsp_len != 1 )
	{
		ret = -EINVAL;
	}

	//kfree(host_req);
	return ret;
#endif	
}   

/*------------------------------------------------------------------*/
/*
 * Standard Wireless Handler : get generic IE
 */
static int itm_giwgenie(struct net_device *	dev,
			  struct iw_request_info *info,
			   union iwreq_data * wrqu,  char * extra)
{
	int ret = 0;
	UWORD8 * host_req  = NULL;
	UWORD8 * trout_rsp = NULL;
	UWORD16  trout_rsp_len = 0;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	PRINTK_ITMIW("itm_giwgenie fuc\n");
	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if(host_req == NULL)
	{
		return -ENOMEM;
	}

	host_req[0] = WID_11I_PSK & 0xFF;
	host_req[1] = (WID_11I_PSK & 0xFF00) >> 8;
	trout_rsp = config_if_for_iw(&g_mac,host_req,2,'Q',&trout_rsp_len);

	if( trout_rsp == NULL )
	{
		ret = -EINVAL;
		goto out1;
	}

	trout_rsp_len -= MSG_HDR_LEN;
	if((ret = get_trour_rsp_data((UWORD8*)extra,wrqu->data.length,
								&trout_rsp[MSG_HDR_LEN],
					      			trout_rsp_len,WID_STR))  ==  0)
	{	
		ret = -EINVAL;
		goto out2;
	}
	wrqu->data.length = ret;
	memcpy(wrqu->data.pointer,extra,wrqu->data.length);

	wrqu->data.flags = 1;
	
	out2:
	pkt_mem_free(trout_rsp);
	out1:
	//kfree(host_req);
	return ret;

}

    /* WPA : Authentication mode parameters */
/*------------------------------------------------------------------*/
/*
 * Standard Wireless Handler : set authentication mode params
 */
static int itm_siwauth(struct net_device *	dev,
			  struct iw_request_info *info,
			   union iwreq_data * wrqu,  char * extra)
{
	int encry_type = 0;
    int auth_Type = 0;
	struct iw_param *param = (struct iw_param*)&(wrqu->param);

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	PRINTK_ITMIW("itm_siwauth fuc\n");

	/*  get encry_type info in driver  */
	if ( (encry_type = itm_get_Encryption_Type()) < 0)
	{
		return -EINVAL;
	
	}
	if( (auth_Type = itm_get_Auth_Type()) < 0 )
	{
		return -EINVAL;
	}

    PRINTK_ITMIW("itm_siwauth fuc WI_flags = %d\n",
                    param->flags & IW_AUTH_INDEX);
    PRINTK_ITMIW("itm_siwauth fuc wrqu->param.value = %x\n",
                    wrqu->param.value);
	
	switch (param->flags & IW_AUTH_INDEX) 
	{
		case IW_AUTH_WPA_VERSION: //ok
            PRINTK_ITMIW("IW_AUTH_WPA_VERSION: %d\n", wrqu->param.value);
			/*
			IW_AUTH_WPA_VERSION_DISABLED	0x00000001
			IW_AUTH_WPA_VERSION_WPA		    0x00000002
			IW_AUTH_WPA_VERSION_WPA2	    0x00000004
			*/
			if(!(wrqu->param.value & IW_AUTH_WPA_VERSION_DISABLED))
			{
				#if 1
				if( wrqu->param.value &  IW_AUTH_WPA_VERSION_WPA )
    			{
    				/*
    				wpa: XXX0 1001(bit X do not change)
    				*/
    				encry_type &= (BIT7 | BIT6 | BIT5);
    				encry_type |= (BIT3 | BIT0);
    				if( itm_set_Encryption_Type(encry_type) < 0)
    				{
    					return -EINVAL;
    				}
    			}
    			else if( wrqu->param.value &  IW_AUTH_WPA_VERSION_WPA2 )
    			{
    				/*
    				wpa2: XXX1 0001(bit X do not change)
    				*/
    				encry_type &= (BIT7 | BIT6 | BIT5);
    				encry_type |= (BIT4 | BIT0);
    				if( itm_set_Encryption_Type(encry_type) < 0)
    				{
    					return -EINVAL;
    				}
    			}
				else
				{
					/* set no encry */
                    //if( itm_set_Auth_Type((UWORD8)(ITM_OPEN_SYSTEM)) < 0)
                    //{
                        //return -EINVAL;
                    //}
					//if( itm_set_Encryption_Type((UWORD8)(No_ENCRYPTION)) < 0)
					//{
						//return -EINVAL;
					//}
				}
				#endif
			}
//Hugh: add MAC_WAPI_SUPP to avoid compiler error.
#ifdef MAC_WAPI_SUPP
			if(wrqu->param.value == IW_AUTH_WAPI_VERSION_1)
			{
				#ifdef ITM_WIFI_SECURITY_WAPI_ENABLE
					
					if( itm_set_wapimode(1) < 0 )
		            {
		                return -EINVAL;
		            }
					
				#endif
				/* set no encry */
                //if( itm_set_Auth_Type((UWORD8)(ITM_OPEN_SYSTEM)) < 0)
                //{
                    //return -EINVAL;
                //}
				//if( itm_set_Encryption_Type((UWORD8)(No_ENCRYPTION)) < 0)
				//{
					//return -EINVAL;
				//}
			}
			break;
#endif

		case IW_AUTH_CIPHER_PAIRWISE://ok
			/* same as  case IW_AUTH_CIPHER_GROUP at moment*/
			PRINTK_ITMIW("IW_AUTH_CIPHER_PAIRWISE: %d \n", wrqu->param.value);

			#if 1
			if (wrqu->param.value & IW_AUTH_CIPHER_TKIP) 
			{
			     /*
			     tkip 010x x001(bit X do not change) 
			     add     - ---- ('-' dont care in this)
			     */
			     //encry_type &= ( BIT3 | BIT4 );
			     //encry_type |= ( BIT6 | BIT0 );
			     encry_type   |= BIT6;
		         if( itm_set_Encryption_Type(encry_type) < 0)
			     {
					return -EINVAL;
			     }
		    }
		    if (wrqu->param.value & IW_AUTH_CIPHER_CCMP) 
			{
				/*
				aes  001X X001(bit X do not change)
				add     - ---- ('-' dont care in this)
				*/
				//encry_type &= ( BIT3 | BIT4 );
			    //encry_type |= ( BIT5 | BIT0 );
			    encry_type   |= BIT5;
				if( itm_set_Encryption_Type((UWORD8)encry_type) < 0)
			    {
					return -EINVAL;
			    }   
		    }
			
			break;
			#endif
		case IW_AUTH_CIPHER_GROUP://ok
		    PRINTK_ITMIW("IW_AUTH_CIPHER_GROUP: %d\n", wrqu->param.value);

			#if 1
			/*
	         one of:
	         IW_AUTH_CIPHER_NONE	0x00000001
	         IW_AUTH_CIPHER_WEP40	0x00000002
	         IW_AUTH_CIPHER_TKIP	0x00000004
	         IW_AUTH_CIPHER_CCMP	0x00000008
	         IW_AUTH_CIPHER_WEP104	0x00000010
		   */
 	       if( wrqu->param.value & IW_AUTH_CIPHER_NONE)
 	       {
 	       	    /* set no encry */
 			    if( itm_set_Encryption_Type((UWORD8)(No_ENCRYPTION)) < 0)
 			    {
 				    return -EINVAL;
 			    }
 	        }
 	        if (wrqu->param.value & IW_AUTH_CIPHER_WEP40) 
			{
		         if( itm_set_Encryption_Type((UWORD8)(WEP64)) < 0)
			     {
					return -EINVAL;
			     }
		    }
		    if (wrqu->param.value & IW_AUTH_CIPHER_WEP104)
			{
		         if( itm_set_Encryption_Type((UWORD8)(WEP128)) < 0)
			     {
					return -EINVAL;
			     }
		    }
		    if (wrqu->param.value & IW_AUTH_CIPHER_TKIP) 
			{
			     /*
			     tkip 010x x001(bit X do not change) 
			     add     - ---- ('-' dont care in this)
			     */
			     //encry_type &= ( BIT3 | BIT4 );
			     //encry_type |= ( BIT6 | BIT0 );
			     encry_type   |= BIT6;
		         if( itm_set_Encryption_Type(encry_type) < 0)
			     {
					return -EINVAL;
			     }
		    }
		    if (wrqu->param.value & IW_AUTH_CIPHER_CCMP) 
			{
				/*
				aes  001X X001(bit X do not change)
				add     - ---- ('-' dont care in this)
				*/
				//encry_type &= ( BIT3 | BIT4 );
			    //encry_type |= ( BIT5 | BIT0 );
			    encry_type   |= BIT5;
				if( itm_set_Encryption_Type((UWORD8)encry_type) < 0)
			    {
					return -EINVAL;
			    }   
		    }
			break;
			#endif
		case IW_AUTH_KEY_MGMT://ok       
			#ifdef ITM_WIFI_SECURITY_WAPI_ENABLE
			if(wrqu->param.value == 4)
			{	
				PRINTK_ITMIW("use wapi psk\n");
			}
			else
			{
				PRINTK_ITMIW("not use wapi psk\n");
			}
			#endif
			
			#if 1
                /*
               IW_AUTH_KEY_MGMT_802_1X 1
               IW_AUTH_KEY_MGMT_PSK    2
               */
            PRINTK_ITMIW("IW_AUTH_KEY_MGMT: %d\n", wrqu->param.value);
			if(  ((encry_type & (BIT0 | BIT3)) == (BIT0 | BIT3)) 
               ||((encry_type & (BIT0 | BIT4)) == (BIT0 | BIT4)) 
              )
            {
                /* wpa auth or wpa2 auth*/
				#if 1 //chenq mod & todo 2013-01-12
                if (wrqu->param.value == IW_AUTH_KEY_MGMT_802_1X) 
                {
                    /*ITM_WIFI_SME_AUTH_MODE_8021X_WPAPSK*/
                    if(itm_set_Auth_Type((UWORD8)ITM_WIFI_ROUTER_PORT_ACTION_8021X_PORT) < 0)
                    {
                        return -EINVAL;
                    }
                    
                }
                else if (wrqu->param.value == IW_AUTH_KEY_MGMT_PSK) 
                {
                    if( itm_set_Auth_Type( (UWORD8)ITM_OPEN_SYSTEM ) < 0)
			        {
					    return -EINVAL;
			        }  
                }
				#else
				if( (wrqu->param.value == IW_AUTH_KEY_MGMT_PSK) 
				   ||(wrqu->param.value == IW_AUTH_KEY_MGMT_802_1X))
				{
					if( itm_set_Auth_Type( (UWORD8)ITM_OPEN_SYSTEM ) < 0)
			        {
					    return -EINVAL;
			        }
				}
				#endif
            }
			#endif
			
			break;
		case IW_AUTH_80211_AUTH_ALG	://ko
			/*
       		IW_AUTH_ALG_OPEN_SYSTEM      0x00000001
       		IW_AUTH_ALG_SHARED_KEY       0x00000002
       		IW_AUTH_ALG_LEAP             0x00000004
    		*/

            auth_Type = 0;
        	if( (wrqu->param.value & IW_AUTH_ALG_OPEN_SYSTEM)
				&&(wrqu->param.value & IW_AUTH_ALG_SHARED_KEY))
			{
			    PRINTK_ITMIW("IW_AUTH_80211_AUTH_ALG: %d (IW_AUTH_ALG_OPEN_SYSTEM & IW_AUTH_ALG_SHARED_KEY)\n", 
                                wrqu->param.value);
				auth_Type  =  ITM_OPEN_SYSTEM & 0xFF; 
		    }
			else if(wrqu->param.value & IW_AUTH_ALG_OPEN_SYSTEM)
			{
				PRINTK_ITMIW("IW_AUTH_80211_AUTH_ALG: %d (IW_AUTH_ALG_OPEN_SYSTEM)\n", 
                                wrqu->param.value);
				auth_Type  =  ITM_OPEN_SYSTEM & 0xFF;
			}
       		else if (wrqu->param.value & IW_AUTH_ALG_SHARED_KEY) 
			{
			    PRINTK_ITMIW("IW_AUTH_80211_AUTH_ALG: %d (IW_AUTH_ALG_SHARED_KEY)\n", 
                                wrqu->param.value);
            	auth_Type  =  ITM_SHARED_KEY & 0xFF;
        	}
        	if (wrqu->param.value & IW_AUTH_ALG_LEAP) 
			{
			    PRINTK_ITMIW("IW_AUTH_80211_AUTH_ALG: %d (IW_AUTH_ALG_LEAP) no use\n", 
                                wrqu->param.value);
       			 /* not supp!*/
        	}

			#if 0
            if(!auth_Type)
            {
                break;
            }
            else
            if(auth_Type > ITM_SHARED_KEY)
            {
                auth_Type = ITM_OPEN_SYSTEM;
            }
			#endif
            
			if( itm_set_Auth_Type(auth_Type) < 0)	
			{
				return -EINVAL;
			}
			break;
		case IW_AUTH_WPA_ENABLED://ko
            PRINTK_ITMIW("IW_AUTH_WPA_ENABLED: %d\n", wrqu->param.value);

			#if 0
            if (wrqu->param.value == 0)
            {
                PRINTK_ITMIW("IW_AUTH_WPA_ENABLED: CSR_WIFI_SME_AUTH_MODE_80211_OPEN\n");
                if( itm_set_Auth_Type((UWORD8)ITM_OPEN_SYSTEM) < 0)	
			    {
					return -EINVAL;
			    }
            }
			#endif
			
            break;
		
		case IW_AUTH_PRIVACY_INVOKED://ko
             /*do nothing*/
             PRINTK_ITMIW("IW_AUTH_PRIVACY_INVOKED: %d\n", wrqu->param.value);
			break;
        case IW_AUTH_TKIP_COUNTERMEASURES://ko
			/*
	         * Set to true at the start of the 60 second backup-off period
	         * following 2 MichaelMIC failures within 60s.
	         */
	         /*do nothing*/
            PRINTK_ITMIW("IW_AUTH_TKIP_COUNTERMEASURES: %d\n", wrqu->param.value);
			break;    
        case IW_AUTH_DROP_UNENCRYPTED://ko
			/* HACK:
			 *
			 * wpa_supplicant calls set_wpa_enabled when the driver
			 * is loaded and unloaded, regardless of if WPA is being
			 * used.  No other calls are made which can be used to
			 * determine if encryption will be used or not prior to
			 * association being expected.  If encryption is not being
			 * used, drop_unencrypted is set to false, else true -- we
			 * can use this to determine if the CAP_PRIVACY_ON bit should
			 * be set.
			 */
			 
			/*do nothing*/
            PRINTK_ITMIW("IW_AUTH_DROP_UNENCRYPTED: %d\n", wrqu->param.value);
			break;    
        case IW_AUTH_RX_UNENCRYPTED_EAPOL://ko
			/*
		         * This is set by wpa_supplicant to allow unencrypted EAPOL messages
		         * even if pairwise keys are set when not using WPA. IEEE 802.1X
		         * specifies that these frames are not encrypted, but WPA encrypts
		         * them when pairwise keys are in use.
		         * I think the itm f/w handles this decision for us.
		         */

			/*do nothing*/
            PRINTK_ITMIW("IW_AUTH_RX_UNENCRYPTED_EAPOL: %d\n", wrqu->param.value);
			break;    
        case IW_AUTH_ROAMING_CONTROL://ko
			/*do nothing*/
            PRINTK_ITMIW("IW_AUTH_ROAMING_CONTROL: %d\n", wrqu->param.value);
			break;
		#ifdef ITM_WIFI_SECURITY_WAPI_ENABLE	
		case IW_AUTH_WAPI_ENABLED:
			PRINTK_ITMIW("set IW_AUTH_WAPI_ENABLED");
			break;	
		#endif
		default://ko
            PRINTK_ITMIW("Unsupported auth param %d to 0x%X\n",
                        wrqu->param.flags & IW_AUTH_INDEX,
                        wrqu->param.value);
            return -EOPNOTSUPP;
	}		
	
	return 0;
}

#if 0 //not use
/*------------------------------------------------------------------*/
/*
 * Standard Wireless Handler : get authentication mode params
 */
static int itm_giwauth(struct net_device *	dev,
			  struct iw_request_info *info,
			   union iwreq_data * wrqu,  char * extra)
{
	/*
	int ret = 0;
	UWORD8 * host_req  = NULL;
	UWORD8 * trout_rsp = NULL;
	UWORD16  trout_rsp_len = 0;
	*/

	PRINTK_ITMIW("itm_giwauth fuc\n");
	
	return -EOPNOTSUPP;
}
#endif

    /* WPA : Extended version of encoding configuration */
/*------------------------------------------------------------------*/
/*
 * Standard Wireless Handler : set encoding token & mode
 */

static int itm_siwencodeext(struct net_device *	dev,
			  struct iw_request_info *info,
			   union iwreq_data * wrqu,  char * extra)
{
    struct iw_encode_ext *ext = (struct iw_encode_ext *)extra;
    int r = 0;
    UWORD8 * keydata;
	UWORD8 tkip_key[32];
	UWORD8 key_type = 0;
    int keyid;
    int device_mode=0;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	PRINTK_ITMIW("itm_siwencodeext func \n");
	
    if( (device_mode = itm_get_device_mode()) < 0 )
    {
        return -EINVAL;
    }

    if(device_mode == DEVICE_MODE_Access_Point ||
       device_mode == DEVICE_MODE_P2P_GO) {
       PRINTK_ITMIW("itm_siwencodeext: not permitted in Mode %d\n",
                                      device_mode);

       return -EPERM;
    }

    PRINTK_ITMIW("itm_siwencodeext fuc set codeext:ext->key_len = %d,",
                  ext->key_len);
    PRINTK_ITMIW("ext->ext_flags = %x,",ext->ext_flags);
    PRINTK_ITMIW("wrqu->encoding.flags & IW_ENCODE_INDEX = %x,",
                    wrqu->encoding.flags & IW_ENCODE_INDEX);
    PRINTK_ITMIW("ext->alg = %x\n",ext->alg); 


    if ((ext->key_len == 0) && (ext->ext_flags & IW_ENCODE_EXT_SET_TX_KEY)) 
    {	
        /* This means use a different key (given by key_idx) for Tx. */
        /* NYI */
        PRINTK_ITMIW("itm_siwencodeext: NYI should change tx key id here!!\n");
        return -ENOTSUPP;
    }

    keydata = (UWORD8 *)(ext + 1);
    keyid = (wrqu->encoding.flags & IW_ENCODE_INDEX);

    /*
     * Check for request to delete keys for an address.
     */
    /* Pick out request for no privacy. */
    if (ext->alg == IW_ENCODE_ALG_NONE) 
    {
        PRINTK_ITMIW("Deleting %s key %d\n",
                    (ext->ext_flags & IW_ENCODE_EXT_GROUP_KEY) ? "GROUP" : "PAIRWISE",
                    keyid);
        /* but we don't care "GROUP" or "PAIRWISE",and we use GROUP*/

        r = itm_remove_wep_key();
        if (r) 
        {
            PRINTK_ITMIW("Delete key request was rejected with result %d\n", r);
            return r;
        }
	/*junbin.wang modify for cr 240268. 20131126*/
	itm_set_Encryption_Type((UWORD8)0);
        return 0;
    }

    /*
     * Request is to set a key, not delete
     */

    /* Pick out WEP and use set_wep_key(). */
    if (ext->alg == IW_ENCODE_ALG_WEP) 
    {
        /* WEP-40, WEP-104 */

        /* Check for valid key length */
        if (!((ext->key_len == 5) || (ext->key_len == 13))) {
            PRINTK_ITMIW("Invalid length for WEP key: %d\n", ext->key_len);
            return -EINVAL;
        }

        PRINTK_ITMIW("Setting WEP key %d tx:%d\n",
                    keyid, ext->ext_flags & IW_ENCODE_EXT_SET_TX_KEY);

        if(itm_add_wep_key(keydata,ext->key_len,keyid - 1) < 0 )
        {
            PRINTK_ITMIW("Setting WEP key fail !!!\n");
            return -EINVAL;
        }

        return 0;
    }

    /*
     *
     * If we reach here, we are dealing with a WPA/WPA2 key
     *
     */

	/*remove wpa/wpa2 all key*/
	/* wpa/wpa2 gtk,ptk*/
	/*
    if(ext->alg == IW_ENCODE_ALG_TKIP)  //ITM_IW_ENCODE_ALG_WPAUSERKEY)
    {	
        //in itm we use internal config
        PRINTK_ITMIW("Setting wpa/wpa2 key: %s\n",ext->key);
        if( itm_set_psk_value(ext->key,ext->key_len) < 0 )
        {
            PRINTK_ITMIW("Setting wpa/wpa2 key fail !!!\n");
            return -EINVAL;
        }
        
    }
	*/
	
	if (ext->key_len > 32) {
        return -EINVAL;
    }

    /*
     * TKIP keys from wpa_supplicant need swapping.
     * What about other supplicants (when they come along)?
     */

	if(ext->alg == IW_ENCODE_ALG_PMK)
	{
		//in itm we use internal config
        PRINTK_ITMIW("Setting wpa/wpa2 key: %s\n",ext->key);

		memcpy(g_psk_value,ext->key,ext->key_len);
		g_psk_value[39] = 1;
		return 0;
	}
	
	

	if( (ext->alg == IW_ENCODE_ALG_TKIP) || (ext->alg == IW_ENCODE_ALG_CCMP) )
	{
	    if ((ext->alg == IW_ENCODE_ALG_TKIP) && (ext->key_len == 32)) {
	        memcpy(tkip_key, keydata, 16);
	        memcpy(tkip_key + 16, keydata + 24, 8);
	        memcpy(tkip_key + 24, keydata + 16, 8);
	        keydata = tkip_key;
	    }
	    
	    key_type = (ext->ext_flags & IW_ENCODE_EXT_GROUP_KEY) ?
					/* Group Key */    /* Pairwise Key */
	        		       0         :       1;
		PRINTK_ITMIW("key type = %d,key index = %d,key len = %d\n",key_type,keyid - 1,ext->key_len);
		PRINTK_ITMIW("addr = %02X:%02X:%02X:%02X:%02X:%02X\n",
			ext->addr.sa_data[0],ext->addr.sa_data[1],ext->addr.sa_data[2],
			ext->addr.sa_data[3],ext->addr.sa_data[4],ext->addr.sa_data[5]);
		PRINTK_ITMIW("rsc = %02X:%02X:%02X:%02X:%02X:%02X\n",
			ext->rx_seq[0],ext->rx_seq[1],ext->rx_seq[2],
			ext->rx_seq[3],ext->rx_seq[4],ext->rx_seq[5]);
		
		return itm_add_wpa_wpa2_value(ext->addr.sa_data,keydata, 
					(UWORD8)ext->key_len, (UWORD8)key_type, 
					(UWORD8)(keyid - 1),ext->rx_seq);
	}

	#ifdef ITM_WIFI_SECURITY_WAPI_ENABLE
	if( ext->alg == IW_ENCODE_ALG_SM4 )
	{
		struct iw_point *pencoding = &wrqu->encoding;
		
		wapi_key_t wapi_key;
		UWORD8 WapiASUEPNInitialValueSrc[16] = {0x5C,0x36,0x5C,0x36,0x5C,0x36,0x5C,0x36,0x5C,0x36,0x5C,0x36,0x5C,0x36,0x5C,0x36} ;
		UWORD8 WapiAEMultiCastPNInitialValueSrc[16] = {0x5C,0x36,0x5C,0x36,0x5C,0x36,0x5C,0x36,0x5C,0x36,0x5C,0x36,0x5C,0x36,0x5C,0x36} ;	

		if(ext->ext_flags & IW_ENCODE_EXT_GROUP_KEY)
		{/* Group Key */
			wapi_key.unicastKey = 0;
			
			memcpy(wapi_key.keyRsc,WapiAEMultiCastPNInitialValueSrc,sizeof(WapiAEMultiCastPNInitialValueSrc));
		}
		else
		{/* Pairwise Key */
			wapi_key.unicastKey = 1;

			memcpy(wapi_key.keyRsc,WapiASUEPNInitialValueSrc,sizeof(WapiASUEPNInitialValueSrc));
		}

		wapi_key.keyIndex   = (pencoding->flags & 0x00FF) -1 ;

		memcpy(wapi_key.address,ext->addr.sa_data,6);

		memcpy(wapi_key.key,ext + 1,32);

		return iwprivswapikey(&wapi_key);
	}
	#endif

	return -EINVAL;
}   

/*------------------------------------------------------------------*/
/*
 * Standard Wireless Handler : get encoding token & mode
 */
static int itm_giwencodeext(struct net_device *	dev,
			  struct iw_request_info *info,
			   union iwreq_data * wrqu,  char * extra)
{
	/*
	int ret = 0;
	UWORD8 * host_req  = NULL;
	UWORD8 * trout_rsp = NULL;
	UWORD16  trout_rsp_len = 0;
	*/

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	PRINTK_ITMIW("itm_giwencodeext fuc\n");
	
	return 0;
} 		
    /* WPA2 : PMKSA cache management */
/*------------------------------------------------------------------*/
/*
 * Standard Wireless Handler : PMKSA cache operation
 */
static int itm_siwpmksa(struct net_device *	dev,
			  struct iw_request_info *info,
			   union iwreq_data * wrqu,  char * extra)
{
	int ret = 0;
	UWORD8 * host_req  = NULL;
	UWORD16  trout_rsp_len = 0;
	struct iw_pmksa *pmksa = (struct iw_pmksa *)extra;
	int len = 0;

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

    PRINTK_ITMIW("itm_siwpmksa fuc \n");
    PRINTK_ITMIW("SIWPMKSA: cmd %s, %02x:%02x:%02x:%02x:%02x:%02x\n",
                (pmksa->cmd == IW_PMKSA_ADD) ? "IW_PMKSA_ADD" : 
                (pmksa->cmd == IW_PMKSA_REMOVE) ? "IW_PMKSA_REMOVE" : 
				(pmksa->cmd == IW_PMKSA_FLUSH) ? "IW_PMKSA_FLUSH" :	"Unknown command",
                pmksa->bssid.sa_data[0],
                pmksa->bssid.sa_data[1],
                pmksa->bssid.sa_data[2],
                pmksa->bssid.sa_data[3],
                pmksa->bssid.sa_data[4],
                pmksa->bssid.sa_data[5]);


	host_req = g_itm_config_buf;

	if( host_req == NULL)
	{
		return -ENOMEM;
	}

	switch (pmksa->cmd) {
		case IW_PMKSA_ADD:
			/*----------------------------------------------------------------------*/
		    /*    NumEntries  | BSSID[1] |  PMKID[1]  | BSSID[2] |  PMKID[2]  |     */
		    /*----------------------------------------------------------------------*/
		    /*       1        |    6     |    16      |    6     |    16      |     */
		    /*----------------------------------------------------------------------*/	
			host_req[len++] = WID_PMKID_INFO  & 0xFF;
			host_req[len++] = (WID_PMKID_INFO  & 0xFF00) >> 8;
			host_req[len++] = 1 + 6 + 16;

			/* NumEntries */
			host_req[len++] = 1;

			/* BSSID */
			memcpy(&host_req[len],pmksa->bssid.sa_data,6);
			len += 6;

			/* PMKID */
			memcpy(&host_req[len],pmksa->pmkid,16);
			len += 16;

			break;
		case IW_PMKSA_REMOVE:
			break;
		case IW_PMKSA_FLUSH:
			/* Replace current PMKID's with an empty list */
			break;
		default:
			PRINTK_ITMIW("SIWPMKSA: Unknown command (0x%x)\n", pmksa->cmd);
			return -EINVAL;
    }

	if(len == 0)
	{
		return 0;
	}

	config_if_for_iw( &g_mac,host_req,len,'W',&trout_rsp_len );

	if( trout_rsp_len != 1 )
	{
		ret = -EINVAL;
	}

	//kfree(host_req);
	return ret;
	
}


/*
static const iw_handler itm_private_handler[] fuc
*/


/* SIOCIWFIRSTPRIV */

/* SIOCIWFIRSTPRIV + 1 */
           
/* SIOCIWFIRSTPRIV + 2 */

 /* SIOCIWFIRSTPRIV + 7 ------> NULL*/

/* SIOCIWFIRSTPRIV + 8 */
             
#ifdef ITM_SUPPORT_WEXT_AP
#define PARAM_TYPE_INT 0
#define PARAM_TYPE_STRING 1
#define ITM_WIFI_MAX_SSID_LEN 32
#define ITM_WIFI_MAX_SEC_LEN 16
#define ITM_WIFI_MAX_KEY_LEN 65

#if 1
static int hex_look_up(char x)
{
    if(x>='0' && x<='9')
        return (x-48);
    if(x>= 'a' && x <= 'f')
        return (x-87);
    return -1;
}
#endif

static int power (int a, int b)
{
    int i;
    int num =1;
    for(i=0;i<b;i++)
       num *=a;
    return num;
}


/* SIOCIWFIRSTPRIV + 9 -------> NULL */


/*
decode_parameter_from_string sub fuc of itm_iwprivsapconfig
*/
#if 0
static int decode_parameter_from_string(
                char **str_ptr,const char *token, int param_type,
                void  *dst, int param_max_len)
{
    UWORD8 int_str[7] = "0";
    UWORD32 param_str_len;
    UWORD8  *param_str_begin,*param_str_end;
    UWORD8  *orig_str = *str_ptr;

    if (!strncmp(*str_ptr, token, strlen(token))) 
    {
        strsep(str_ptr, "=,");
        param_str_begin = *str_ptr;
        strsep(str_ptr, "=,");
        if (*str_ptr == NULL) 
        {
            param_str_len = strlen(param_str_begin);
        } else 
        {
            param_str_end = *str_ptr-1;
            param_str_len = param_str_end - param_str_begin;
        }
        PRINTK_ITMIW("'token:%s', len:%d, ", token, param_str_len);
        if (param_str_len > param_max_len) 
        {
            PRINTK_ITMIW("extracted param len:%d is > MAX:%d\n",
                                    param_str_len, param_max_len);
            param_str_len = param_max_len;
        }
        switch (param_type) 
        {
            case PARAM_TYPE_INT:
            {
                UWORD32 *pdst_int = dst,num =0;
                int i,j=0;
                if (param_str_len > sizeof(int_str)) 
                {
                    param_str_len = sizeof(int_str);
                }
                memcpy(int_str, param_str_begin, param_str_len);
                for(i = param_str_len; i>0;i--) 
                {
                    if(int_str[i-1] >= '0' && int_str[i-1] <='9') 
                    {
                        num += ((int_str[i-1]-'0')*power(10,j));
                        j++;
                    } else 
                    {
                        PRINTK_ITMIW("decode_parameter_from_string:not a number %c\n",(int_str[i-1]));
                        return -1;
                    }
                }
                *pdst_int = num;
                PRINTK_ITMIW("decode_parameter_from_string:decoded int = %d\n",*pdst_int);
            }
            break;
            default:
                memcpy(dst, param_str_begin, param_str_len);
                *((char *)dst + param_str_len) = 0;
                PRINTK_ITMIW("decode_parameter_from_string:decoded string = %s\n",(char *)dst);
            break;
        }
    } 
    else
    {
        PRINTK_ITMIW("decode_parameter_from_string: Token:%s not found in %s \n",token,orig_str);
        return -1;
    }
    return 0;
}

/*
store_ap_advanced_config_from_string sub fuc of itm_iwprivsapconfig
*/
static int store_ap_advanced_config_from_string(char *param_str)
{
    char * str_ptr=param_str;
    int ret = 0,tmp_var;
    char phy_mode[6];
    UWORD8 n_op_mode = 0;

    /* Check for BI */
    ret = decode_parameter_from_string(&str_ptr, "BI=",
                                       PARAM_TYPE_INT, &tmp_var, 5);
    if(ret) {
        PRINTK_ITMIW("store_ap_advanced_config_from_string: BI not found\n");
        return -1;
    }

    if(itm_set_beaconInterval(tmp_var) < 0 )
    {
        return -1;
    }

    ret = decode_parameter_from_string(&str_ptr, "DTIM_PER=",
                                        PARAM_TYPE_INT, &tmp_var, 5);
    if(ret) {
        PRINTK_ITMIW("store_ap_advanced_config_from_string: DTIM_PER not found\n");
        return -1;
    }

    if( itm_set_dtim_period(tmp_var) < 0 )
    {
        return -1;
    }
    
    ret = decode_parameter_from_string(&str_ptr, "WMM=",
                                        PARAM_TYPE_INT, &tmp_var, 5);
    if(ret) {
        PRINTK_ITMIW("store_ap_advanced_config_from_string: WMM not found\n");
        return -1;
    }

    if( itm_set_wmmEnable(tmp_var) < 0 )
    {
        return -1;
    }    
 
    ret = decode_parameter_from_string(&str_ptr, "PHY=",
                                        PARAM_TYPE_STRING, phy_mode, 5);
    if(ret) 
    {
        PRINTK_ITMIW("store_ap_advanced_config_from_string: PHY not found\n");
    } 
    else 
    {
        #if 0
       if(strstr(phy_mode,"b"))
       {

       }
       if(strstr(phy_mode,"g")) 
       {

       }
       #endif
       if(strstr(phy_mode,"n")) 
       {
           n_op_mode = 1;
       }

       if( itm_set_11g_op_mode(3) < 0)
       {
           return -1;
       }

       if(n_op_mode)
       {
            if(set_11n_enable(1) < 0)
            {
                return -1;
            }
       }
    }
    return ret;
}
/*
store_ap_config_from_string sub fuc of itm_iwprivsapconfig
*/
static int store_ap_config_from_string(char *param_str)
{
    char *str_ptr = param_str;
    char sub_cmd[16];
    char sec[ITM_WIFI_MAX_SEC_LEN];
    char key[ITM_WIFI_MAX_KEY_LEN];
    char ssid[ITM_WIFI_MAX_SSID_LEN];
    int  ssid_len = 0;
    int ret = 0;
	int tmp_var = 0;
	int i = 0;
	int j = 0;

    memset(sub_cmd, 0, sizeof(sub_cmd));
    if(!strstr(param_str,"END")) 
    {
        PRINTK_ITMIW("store_ap_config_from_string:Invalid config string:%s\n",param_str);
        return -1;
    }
    if (decode_parameter_from_string(&str_ptr, "ASCII_CMD=",
        PARAM_TYPE_STRING, sub_cmd, 6) != 0) 
    {
         return -1;
    }
    if (strncmp(sub_cmd, "AP_CFG", 6)) 
    {
        if(!strncmp(sub_cmd ,"ADVCFG", 6)) 
        {
           return store_ap_advanced_config_from_string(str_ptr);
        }
        PRINTK_ITMIW("store_ap_config_from_string: sub_cmd:%s != 'AP_CFG or ADVCFG'!\n", sub_cmd);
        return -1;
    }

    memset(ssid,0x00,sizeof(ssid));
    ret = decode_parameter_from_string(&str_ptr, "SSID=",
                                       PARAM_TYPE_STRING,ssid,
                                       ITM_WIFI_MAX_SSID_LEN);
    if(ret) 
    {
        PRINTK_ITMIW("store_ap_config_from_string: SSID not found\n");
        return -1;
    }
	ssid_len = strlen(ssid);

    ret = decode_parameter_from_string(&str_ptr, "SEC=",
                         PARAM_TYPE_STRING, sec, ITM_WIFI_MAX_SEC_LEN);
    if(ret) 
    {
        PRINTK_ITMIW("store_ap_config_from_string: SEC not found\n");
        return -1;
    }

    memset(key,0x00,sizeof(key));
    ret = decode_parameter_from_string(&str_ptr, "KEY=",
                         PARAM_TYPE_STRING,  key, ITM_WIFI_MAX_KEY_LEN);
    if(!strcasecmp(sec,"open")) 
    {
        PRINTK_ITMIW("store_ap_config_from_string: security open");

        if( itm_set_Auth_Type( (UWORD8)(ITM_OPEN_SYSTEM & 0xFF) ) < 0 )
		{
			return -1;
		}
        if(ret) 
        {
            PRINTK_ITMIW("store_ap_config_from_string: KEY not found:fine with Open\n");
        }
    }
    else if(!strcasecmp(sec,"wpa2-psk")) 
    {


        PRINTK_ITMIW("store_ap_config_from_string: security WPA2");
        if(ret) 
        {
            PRINTK_ITMIW("store_ap_config_from_string: KEY not found for WPA2\n");
            return -1;
        }

        if( itm_set_Auth_Type( (UWORD8)(ITM_OPEN_SYSTEM & 0xFF) ) < 0 )
		{
			return -1;
		}
		
        if( itm_set_Encryption_Type( (UWORD8)WPA2_AES_PSK ) < 0)
        {
            return -1;
        }

        /* cac psk */
        #if 1
        for(i=0;i<32;i++)
        {
           g_psk_value[i] = (16*hex_look_up(key[j]))+hex_look_up(key[j+1]);
           j+=2;
        }
        #else
        if( itm_set_psk_value(key,strlen(key)) < 0 )
        {
            return -1;
        }
        #endif

    }
    else 
    {
    
       PRINTK_ITMIW("store_ap_config_from_string: Unknown security: Assuming Open");
       if( itm_set_Auth_Type( (UWORD8)(ITM_OPEN_SYSTEM & 0xFF) ) < 0 )
	   {
			return -1;
	   }
       return -1;
    }

   /* Get the decoded value in a temp int variable to ensure that other fields within the struct
      which are of type other than int are not over written */
    ret = decode_parameter_from_string(&str_ptr, "CHANNEL=", PARAM_TYPE_INT, &tmp_var, 5);
    if(ret)
    {
        return -1;
    }

    if( itm_set_channel(tmp_var) < 0 )
    {
        return -1;
    }


    ret = decode_parameter_from_string(&str_ptr, "PREAMBLE=", PARAM_TYPE_INT, &tmp_var, 5);
    if(ret)
        return -1;

	#if 0
	if(itm_set_preamble(tmp_var) < 0)
    {
        return -1;
    }
	#endif

	return itm_set_essid(ssid,ssid_len);
	
	//ret = decode_parameter_from_string(&str_ptr, "MAX_SCB=", PARAM_TYPE_INT,  &tmp_var, 5);
    /* in itm code this option is set internel*/

	
    return ret;
}

/* SIOCIWFIRSTPRIV + 10 */
static int
iwprivsapconfig(struct net_device *dev, struct iw_request_info *info,
                union iwreq_data *wrqu, char *extra)
{
    

    char  *cfg_str = NULL;
    int r;

	union iwreq_data tmp_wrqu;
	char tmp_data[]  = "AP_SET_CFG_OK";

    PRINTK_ITMIW("itm_iwprivsapconfig \n");

#if 0//20120731 caisf add for enable p2p function
    PRINTK_ITMIW("P2P: caisf skip iwprivsapconfig!\n");
	return 0;
#endif

    if (wrqu->data.length != 0) 
    {
        char *str;
        if (!(cfg_str = kmalloc(wrqu->data.length+1, GFP_KERNEL)))
        {
            return -ENOMEM;
        }
        if (copy_from_user(cfg_str, wrqu->data.pointer, wrqu->data.length)) 
        {
            kfree(cfg_str);
            return -EFAULT;
        }
        cfg_str[wrqu->data.length] = 0;
        PRINTK_ITMIW("length:%d\n",wrqu->data.length);
        PRINTK_ITMIW("AP configuration string:%s\n",cfg_str);
        str = cfg_str;
        if( (r = store_ap_config_from_string(str)) )
        {
           PRINTK_ITMIW("iwprivsapconfig:Failed  to decode the string %d\n",r);
           kfree(cfg_str);
           return -EIO;
        }
    } 
    else 
    {
        PRINTK_ITMIW("iwprivsapconfig argument length = 0 \n");
        return -EIO;
    }

	tmp_wrqu.data.length = sizeof(tmp_data);
	tmp_wrqu.data.flags = 0;
	wireless_send_event(g_mac_dev, IWEVCUSTOM, &tmp_wrqu, tmp_data);

    kfree(cfg_str);
    return 0;
}
#endif

/* SIOCIWFIRSTPRIV + 11 -------> NULL */    

/* SIOCIWFIRSTPRIV + 12 */
static int
iwprivsapstart(struct net_device *dev, struct iw_request_info *info,
               union iwreq_data *wrqu, char *extra)
{
	union iwreq_data tmp_wrqu;
	char data[]  = "START";

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	PRINTK_ITMIW("itm_iwprivsapstart \n");
	
	tmp_wrqu.data.length = sizeof(data);
	tmp_wrqu.data.flags = 0;
#ifdef CONFIG_TROUT_WEXT
		wireless_send_event(g_mac_dev, IWEVCUSTOM, &tmp_wrqu, data);
#endif
    return 0;
}

/* SIOCIWFIRSTPRIV + 13 -------> NULL */    

/* SIOCIWFIRSTPRIV + 14 */
static int
iwprivsapstop(struct net_device *dev, struct iw_request_info *info,
        union iwreq_data *wrqu, char *extra)
{
	union iwreq_data tmp_wrqu;
	char data[] = "STOP";

	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

	PRINTK_ITMIW("itm_iwprivsapstop \n");
	
	tmp_wrqu.data.length = sizeof(data);
	tmp_wrqu.data.flags = 0;
#ifdef CONFIG_TROUT_WEXT
		wireless_send_event(g_mac_dev, IWEVCUSTOM, &tmp_wrqu, data);
#endif
    return 0;
}

/* SIOCIWFIRSTPRIV + 15 -------> NULL */
    
#ifdef ANDROID_BUILD
/* SIOCIWFIRSTPRIV + 16 */
static int
iwprivsapfwreload(struct net_device *dev, struct iw_request_info *info,
                  union iwreq_data *wrqu, char *extra)
{
	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

    PRINTK_ITMIW("itm_iwprivsapfwreload \n");
    //return -ESRCH;
    return 0;
}

/* SIOCIWFIRSTPRIV + 17 -------> NULL */    

/* SIOCIWFIRSTPRIV + 18 */
static int
iwprivsstackstart(struct net_device *dev, struct iw_request_info *info,
        union iwreq_data *wrqu, char *extra)
{
	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

    PRINTK_ITMIW("itm_iwprivsstackstart \n");
    //return -ESRCH;
    return 0;
}

/* SIOCIWFIRSTPRIV + 19 -------> NULL */    

/* SIOCIWFIRSTPRIV + 20 */
static int
iwprivsstackstop(struct net_device *dev, struct iw_request_info *info,
        union iwreq_data *wrqu, char *extra)
{
	CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

    PRINTK_ITMIW("itm_iwprivsstackstop \n");
    //return -ESRCH;
    return 0;
}

#endif /* ANDROID_BUILD */
#endif /* ITM_SUPPORT_WEXT_AP */


/* SIOCIWFIRSTPRIV + 21  */


/* SIOCIWFIRSTPRIV + 22 */    



/* SIOCIWFIRSTPRIV + 23  */


#if 1
//chenq add for IC test
/* SIOCIWFIRSTPRIV + 24  */
static int
itm_ICtest(struct net_device *dev, struct iw_request_info *info,
        union iwreq_data *wrqu, char *extra)
{
	struct iw_encode_ext *ext = (struct iw_encode_ext *)extra;
	static UWORD32 is_ap  = 0;
	static UWORD32 is_sta = 0;
	static UWORD32 key_id = 0;
	static UWORD32 enc_type    = 0;
	static UWORD8 ap_name[34] = {0};
	static UWORD8 ap_channel  = 0;
	char * tmp_char = ext->key;
	static UWORD8 wid_go = 0;

	UWORD8 * host_req  = NULL;
	UWORD16  trout_rsp_len = 0;
	int len = 0;
	int ret = 0;

	PRINTK_ITMIW("itm_ICtest \n");
	PRINTK_ITMIW("itm_ICtest flag:%d\n",ext->ext_flags);
	//PRINTK_ITMIW("0x%02x\n",ext->key[0]);
	//PRINTK_ITMIW("0x%02x\n",ext->key[1]);
	//PRINTK_ITMIW("0x%02x\n",ext->key[2]);
	//PRINTK_ITMIW("0x%02x\n",ext->key[3]);
	//PRINTK_ITMIW("0x%02x\n",ext->key[4]);
	//PRINTK_ITMIW("0x%02x\n",ext->key[5]);
	switch( ext->ext_flags )
	{
		case 1:
		{
			//is_ap mode
			is_ap = 1;
			break;
		}
		case 2:
		{
			//is_sta
			is_sta = 1;
			break;
		}
		case 3:
		{
			//key_id
			key_id = *tmp_char - 1;
			//PRINTK_ITMIW("key_id = %d\n",*tmp_char & 0xFF);
			break;
		}
		case 4:
		{
			//enc_type
			if(*tmp_char == 0)
	   		{
	   			enc_type = 0x00;
	   		}
			else if(*tmp_char == 1)
			{
				enc_type = 0x49;
			}
			else if(*tmp_char == 2)
			{
				enc_type = 0x31;
			}
			else if(*tmp_char == 3)
			{
				enc_type = 0x03;
			}
			else if(*tmp_char == 4)
			{
				enc_type = 0x07;
			}
			//PRINTK_ITMIW("enc_type = %02x\n",(*tmp_char) & 0xFF);
			break;
		}
		case 5:
		{
			//ap_mac
			//memcpy(ap_mac,tmp_char,sizeof(ap_mac));
			break;
		}
		case 6:
		{
			//sta_mac
			//memcpy(sta_mac,tmp_char,sizeof(sta_mac));
			break;
		}
		case 7:
		{
			//ap_name
			strncpy(ap_name,tmp_char,sizeof(ap_name)-1);
			//PRINTK_ITMIW("ap_name = %s\n",tmp_char);
			break;
		}
		case 8:
		{
			//ap_channel
			ap_channel = *tmp_char;
			//PRINTK_ITMIW("ap_channel = %d\n",(*tmp_char) & 0xFF);
			break;
		}
		case 9:
		{
			//password and begin wid
			wid_go = 1;
			break;
		}
		default:
			break;
	}

	if(wid_go == 0)
	{
		//PRINTK_ITMIW("itm_ICtest return \n");
		return 0;
	}
	else
	{
		//PRINTK_ITMIW("itm_ICtest go wid \n");
		wid_go = 0;
	}

	/*make wid pkt*/
	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if(host_req == NULL)
	{
		return -ENOMEM;
	}

	host_req[len+0] = WID_11I_MODE & 0xFF;
    host_req[len+1] = (WID_11I_MODE & 0xFF00) >> 8;
	host_req[len+2] = 1; 
	host_req[len+3] = enc_type; 
	len += WID_CHAR_CFG_LEN;

    host_req[len+0] = WID_AUTH_TYPE & 0xFF;
    host_req[len+1] = (WID_AUTH_TYPE & 0xFF00) >> 8;
	host_req[len+2] = 1; 
	host_req[len+3] = 1; 
	len += WID_CHAR_CFG_LEN;

	if(enc_type > 0x07)
	{
		//wpa/wpa2
		host_req[len+0] = WID_11I_PSK & 0xFF;
		host_req[len+1] = (WID_11I_PSK & 0xFF00) >> 8;
		host_req[len+2] = ext->key_len; 
		memcpy(&host_req[len+3],ext->key,ext->key_len);
		len += (3 + ext->key_len);
	}
	else if(enc_type > 0x00)
	{
		/*
		//wep
		host_req[len+0] = WID_ADD_WEP_KEY & 0xFF;
		host_req[len+1] = (WID_ADD_WEP_KEY & 0xFF00) >> 8;

		host_req[len+2] = (ext->key_len + 1 + 1) & 0xFF;
	    host_req[len+3] = key_id & 0xFF;
	    host_req[len+4] = ext->key_len & 0xFF;
		memcpy(&host_req[len+5],ext->key,ext->key_len);
		host_req[len + 5 + ext->key_len] = 0x00;
		
		len+= 5 + ext->key_len;
		*/
		
		UWORD8 * ppkey = ext->key;
		int i = 0;

		host_req[len+0] = WID_KEY_ID & 0xFF;
		host_req[len+1] = (WID_KEY_ID & 0xFF00) >> 8;
		host_req[len+2] = 1;
		host_req[len+3] = key_id & 0xFF;
		len += WID_CHAR_CFG_LEN;

		host_req[len+0] = WID_WEP_KEY_VALUE & 0xFF;
		host_req[len+1] = (WID_WEP_KEY_VALUE & 0xFF00) >> 8;
		host_req[len+2] = ext->key_len * 2;

		memcpy(&host_req[len+3],ext->key,ext->key_len);
		for( i = 0 ; i < ext->key_len ; i++)
		{
			host_req[len+3+2*i] = hex_2_char((UWORD8)( (*ppkey & 0xF0) >> 4));
			host_req[len+3+2*i + 1] = hex_2_char((UWORD8)( *ppkey & 0x0F));

			ppkey++;
		}
		len +=WID_HEAD_LEN + ext->key_len * 2;
		
	}

	host_req[len+0] = WID_SSID & 0xFF;
	host_req[len+1] = (WID_SSID & 0xFF00) >> 8;
	host_req[len+2] = strlen(ap_name);
	memcpy(&host_req[len+3],ap_name,strlen(ap_name));
	len += 3 + strlen(ap_name);
		
	if(is_ap == 1)
	{
	   //channel
	   host_req[len+0] = WID_PRIMARY_CHANNEL & 0xFF;
	   host_req[len+1] = (WID_PRIMARY_CHANNEL & 0xFF00) >> 8;
	   host_req[len+2] = 1;
	   host_req[len+3] = ap_channel & 0xFF;
	   len += WID_CHAR_CFG_LEN;	

	   is_ap = 0;
	}
	else if(is_sta == 1)
	{
		/*do nothing*/;
		is_sta = 0;
	}
	else
	{
		ret = -ENOTSUPP;
		goto out1;
	}
	

	config_if_for_iw(&g_mac,host_req,len,'W',&trout_rsp_len);

	if( trout_rsp_len != 1 )
	{
		ret = -EINVAL;
	}

	out1:
	//kfree(host_req);
	return ret;
}

#endif

#if 0

static char itm_debug_data_buf[IW_ESSID_MAX_SIZE+1] = {0};

INLINE int str2ui(char * str,int len,UWORD32 * ui)
{
	int i = 0;
	int max_str2ui_len = 8;
	UWORD8 tmp_uchar = 0;

 	if(len > max_str2ui_len)
 	{
 		return -EINVAL;
 	}

	*ui = 0;
	for( i = 0 ; i < len ; i++ )
	{
		if(isxdigit(str[i]))
		{
			tmp_uchar = tolower(str[i]);
			if(  ( tmp_uchar >= '0' ) && ( tmp_uchar <= '9' ) )
			{
				tmp_uchar -= '0'; 
			}
			else
			{
				tmp_uchar -= 'a'; 
			}
		}
		else
		{
			return -EINVAL;
		}

		*ui = (*ui << 4) | tmp_uchar ;
	}

	return 0;
}

INLINE int itm_debug_read_reg(char * reg_type,char * reg_addr,char * reg_data)
{
	UWORD32 addr = 0;
	UWORD32 data = 0;
	int index = 0;

	if( str2ui(&reg_addr[index],3,&addr) < 0 )
	{
		return -EINVAL;
	}

	if( !strcmp(reg_type,itm_debug_all_reg_flag) )
	{
		/* read all reg of all reg type */
	}
	else if(  !strcmp(reg_type,itm_debug_user_ind_flag) )
	{
		/* user indicate all reg if some type regs*/
	}
	else if(addr == 0xFFF)
	{
		/* read all reg of some one reg type */
	}
	else if( !strcmp(reg_type,itm_debug_pa_flag) )
	{
		data = convert_to_le(host_read_trout_reg((UWORD32)(PA_BASE + addr)));
	}
	else if( !strcmp(reg_type,itm_debug_ce_flag) )
	{
		data = convert_to_le(host_read_trout_reg((UWORD32)(CE_BASE + addr)));
	}
	else if( !strcmp(reg_type,itm_debug_cm_flag) )
	{
		data = convert_to_le(host_read_trout_reg((UWORD32)(TROUT_COMM_REG + addr)));
	}
	else if( !strcmp(reg_type,itm_debug_py_flag) )
	{
		read_dot11_phy_reg((UWORD8)addr,&data);
	}
	else if( !strcmp(reg_type,itm_debug_sys_flag) )
	{
		data = convert_to_le(host_read_trout_reg((UWORD32)(0x00 + addr)));
	}
	else if( !strcmp(reg_type,itm_debug_rf_flag) )
	{
		return -EINVAL;
	}
	else if( !strcmp(reg_type,itm_debug_ada_flag) )
	{
		return -EINVAL;
	}
	else 
	{
		return -EINVAL;
	}

	sprintf(reg_data,"%08x",data);
	return 0;
}


INLINE int itm_debug_write_reg(char * reg_type,char * reg_write)
{
	UWORD32 addr = 0;
	UWORD32 data = 0;
	int index = 0;

	if( str2ui(&reg_write[index],3,&addr) < 0)
	{
		return -EINVAL;
	}

	index += 3;
	if( str2ui(&reg_write[index],8,&data) < 0)
	{
		return -EINVAL;
	}

	if( !strcmp(reg_type,itm_debug_pa_flag) )
	{
        host_write_trout_reg(convert_to_le(data),(UWORD32)(PA_BASE + addr));
	}
	else if( !strcmp(reg_type,itm_debug_ce_flag) )
	{
		host_write_trout_reg(convert_to_le(data),(UWORD32)(CE_BASE + addr));
	}
	else if( !strcmp(reg_type,itm_debug_cm_flag) )
	{
		host_write_trout_reg(convert_to_le(data),(UWORD32)(TROUT_COMM_REG + addr));
	}
	else if( !strcmp(reg_type,itm_debug_py_flag) )
	{
		write_dot11_phy_reg( (UWORD8)addr,data);
	}
	else if( !strcmp(reg_type,itm_debug_sys_flag) )
	{
		host_write_trout_reg(convert_to_le(data),(UWORD32)(0x00 + addr));
	}
	else if( !strcmp(reg_type,itm_debug_rf_flag) )
	{
		write_RF_reg_ittiam( (UWORD8)addr,data );
	}
	else if( !strcmp(reg_type,itm_debug_ada_flag) )
	{
		write_mxfe_reg((UWORD8)addr,(UWORD8)data);
	}
	else 
	{
		return -EINVAL;
	}

	return 0;
}


INLINE int itm_debug_print_log(char setget_flag,char * log_type,char * log_flag)
{
	int index = 0;
	int type  = 0;
	int flag  = 0;
	int value = 0;

	if( !strcmp(log_type,itm_debug_plog_sharemem) )
	{
		if( str2ui(&log_flag[index],8,&type) < 0)
		{
			return -EINVAL;
		}
		index +=8;

		flag = (setget_flag == 's') ? 0 : 1;
		/*ShareMemInfo(type,flag,value,NULL);*/

		if( (log_flag[index] != 0x00) && (str2ui(&log_flag[index],8,&type) < 0) )
		{
			return -EINVAL;
		}
		
		return 0;
	}
	else if( !strcmp(log_type,itm_debug_plog_mactxrx) )
	{ 
		if( str2ui(&log_flag[index],8,&type) < 0)
		{
			return -EINVAL;
		}

		flag = (setget_flag == 's') ? 0 : 1;
		/* use api to print*/
		/* MacTxRxStatistics(type,flag,NULL)*/
		return 0;
	}
	else if( !strcmp(log_type,itm_debug_plog_spisdiodma) )
	{
		if( str2ui(&log_flag[index],8,&type) < 0)
		{
			return -EINVAL;
		}
		index+=8;

		flag = (setget_flag == 's') ? 0 : 1;

		
		if( (log_flag[index] != 0x00) && (str2ui(&log_flag[index],8,&value) < 0) )
		{
			return -EINVAL;
		}

		/*SpiSdioDmaState(type,flag,value,NULL);*/
		return 0;
	}
	else if( !strcmp(log_type,itm_debug_plog_macfsm_mib) )
	{
		if( str2ui(&log_flag[index],8,&type) < 0)
		{
			return -EINVAL;
		}
		index+=8;

		if( (log_flag[index] != 0x00) && (str2ui(&log_flag[index],8,&value) < 0) )
		{
			return -EINVAL;
		}
	
		/*MacFsmMibState(type,value,NULL)*/
		return 0;
	}
	else if( !strcmp(log_type,itm_debug_plog_host6820info) )
	{
		if( str2ui(&log_flag[index],8,&type) < 0)
		{
			return -EINVAL;
		}

		/*Host6820Info(type,NULL)*/
		return 0;
	}
	else
	{
		return -EINVAL;
	}
}
#endif

#define FM_BASE             (0x00003000)

static int
itm_gfordebug(struct net_device *dev, struct iw_request_info *info,
        union iwreq_data *wrqu, char *extra)
{
#ifdef DEBUG_MODE

#if 0
	char * tmp_extra = extra;
	char debug_type[3] = {0};
	char debug_subtype[3] = {0};
	int index = 0;

	PRINTK_ITMIW("itm_gfordebug \n");

	if( itm_debug_data_buf[index] != itm_debug_g_flag )
	{
		memset(itm_debug_data_buf,0x00,sizeof(itm_debug_data_buf));
		return -EINVAL;
	}

	index++;
	strncpy(debug_type,&itm_debug_data_buf[index],2);
	index += 2;
	strncpy(debug_subtype,&itm_debug_data_buf[index],2);
	index += 2;

	if( !strcmp(debug_type,itm_debug_reg_flag) )
	{
		return itm_debug_read_reg(debug_subtype,&itm_debug_data_buf[index],tmp_extra);

	}
	else if( !strcmp(debug_type,itm_debug_plevel_flag) )
	{
		return 0;
	}
	else if( !strcmp(debug_type,itm_debug_plog_flag) )
	{
		return 0;
	}
	else 
	{
		return -EINVAL;
	}
	return 0;
#else	
	UWORD32           temp = 0;
    WORD32            i    = 0;
    //mem_access_info_t mem_access_info;
	mem_access_info_t * tmp_extra = (mem_access_info_t *)extra;

	/*wrqu->encoding.length*/
	/*high 8=>read_write,low 8=>count*/
	int read_write = (wrqu->encoding.length & 0xFF00) >> 8;
	int count = wrqu->encoding.length & 0xFF;
	int addr  = wrqu->encoding.flags & 0xFFFF;

	PRINTK_ITMIW("itm_gfordebug \n");

	printk("chenq debug read read_write = 0x%x,count = %d,addr = 0x%x\n",
		read_write,count,addr);

/* Read/Write to the MAC H/w PA registers */
    if(read_write == MAC_PA_READ_FLAG)
    {
        temp = (UWORD32)(PA_BASE + addr);

        for(i = 0; i < count; i++)
        {
            tmp_extra->data[i] = convert_to_le(host_read_trout_reg(temp + i*4));
        }
    }
    /* Read/Write to the MAC H/w CE registers */
    else if(read_write == MAC_CE_READ_FLAG)
    {
        temp = (UWORD32)(CE_BASE + addr);

        for(i = 0; i < count; i++)
        {
            tmp_extra->data[i] = convert_to_le(host_read_trout_reg(temp + i*4));
        }
    }
    else if(read_write == PHY_READ_FLAG)
    {
    	UWORD32 regret = 0;
        for(i = 0; i < count; i++)
        {
            read_dot11_phy_reg(((UWORD8)addr + i),
                               &(tmp_extra->data[i]));
			//TROUT_PRINT("itmrw addr = 0x%x, data = 0x%x\n",((UWORD8)addr + i),(tmp_extra->data[i]));
        }
    }
    else if(read_write == MAC_DEBUG_FLAG)
    {
        if(count == 0)
        {
            if(g_enable_debug_print > 0)
                g_enable_debug_print--;
            PRINTK("Current Debug Level = %d\n",g_enable_debug_print);
        }
        else if(count == 1)
        {
            if(g_enable_debug_print < MAX_DEBUG_PRINT_LEVEL)
                g_enable_debug_print++;
            PRINTK("Current Debug Level = %d\n",g_enable_debug_print);
        }
        else if(count == 2)
            print_test_stats(0);
        else if(count == 5)
            print_debug_stats();
#ifdef MAC_HW_UNIT_TEST_MODE
        else if(count == 3)
            print_frm_stats();
        else if(count == 4)
            reset_frm_stats();
#endif /* MAC_HW_UNIT_TEST_MODE */
        else if(count == 6)
            print_build_flags();
        else if(count == 7)
            print_ba_debug_stats();
    }
	else if(read_write == MAC_COM_READ_FLAG)
	{
		temp = (UWORD32)(TROUT_COMM_REG + addr);

        for(i = 0; i < count; i++)
        {
            tmp_extra->data[i] = convert_to_le(host_read_trout_reg(temp + i*4));
        }
	}
	else if(read_write == MAC_SYS_READ_FALG)
	{
		temp = (UWORD32)(0x00 + addr);

        for(i = 0; i < count; i++)
        {
            tmp_extra->data[i] = convert_to_le(host_read_trout_reg(temp + i*4));
        }
	}
    else if(read_write == MAC_RF_READ_FLAG)
	{
		temp = (UWORD32)(0x00 + addr);
		//printk("itmrw read rf addr = 0x%x , temp = 0x%x\n", addr, temp);

		#ifdef TROUT_WIFI_EVB
        for(i = 0; i < count; i++)
        {
            trout_ic_rf_reg_read((temp + i),
                               &(tmp_extra->data[i]));
		    //printk("itmrw read rf reg = 0x%x , val = 0x%x\n", (temp + i), tmp_extra->data[i]);
        }
		#endif
		
		#ifdef TROUT2_WIFI_IC
		  	temp = ((UWORD32)(0x1000 + addr)<<2);
          	tmp_extra->data[i] =
		  		convert_to_le(host_read_trout_reg(temp + i*4));      
		#endif
	}
    else if(tmp_extra->read_write == FM_REG_READ_FLAG)
	{
        temp = (UWORD32)((FM_BASE + addr) << 2);
        for(i = 0; i < count; i++)
        {
            tmp_extra->data[i] = convert_to_le(host_read_trout_reg(temp + i*4));
        }
    }

	return 0;
#endif	

#endif
}

static int
itm_sfordebug(struct net_device *dev, struct iw_request_info *info,
        union iwreq_data *wrqu, char *extra)
{
#ifdef DEBUG_MODE

#if 0
	struct iw_point * data = (struct iw_point *)wrqu;
	char * tmp_extra = extra;
	char debug_type[3] = {0};
	char debug_subtype[3] = {0};
	int index = 0;
	char setget_flag = 0;

	PRINTK_ITMIW("itm_sfordebug \n");

	//if user get ,will save user get cmd in this fuc
	setget_flag = tmp_extra[index];
	index++;
	strncpy(debug_type,&tmp_extra[index],2);
	index += 2;
	
	if(    ( setget_flag == itm_debug_g_flag ) 
		&& (!strcmp(debug_type,itm_debug_reg_flag)) )
	{
		if( data->length > IW_ESSID_MAX_SIZE )
		{
			return -E2BIG;
		}
		sprintf(itm_debug_data_buf,"%s",extra);
		return 0;
	}

	
	strncpy(debug_subtype,&tmp_extra[index],2);
	index += 2;

	if( !strcmp(debug_type,itm_debug_reg_flag) )
	{
		return itm_debug_write_reg(debug_subtype,&tmp_extra[index]);

	}
	else if( !strcmp(debug_type,itm_debug_plevel_flag) )
	{
		return 0;
	}
	else if( !strcmp(debug_type,itm_debug_plog_flag) )
	{
		return itm_debug_print_log(setget_flag,debug_subtype,&tmp_extra[index]);
	}
	else 
	{
		return -EINVAL;
	}

	return 0;
#else
	UWORD32           temp = 0;
    WORD32            i     = 0;
    //mem_access_info_t mem_access_info;
	mem_access_info_t * tmp_extra = (mem_access_info_t *)extra;

	PRINTK_ITMIW("itm_sfordebug \n");
	printk("chenq debug write read_write = 0x%x,count = %d,addr = 0x%x\n",
		tmp_extra->read_write,tmp_extra->count,tmp_extra->data[0]);

    if(tmp_extra->read_write == MAC_PA_WRITE_FLAG)
    {
        temp = (UWORD32)(PA_BASE + tmp_extra->addr);
        host_write_trout_reg(convert_to_le(tmp_extra->data[0]),temp);
    }
    else if(tmp_extra->read_write == MAC_CE_WRITE_FLAG)
    {
        temp = (UWORD32)(CE_BASE + tmp_extra->addr);
        host_write_trout_reg(convert_to_le(tmp_extra->data[0]),temp);
    }
    else if(tmp_extra->read_write == PHY_WRITE_FLAG)
    {
    	UWORD32 regret = 0;
        write_dot11_phy_reg((UWORD8)tmp_extra->addr,
                            tmp_extra->data[0]);
		read_dot11_phy_reg(((UWORD8)tmp_extra->addr),
						   &regret);
		//TROUT_PRINT("itmrw PHY_WRITE_FLAG reread addr = 0x%x, data = 0x%x\n",((UWORD8)tmp_extra->addr),regret);
		
    }
    else if(tmp_extra->read_write == MAC_DEBUG_FLAG)
    {
        if(tmp_extra->count == 0)
        {
            if(g_enable_debug_print > 0)
                g_enable_debug_print--;
            PRINTK("Current Debug Level = %d\n",g_enable_debug_print);
        }
        else if(tmp_extra->count == 1)
        {
            if(g_enable_debug_print < MAX_DEBUG_PRINT_LEVEL)
                g_enable_debug_print++;
            PRINTK("Current Debug Level = %d\n",g_enable_debug_print);
        }
        else if(tmp_extra->count == 2)
            print_test_stats(0);
        else if(tmp_extra->count == 5)
            print_debug_stats();
        else if(tmp_extra->count == 8)	//add by chengwg.
        	dump_allregs(0, 0);
#ifdef MAC_HW_UNIT_TEST_MODE
        else if(tmp_extra->count == 3)
            print_frm_stats();
        else if(tmp_extra->count == 4)
            reset_frm_stats();
#endif /* MAC_HW_UNIT_TEST_MODE */
        else if(tmp_extra->count == 6)
            print_build_flags();
        else if(tmp_extra->count == 7)
            print_ba_debug_stats();
    }
	else if(tmp_extra->read_write == MAC_COM_WRITE_FALG)
	{
		temp = (UWORD32)(TROUT_COMM_REG + tmp_extra->addr);
        host_write_trout_reg(convert_to_le(tmp_extra->data[0]),temp);
	}
	else if(tmp_extra->read_write == MAC_SYS_WRITE_FLAG)
	{
		temp = (UWORD32)(0x00 + tmp_extra->addr);
        host_write_trout_reg(convert_to_le(tmp_extra->data[0]),temp);
	}
	else if(tmp_extra->read_write == MAC_RF_WRITE_FLAG)
	{
		UWORD32 regret = 0;
		#ifdef TROUT_WIFI_EVB
		trout_ic_rf_reg_write( tmp_extra->addr,tmp_extra->data[0] );
		//trout_ic_rf_reg_read((tmp_extra->addr),
						   //&regret);

		//TROUT_PRINT("itmrw reread writed addr = 0x%x, data = 0x%x\n",((UWORD8)tmp_extra->addr),regret);
        #endif
		
		#ifdef TROUT2_WIFI_IC
		temp = ((UWORD32)(0x1000 + tmp_extra->addr) << 2);
		host_write_trout_reg(convert_to_le(tmp_extra->data[0]),temp);
		//TROUT_PRINT("itmrw reread writed addr = 0x%x, data = 0x%x\n",((UWORD8)tmp_extra->addr),regret);
		#endif 
	}
	else if(tmp_extra->read_write == FM_REG_WRITE_FLAG)
	{
        temp = (UWORD32)((FM_BASE + tmp_extra->addr) << 2);
        host_write_trout_reg(convert_to_le(tmp_extra->data[0]),temp);
	}
    // add by Ke.Li at 2013-04-11 for itm_config read ram data
    else if(tmp_extra->read_write == TROUT_RAM_READ_FLAG)
    {
        display_trout_ram(SHARE_MEM_BEGIN + tmp_extra->addr, tmp_extra->count);
    }
    // end add by Ke.Li at 2013-04-11 for itm_config read ram data

	return 0;
#endif	

#endif
}


//chenq add for 11n
typedef struct
{
	int flag;
	int len;
    UWORD8 data[20];
}itm_11n_config_t;

static int
itm_sfor11n(struct net_device *dev, struct iw_request_info *info,
        union iwreq_data *wrqu, char *extra)
{
	itm_11n_config_t * tmp_extra = (itm_11n_config_t *)extra;
	
	static UWORD8 * host_req  = NULL;
	static int host_req_len = 0;

	UWORD16  trout_rsp_len = 0;

	int ret = 0;

	PRINTK_ITMIW("itm_sfor11n \n");
	PRINTK_ITMIW("%d,%d\n",tmp_extra->flag,tmp_extra->len);

	if(NULL == host_req)
	{
		//if( (host_req = (UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL))
			//== NULL )
		if( (host_req = g_itm_config_buf) == NULL )	
		{
			return -ENOMEM;
		}
		
		host_req[host_req_len+0] = WID_11N_IMMEDIATE_BA_ENABLED & 0xFF;
    	host_req[host_req_len+1] = (WID_11N_IMMEDIATE_BA_ENABLED & 0xFF00) >> 8;
		host_req[host_req_len+2] = 1;
		host_req[host_req_len+3] = 1;
		host_req_len += WID_CHAR_CFG_LEN;
		
	}

	host_req[host_req_len+0] = WID_11N_P_ACTION_REQ & 0xFF;
    host_req[host_req_len+1] = (WID_11N_P_ACTION_REQ & 0xFF00) >> 8;
	host_req[host_req_len+2] = tmp_extra->len;
	memcpy(&host_req[host_req_len+3],tmp_extra->data,tmp_extra->len);
	host_req_len += WID_HEAD_LEN + tmp_extra->len;

    //more config ?
	if(tmp_extra->flag == 1)
		return 0;

	config_if_for_iw(&g_mac,host_req,host_req_len,'W',&trout_rsp_len);

	if( trout_rsp_len != 1 )
	{
		ret = -EINVAL;
	}
	   

	//kfree(host_req);
    host_req = NULL;
	host_req_len = 0;
	
	return ret;
}


//chenq add for wid
typedef struct
{
	int wid;
	int len;
	int more_frag;
	int more_wid;
    UWORD8 data[256];
} itm_wid_config_t;

static int
itm_sforwid(struct net_device *dev, struct iw_request_info *info,
        union iwreq_data *wrqu, char *extra)
{
	itm_wid_config_t * tmp_extra = (itm_wid_config_t *)extra;
	
	static UWORD8 * host_req  = NULL;
	static int host_req_len = 0;
	static int len_index = 0;
	static int wid_data_len = 0;

	UWORD16  trout_rsp_len = 0;

	int ret = 0;

	PRINTK_ITMIW("itm_sforwid \n");
	PRINTK_ITMIW("wid:%04x,len = %d,more_frag = %d,more_wid = %d\n",
		          tmp_extra->wid,tmp_extra->len,tmp_extra->more_frag,tmp_extra->more_wid);

	if(NULL == host_req)
	{
		//if( (host_req = (UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL))
			//== NULL )
		if( (host_req = g_itm_config_buf) == NULL )	
		{
			return -ENOMEM;
		}
	}

    // tmp_extra->more_frag: 0 no frag,1 head frag 2 ...  3 end frag
    if(tmp_extra->more_frag == 0)
	{
	    // no frag 
		host_req[host_req_len+0] = tmp_extra->wid & 0xFF;
    	host_req[host_req_len+1] = (tmp_extra->wid & 0xFF00) >> 8;
		host_req[host_req_len+2] = tmp_extra->len;
		memcpy(&host_req[host_req_len+3],tmp_extra->data,tmp_extra->len);
		host_req_len += WID_HEAD_LEN + tmp_extra->len;
    }
	else if(tmp_extra->more_frag == 1)
	{
		//first frag
		host_req[host_req_len+0] = tmp_extra->wid & 0xFF;
    	host_req[host_req_len+1] = (tmp_extra->wid & 0xFF00) >> 8;
		//host_req[host_req_len+2] = tmp_extra->len;
		len_index = host_req_len+2;
		memcpy(&host_req[host_req_len+3],tmp_extra->data,tmp_extra->len);
		host_req_len += WID_HEAD_LEN + tmp_extra->len;
		wid_data_len += tmp_extra->len;
	}
	else if(tmp_extra->more_frag == 2)
	{
		memcpy(&host_req[host_req_len+0],tmp_extra->data,tmp_extra->len);
		host_req_len += tmp_extra->len;
		wid_data_len += tmp_extra->len;
	}
	else if(tmp_extra->more_frag == 3)
	{
		memcpy(&host_req[host_req_len+0],tmp_extra->data,tmp_extra->len);
		host_req_len += tmp_extra->len;
		wid_data_len += tmp_extra->len;

		host_req[len_index] = wid_data_len;
		len_index = 0;
		wid_data_len = 0;
	}
	
    //more config ?
	if(tmp_extra->more_wid == 1)
		return 0;

	config_if_for_iw(&g_mac,host_req,host_req_len,'W',&trout_rsp_len);

	if( trout_rsp_len != 1 )
	{
		ret = -EINVAL;
	}
	   
	//kfree(host_req);
	
    host_req = NULL;
	host_req_len = 0;
	len_index = 0;
	wid_data_len = 0;
	
	return ret;
}

static int
itm_swdirect(struct net_device *dev, struct iw_request_info *info,
        			union iwreq_data *wrqu, char *extra)
{
	UWORD8 * extra_data = (UWORD8 * )extra;
	UWORD8 cmd[64] = {0};
	int tmp_index = 0;

	int ret = 0;
	UWORD8 * host_req  = NULL;
	UWORD16  trout_rsp_len = 0;
	int len = 0;

	PRINTK_ITMIW("itm_sforwifi_direct fuc \n");

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if( host_req == NULL)
	{
		return -ENOMEM;
	}

	memcpy(cmd,&extra_data[tmp_index + 1],extra_data[tmp_index]);
	cmd[extra_data[tmp_index]] = '\0';
	tmp_index += 1 + extra_data[tmp_index];

	if(!strcasecmp(cmd, "on"))
	{
		PRINTK_ITMIW("itm_sforwifi_direct fuc  on \n");
	
		host_req[len+0] = WID_P2P_ENABLE  & 0xFF;
		host_req[len+1] = (WID_P2P_ENABLE  & 0xFF00) >> 8;
		host_req[len+2] = 1;
		host_req[len+3] = 1;
		len += WID_CHAR_CFG_LEN;

		//host_req[len+0] = WID_P2P_GO_INT_VAL & 0xFF;
		//host_req[len+1] = (WID_P2P_GO_INT_VAL & 0xFF00) >> 8;
		//host_req[len+2] = 1;
		//host_req[len+3] = 0x0F;
		//len += WID_CHAR_CFG_LEN;

		config_if_for_iw( &g_mac,host_req,len,'W',&trout_rsp_len );

		if( trout_rsp_len != 1 )
		{
			ret = -EINVAL;
		}

		//kfree(host_req);
		return ret;
	}
	else
	if(!strcasecmp(cmd, "off"))
	{
		PRINTK_ITMIW("itm_sforwifi_direct fuc off \n");

		//wxb add
		host_req[len+0] = WID_P2P_ENABLE  & 0xFF;
		host_req[len+1] = (WID_P2P_ENABLE  & 0xFF00) >> 8;
		host_req[len+2] = 1;
		host_req[len+3] = 0;
		len += WID_CHAR_CFG_LEN;

		config_if_for_iw( &g_mac,host_req,len,'W',&trout_rsp_len );

		if( trout_rsp_len != 1 )
		{
			ret = -EINVAL;
		}

		//kfree(host_req);
		return ret;
	}

	//if(!strcasecmp(cmd, "type"))
	//{
		//PRINTK_ITMIW("itm_sforwifi_direct fuc type \n");
		//memcpy(cmd,&extra_data[tmp_index + 1],extra_data[tmp_index]);
		//cmd[extra_data[tmp_index]] = '\0';
		//tmp_index += 1 + extra_data[tmp_index];

		if(!strcasecmp(cmd, "pbc"))
		{
			PRINTK_ITMIW("itm_sforwifi_direct fuc pbc \n");
		
			host_req[len+0] = WID_WPS_PASS_ID  & 0xFF;
			host_req[len+1] = (WID_WPS_PASS_ID  & 0xFF00) >> 8;
			host_req[len+2] = 2;
			host_req[len+3] = 0x04;
			host_req[len+4] = 0x00;
			len += WID_SHORT_CFG_LEN;

			host_req[len+0] = WID_WPS_START & 0xFF;
			host_req[len+1] = (WID_WPS_START & 0xFF00) >> 8;
			host_req[len+2] = 1;
			host_req[len+3] = 0x02;
			len += WID_CHAR_CFG_LEN;

			config_if_for_iw( &g_mac,host_req,len,'W',&trout_rsp_len );

			if( trout_rsp_len != 1 )
			{
				ret = -EINVAL;
			}

			//kfree(host_req);
			return ret;
		}
		else
		if(!strcasecmp(cmd, "pin"))
		{
			int pin_data_i = 0;
			int pin_data_len = (int)extra_data[tmp_index];

			PRINTK_ITMIW("itm_sforwifi_direct fuc pin \n");
			
			for(pin_data_i = 0 ; pin_data_i < pin_data_len ; pin_data_i++)
			{
				cmd[pin_data_i*2]   = hex_2_char((UWORD8)( extra_data[tmp_index + 1 + pin_data_i] >> 4));
				cmd[pin_data_i*2+1] = hex_2_char((UWORD8)( extra_data[tmp_index + 1 + pin_data_i] & 0x0F));
			}

			//memcpy(cmd,&extra_data[tmp_index + 1],extra_data[tmp_index]);
			//cmd[extra_data[tmp_index]] = '\0';
			tmp_index += 1 + extra_data[tmp_index];

			host_req[len+0] = WID_WPS_PIN  & 0xFF;
			host_req[len+1] = (WID_WPS_PIN  & 0xFF00) >> 8;
			host_req[len+2] = pin_data_len*2;
			memcpy(&host_req[len+3],cmd,pin_data_len*2);
			len += WID_HEAD_LEN + pin_data_len*2;

			host_req[len+0] = WID_WPS_PASS_ID & 0xFF;
			host_req[len+1] = (WID_WPS_PASS_ID & 0xFF00) >> 8;
			host_req[len+2] = 2;
			host_req[len+3] = 0x00;
			host_req[len+4] = 0x00;
			len += WID_SHORT_CFG_LEN;

			host_req[len+0] = WID_WPS_START & 0xFF;
			host_req[len+1] = (WID_WPS_START & 0xFF00) >> 8;
			host_req[len+2] = 1;
			host_req[len+3] = 0x01;
			len += WID_CHAR_CFG_LEN;

			config_if_for_iw( &g_mac,host_req,len,'W',&trout_rsp_len );

			if( trout_rsp_len != 1 )
			{
				ret = -EINVAL;
			}

			//kfree(host_req);
			return ret;
			
		}
	//}
	else
	if(!strcasecmp(cmd, "daddr"))
	{
		PRINTK_ITMIW("itm_sforwifi_direct fuc daddr \n");
		
		memcpy(cmd,&extra_data[tmp_index + 1],extra_data[tmp_index]);
		cmd[extra_data[tmp_index]] = '\0';
		tmp_index += 1 + extra_data[tmp_index];

		host_req[len+0] = WID_P2P_TARGET_DEV_ID  & 0xFF;
		host_req[len+1] = (WID_P2P_TARGET_DEV_ID  & 0xFF00) >> 8;
		host_req[len+2] = 6;
		memcpy(&host_req[len+3],cmd,6);
		len += WID_HEAD_LEN + 6;

		config_if_for_iw( &g_mac,host_req,len,'W',&trout_rsp_len );

		if( trout_rsp_len != 1 )
		{
			ret = -EINVAL;
		}

		//kfree(host_req);
		return ret;
	}
	else if (!strcasecmp(cmd, "go_int")) //wxb add
       {
		PRINTK_ITMIW("itm_sforwifi_direct fuc go_int \n");

		memcpy(cmd,&extra_data[tmp_index + 1],extra_data[tmp_index]);
		cmd[extra_data[tmp_index]] = '\0';
		tmp_index += 1 + extra_data[tmp_index];
		UWORD8 value = (UWORD8)simple_strtol(cmd, NULL, 10);

		host_req[len+0] = WID_P2P_GO_INT_VAL & 0xFF;
		host_req[len+1] = (WID_P2P_GO_INT_VAL & 0xFF00) >> 8;
		host_req[len+2] = 1;
		host_req[len+3] = value;
		len += WID_CHAR_CFG_LEN;

		config_if_for_iw( &g_mac,host_req,len,'W',&trout_rsp_len );

		if( trout_rsp_len != 1 )
		{
			ret = -EINVAL;
		}
		//kfree(host_req);
		return ret;
	}


	return 0;	
}

static int
itm_gwdirect(struct net_device *dev, struct iw_request_info *info,
        			union iwreq_data *wrqu, char *extra)
{
	struct iw_point * tmp_extra = (struct iw_point *)wrqu;
	UWORD8 * tmp_data = (UWORD8 * )extra;
	int ret = 0;
	UWORD8 * host_req  = NULL;
	UWORD8 * trout_rsp = NULL;
	UWORD16  trout_rsp_len = 0;
	UWORD8   wifi_direct_enable = 0;
	UWORD8   wifi_direct_go_int_value = 0;
	UWORD16  wifi_direct_type = 0;
	UWORD8 wifi_direct_data[64] = {0};
	int tmp_index = 1;
	int tmp_len = 0;
	int i = 0;
	int j = 0;

	PRINTK_ITMIW("itm_gforwifi_direct fuc \n");

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if( host_req == NULL)
	{
		return -ENOMEM;
	}


	/*************** get wifi direct enable ****************/
	host_req[0] = WID_P2P_ENABLE & 0xFF;
	host_req[1] = (WID_P2P_ENABLE & 0xFF00) >> 8;

	trout_rsp = config_if_for_iw(&g_mac,host_req,2,'Q',&trout_rsp_len);

	if( trout_rsp == NULL )
	{
		ret = -1;
		goto out1;
	}

	trout_rsp_len -= MSG_HDR_LEN;
	if( get_trour_rsp_data(&wifi_direct_enable,(UWORD16)sizeof(UWORD8),
					       &trout_rsp[MSG_HDR_LEN],trout_rsp_len,
					       WID_CHAR) == 0 )
	{
		ret = -1;
		goto out2;
	}

	tmp_data[0] = 0;
	if(wifi_direct_enable)
	{
		tmp_data[tmp_index] = strlen("wifi-direct on");
		sprintf(&tmp_data[tmp_index + 1],"wifi-direct on");
		tmp_index += 1 + tmp_data[tmp_index];
		
		//tmp_extra->flags  += 1;
		tmp_extra->length = tmp_index;
		tmp_data[0]  += 1;
	}
	else
	{
		tmp_data[tmp_index] = strlen("wifi-direct off");
		sprintf(&tmp_data[tmp_index + 1],"wifi-direct off");
		tmp_index += 1 + tmp_data[tmp_index];
		
		//tmp_extra->flags  += 1;
		tmp_extra->length = tmp_index;
		tmp_data[0]  += 1;

		goto out2;
	}

	pkt_mem_free(trout_rsp);
	/*************** get wifi direct GO int ****************/
	host_req[0] = WID_P2P_GO_INT_VAL & 0xFF;
	host_req[1] = (WID_P2P_GO_INT_VAL & 0xFF00) >> 8;

	trout_rsp = config_if_for_iw(&g_mac,host_req,2,'Q',&trout_rsp_len);

	if( trout_rsp == NULL )
	{
		ret = -1;
		goto out1;
	}

	trout_rsp_len -= MSG_HDR_LEN;
	if( get_trour_rsp_data(&wifi_direct_go_int_value,(UWORD16)sizeof(UWORD8),
						&trout_rsp[MSG_HDR_LEN],trout_rsp_len,
						WID_CHAR) == 0 )
	{
		ret = -1;
		goto out2;
	}

	tmp_data[tmp_index] = strlen("wifi-direct go_int value ") + 1;
	sprintf(&tmp_data[tmp_index + 1],"wifi-direct go_int value %d", wifi_direct_go_int_value);
	tmp_index += 1 + tmp_data[tmp_index];

	//tmp_extra->flags  += 1;
	tmp_extra->length = tmp_index;
	tmp_data[0]  += 1;

	pkt_mem_free(trout_rsp);
	/*************** get wifi direct addr ****************/
	host_req[0] = WID_P2P_TARGET_DEV_ID & 0xFF;
	host_req[1] = (WID_P2P_TARGET_DEV_ID & 0xFF00) >> 8;

	trout_rsp = config_if_for_iw(&g_mac,host_req,2,'Q',&trout_rsp_len);

	if( trout_rsp == NULL )
	{
		ret = -1;
		goto out1;
	}

	trout_rsp_len -= MSG_HDR_LEN;
	if( get_trour_rsp_data(wifi_direct_data,(UWORD16)sizeof(wifi_direct_data),
					       &trout_rsp[MSG_HDR_LEN],trout_rsp_len,
					       WID_STR) == 0 )
	{
		ret = -1;
		goto out2;
	}

	tmp_data[tmp_index] = strlen("wifi-direct daddr ") + 6*2 + 5;
	sprintf(&tmp_data[tmp_index + 1],"wifi-direct daddr %02x:%02x:%02x:%02x:%02x:%02x",
							wifi_direct_data[0],wifi_direct_data[1],wifi_direct_data[2],
							wifi_direct_data[3],wifi_direct_data[4],wifi_direct_data[5]);
	tmp_index += 1 + tmp_data[tmp_index];
	
	//tmp_extra->flags  += 1;
	tmp_extra->length = tmp_index;
	tmp_data[0]  += 1;


	pkt_mem_free(trout_rsp);
	/*************** get wifi direct type ****************/
	host_req[0] = WID_WPS_PASS_ID & 0xFF;
	host_req[1] = (WID_WPS_PASS_ID & 0xFF00) >> 8;

	trout_rsp = config_if_for_iw(&g_mac,host_req,2,'Q',&trout_rsp_len);

	if( trout_rsp == NULL )
	{
		ret = -1;
		goto out1;
	}

	trout_rsp_len -= MSG_HDR_LEN;
	if( get_trour_rsp_data((UWORD8 *)(&wifi_direct_type),(UWORD16)sizeof(wifi_direct_type),
					       &trout_rsp[MSG_HDR_LEN],trout_rsp_len,
					       WID_SHORT) == 0 )
	{
		ret = -1;
		goto out2;
	}
	
	if(wifi_direct_type)//pbc
	{
		tmp_data[tmp_index] = strlen("wifi-direct type pbc");
		sprintf(&tmp_data[tmp_index + 1],"wifi-direct type pbc");
		tmp_index += 1 + tmp_data[tmp_index];
		
		//tmp_extra->flags  += 1;
		tmp_extra->length = tmp_index;
		tmp_data[0]  += 1;

		goto out2;
	}
	else//pin
	{
		tmp_data[tmp_index] = strlen("wifi-direct type pin");
		sprintf(&tmp_data[tmp_index + 1],"wifi-direct type pin");
		tmp_index += 1 + tmp_data[tmp_index];
		
		//tmp_extra->flags  += 1;
		tmp_extra->length = tmp_index;
		tmp_data[0]  += 1;

	}

	pkt_mem_free(trout_rsp);
	/*************** get wifi direct pin data ****************/
	host_req[0] = WID_WPS_PIN & 0xFF;
	host_req[1] = (WID_WPS_PIN & 0xFF00) >> 8;

	trout_rsp = config_if_for_iw(&g_mac,host_req,2,'Q',&trout_rsp_len);

	if( trout_rsp == NULL )
	{
		ret = -1;
		goto out1;
	}

	trout_rsp_len -= MSG_HDR_LEN;
	if( (tmp_len = get_trour_rsp_data(wifi_direct_data,(UWORD16)sizeof(wifi_direct_data),
					       &trout_rsp[MSG_HDR_LEN],trout_rsp_len,
					       WID_STR)) < 0 )
	{
		ret = -1;
		goto out2;
	}

	j = strlen("wifi-direct pin data ");
	tmp_data[tmp_index] = j + tmp_len/2;
	sprintf(&tmp_data[tmp_index+1],"wifi-direct pin data ");
	
	tmp_index += 1 + j;
	for( i = 0 ; i < tmp_len/2 ; i++)
	{
		str_2_hex(&wifi_direct_data[i*2],2);
		tmp_data[tmp_index + i] = wifi_direct_data[i*2];
	}
	tmp_index += tmp_len/2;
	
	//tmp_extra->flags  += 1;
	tmp_extra->length = tmp_index;
	tmp_data[0]  += 1;

	out2:
	pkt_mem_free(trout_rsp);
	out1:
	//kfree(host_req);
	return ret;
}


static int
itm_sampdu(struct net_device *dev, struct iw_request_info *info,
        			union iwreq_data *wrqu, char *extra)
{
	//struct iw_point * tmp_extra = (struct iw_point *)wrqu;
	int tmp_data = *((int *)extra);

	PRINTK_ITMIW("itm_sfora_mpdu fuc \n");
	PRINTK_ITMIW("%d\n",tmp_data);

	tmp_data = (tmp_data) ? 1 : 0;

	//config_a_mpdu_set(tmp_data);

	return 0;
}


static int
itm_gampdu(struct net_device *dev, struct iw_request_info *info,
        			union iwreq_data *wrqu, char *extra)
{
	int tmp_data = 0;

	PRINTK_ITMIW("itm_gfora_mpdu fuc \n");
	PRINTK_ITMIW("%d\n",tmp_data);

	//tmp_data = config_a_mpdu_get();

	*((int *)extra) = tmp_data;
	return 0;
}


static int
itm_samsdu(struct net_device *dev, struct iw_request_info *info,
        			union iwreq_data *wrqu, char *extra)
{
	int tmp_data = *((int *)extra);

	PRINTK_ITMIW("itm_sfora_msdu fuc \n");

	PRINTK_ITMIW("%d\n",tmp_data);

	tmp_data = (tmp_data) ? 1 : 0;

	//config_a_msdu_set(tmp_data);

	return 0;
}


static int
itm_gamsdu(struct net_device *dev, struct iw_request_info *info,
        			union iwreq_data *wrqu, char *extra)
{
	int tmp_data = 0;

	PRINTK_ITMIW("itm_gfora_msdu fuc \n");
	
	//tmp_data = config_a_msdu_get();

	*((int *)extra) = tmp_data;
	return 0;
}


static int
itm_sblockack(struct net_device *dev, struct iw_request_info *info,
        			union iwreq_data *wrqu, char *extra)
{
	int tmp_data = *((int *)extra);

	PRINTK_ITMIW("itm_sforblock_ack fuc \n");
	PRINTK_ITMIW("%d\n",tmp_data);

	tmp_data = (tmp_data) ? 1 : 0;

	//config_block_ack_set(tmp_data);

	return 0;
}


static int
itm_gblockack(struct net_device *dev, struct iw_request_info *info,
        			union iwreq_data *wrqu, char *extra)
{
	int tmp_data = 0;

	PRINTK_ITMIW("itm_gforblock_ack fuc \n");


	//tmp_data = config_block_ack_get();

	*((int *)extra) = tmp_data;
	return 0;
}


static int
itm_sshortgi(struct net_device *dev, struct iw_request_info *info,
        			union iwreq_data *wrqu, char *extra)
{
	int tmp_data = *((int *)extra);

	int ret = 0;
	UWORD8 * host_req  = NULL;
	UWORD16  trout_rsp_len = 0;

	PRINTK_ITMIW("itm_sforshort_gi fuc \n");
	PRINTK_ITMIW("%d\n",tmp_data);
	

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if(host_req == NULL)
	{
		return -ENOMEM;
	}

	host_req[0] = WID_11N_SHORT_GI_ENABLE & 0xFF;
	host_req[1] = (WID_11N_SHORT_GI_ENABLE & 0xFF00) >> 8;
	host_req[2] = 1;
	host_req[3] = tmp_data;

	config_if_for_iw(&g_mac,host_req,WID_CHAR_CFG_LEN,'W',&trout_rsp_len);

	if( trout_rsp_len != 1 )
	{
		ret = -EINVAL;
		goto out1;
	}
	   
	out1:
	//kfree(host_req);
	return ret;
}


static int
itm_gshortgi(struct net_device *dev, struct iw_request_info *info,
        			union iwreq_data *wrqu, char *extra)
{
	int tmp_data = 0;
	
	int ret = 0;
	UWORD8 * host_req  = NULL;
	UWORD8 * trout_rsp = NULL;
	UWORD16  trout_rsp_len = 0;
	UWORD8 gi_enable = 0;

	PRINTK_ITMIW("itm_gforshort_gi fuc\n");

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if(host_req == NULL)
	{
		return -ENOMEM;
	}

	host_req[0] = WID_11N_SHORT_GI_ENABLE & 0xFF;
	host_req[1] = (WID_11N_SHORT_GI_ENABLE & 0xFF00) >> 8;

	trout_rsp = config_if_for_iw(&g_mac,host_req,2,'Q',&trout_rsp_len);

	if( trout_rsp == NULL )
	{
		ret = -EINVAL;
		goto out1;
	}

	trout_rsp_len -= MSG_HDR_LEN;
	if(get_trour_rsp_data(&gi_enable,sizeof(gi_enable),&trout_rsp[MSG_HDR_LEN],
					      trout_rsp_len,WID_CHAR)  ==  0)
	{
		ret = -EINVAL;
		goto out2;
	}
	
	tmp_data = gi_enable;
	*((int *)extra) = tmp_data;

	out2:
	pkt_mem_free(trout_rsp);
	out1:
	//kfree(host_req);
	return ret;

}


static int
itm_smu_domain(struct net_device *dev, struct iw_request_info *info,
        			union iwreq_data *wrqu, char *extra)
{
	int tmp_data = *((int *)extra);

	PRINTK_ITMIW("itm_sformuti_domain fuc \n");
	PRINTK_ITMIW("%d\n",tmp_data);
	

	return itm_set_domain(tmp_data, 11);

}


static int
itm_gmu_domain(struct net_device *dev, struct iw_request_info *info,
        			union iwreq_data *wrqu, char *extra)
{
	int tmp_data = 0;
	
	int ret = 0;
	UWORD8 * host_req  = NULL;
	UWORD8 * trout_rsp = NULL;
	UWORD16  trout_rsp_len = 0;
	UWORD8 muti_domain = 0;

	PRINTK_ITMIW("itm_gformuti_domain fuc\n");

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if(host_req == NULL)
	{
		return -ENOMEM;
	}

	host_req[0] = WID_ENABLE_MULTI_DOMAIN & 0xFF;
	host_req[1] = (WID_ENABLE_MULTI_DOMAIN & 0xFF00) >> 8;

	trout_rsp = config_if_for_iw(&g_mac,host_req,2,'Q',&trout_rsp_len);

	if( trout_rsp == NULL )
	{
		ret = -EINVAL;
		goto out1;
	}

	trout_rsp_len -= MSG_HDR_LEN;
	if(get_trour_rsp_data(&muti_domain,sizeof(muti_domain),&trout_rsp[MSG_HDR_LEN],
					      trout_rsp_len,WID_CHAR)  ==  0)
	{
		ret = -EINVAL;
		goto out2;
	}
	
	tmp_data = muti_domain;
	*((int *)extra) = tmp_data;

	out2:
	pkt_mem_free(trout_rsp);
	out1:
	//kfree(host_req);
	return ret;

}

static int
itm_swps(struct net_device *dev, struct iw_request_info *info,
        			union iwreq_data *wrqu, char *extra)
{
	UWORD8 * extra_data = (UWORD8 * )extra;
	UWORD8 cmd[64] = {0};
	UWORD8 val[64] = {0};
	int tmp_index = 0;

	int ret = 0;
	UWORD8 * host_req  = NULL;
	UWORD16  trout_rsp_len = 0;
	int len = 0;

	PRINTK_ITMIW("itm_swps fuc \n");

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if( host_req == NULL)
	{
		return -ENOMEM;
	}

	memcpy(cmd,&extra_data[tmp_index + 1],extra_data[tmp_index]);
	cmd[extra_data[tmp_index]] = '\0';
	tmp_index += 1 + extra_data[tmp_index];

	memcpy(val,&extra_data[tmp_index + 1],extra_data[tmp_index]);
	val[extra_data[tmp_index]] = '\0';
	tmp_index += 1 + extra_data[tmp_index];

	if(!strcasecmp(cmd, "pbc"))
	{
		PRINTK_ITMIW("itm_swps fuc pbc \n");

		if(!strcasecmp(val, "set"))
		{
			PRINTK_ITMIW("itm_swps fuc set \n");
		
			host_req[len+0] = WID_WPS_ENABLE & 0xFF;
			host_req[len+1] = (WID_WPS_ENABLE & 0xFF00) >> 8;
			host_req[len+2] = 1;
			host_req[len+3] = 0x01;
			len += WID_CHAR_CFG_LEN;

			host_req[len+0] = WID_WPS_DEV_MODE & 0xFF;
			host_req[len+1] = (WID_WPS_DEV_MODE & 0xFF00) >> 8;
			host_req[len+2] = 1;
			host_req[len+3] = 0x00;
			len += WID_CHAR_CFG_LEN;

			host_req[len+0] = WID_WPS_PASS_ID  & 0xFF;
			host_req[len+1] = (WID_WPS_PASS_ID  & 0xFF00) >> 8;
			host_req[len+2] = 2;
			host_req[len+3] = 0x04;
			host_req[len+4] = 0x00;
			len += WID_SHORT_CFG_LEN;				
		}
		else
		if(!strcasecmp(val, "start"))
		{ 
			PRINTK_ITMIW("itm_swps fuc start \n");
		
			host_req[len+0] = WID_WPS_START & 0xFF;
			host_req[len+1] = (WID_WPS_START & 0xFF00) >> 8;
			host_req[len+2] = 1;
			host_req[len+3] = 0x02;
			len += WID_CHAR_CFG_LEN;
		}
		else
		{
			return -EINVAL;
		}

		config_if_for_iw( &g_mac,host_req,len,'W',&trout_rsp_len );

		if( trout_rsp_len != 1 )
		{
			ret = -EINVAL;
		}

		//kfree(host_req);
		return ret;
	}
	else
	if(!strcasecmp(cmd, "pin"))
	{
		PRINTK_ITMIW("itm_swps fuc pin \n");	
	
		if(!strcasecmp(val, "start"))
		{
			PRINTK_ITMIW("itm_swps fuc start \n");
		
			host_req[len+0] = WID_WPS_START & 0xFF;
			host_req[len+1] = (WID_WPS_START & 0xFF00) >> 8;
			host_req[len+2] = 1;
			host_req[len+3] = 0x01;
			len += WID_CHAR_CFG_LEN;
		}
		else
		{
			int pin_data_i = 0;
			int pin_data_len = strlen(val);
			UWORD8 tmp_data[64] = {0};

			PRINTK_ITMIW("itm_swps fuc set \n");
			
			for(pin_data_i = 0 ; pin_data_i < pin_data_len ; pin_data_i++)
			{
				tmp_data[pin_data_i*2]   = hex_2_char((UWORD8)( val[pin_data_i] >> 4));
				tmp_data[pin_data_i*2+1] = hex_2_char((UWORD8)( val[pin_data_i] & 0x0F));
			}

			host_req[len+0] = WID_WPS_ENABLE & 0xFF;
			host_req[len+1] = (WID_WPS_ENABLE & 0xFF00) >> 8;
			host_req[len+2] = 1;
			host_req[len+3] = 0x01;
			len += WID_CHAR_CFG_LEN;

			host_req[len+0] = WID_WPS_DEV_MODE & 0xFF;
			host_req[len+1] = (WID_WPS_DEV_MODE & 0xFF00) >> 8;
			host_req[len+2] = 1;
			host_req[len+3] = 0x00;
			len += WID_CHAR_CFG_LEN;

			host_req[len+0] = WID_WPS_PIN  & 0xFF;
			host_req[len+1] = (WID_WPS_PIN  & 0xFF00) >> 8;
			host_req[len+2] = pin_data_len*2;
			memcpy(&host_req[len+3],tmp_data,pin_data_len*2);
			len += WID_HEAD_LEN + pin_data_len*2;

			host_req[len+0] = WID_WPS_PASS_ID & 0xFF;
			host_req[len+1] = (WID_WPS_PASS_ID & 0xFF00) >> 8;
			host_req[len+2] = 2;
			host_req[len+3] = 0x00;
			host_req[len+4] = 0x00;
			len += WID_SHORT_CFG_LEN;
		}
		
		config_if_for_iw( &g_mac,host_req,len,'W',&trout_rsp_len );

		if( trout_rsp_len != 1 )
		{
			ret = -EINVAL;
		}

		//kfree(host_req);
		return ret;
		
	}
	else	
	{
		return -EINVAL;
	}	
}   
#if 0
{
	pbc
	00 41 -> 01 -> 01    WID_WPS_ENABLE
	00 44 -> 01 -> 00    WID_WPS_DEV_MODE
	10 17 -> 02 -> 00 04 WID_WPS_PASS_ID

	00 43 -> 01 -> 02    WID_WPS_START

	pin
	00 41 -> 01 -> 01	 WID_WPS_ENABLE
	00 44 -> 01 -> 00    WID_WPS_DEV_MODE
	30 25 -> 08 -> 39 36 31 39 36 31 39 36 WID_WPS_PIN
	10 17 -> 02 -> 00 00 WID_WPS_PASS_ID

	00 43 -> 01 -> 01    WID_WPS_START

}
#endif
static int
itm_gwps(struct net_device *dev, struct iw_request_info *info,
        			union iwreq_data *wrqu, char *extra)
{
	struct iw_point * tmp_extra = (struct iw_point *)wrqu;
	UWORD8 * tmp_data = (UWORD8 * )extra;
	int ret = 0;
	UWORD8 * host_req  = NULL;
	UWORD8 * trout_rsp = NULL;
	UWORD16  trout_rsp_len = 0;
	UWORD8   wps_enable = 0;
	UWORD8   wps_device_mode = 0;
	UWORD16  wps_type = 0;
	UWORD8   wps_pin_data[64] = {0};
	int tmp_index = 1;
	int tmp_len = 0;
	int i = 0;
	int j = 0;

	PRINTK_ITMIW("itm_gwps fuc \n");

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if( host_req == NULL)
	{
		return -ENOMEM;
	}


	/*************** get wps enable ****************/
	host_req[0] = WID_WPS_ENABLE & 0xFF;
	host_req[1] = (WID_WPS_ENABLE & 0xFF00) >> 8;

	trout_rsp = config_if_for_iw(&g_mac,host_req,2,'Q',&trout_rsp_len);

	if( trout_rsp == NULL )
	{
		ret = -1;
		goto out1;
	}

	trout_rsp_len -= MSG_HDR_LEN;
	if( get_trour_rsp_data(&wps_enable,(UWORD16)sizeof(UWORD8),
					       &trout_rsp[MSG_HDR_LEN],trout_rsp_len,
					       WID_CHAR) == 0 )
	{
		ret = -1;
		goto out2;
	}

	tmp_data[0] = 0;
	if(wps_enable)
	{
		tmp_data[tmp_index] = strlen("wps on");
		sprintf(&tmp_data[tmp_index + 1],"wps on");
		tmp_index += 1 + tmp_data[tmp_index];
		
		//tmp_extra->flags  += 1;
		tmp_extra->length = tmp_index;
		tmp_data[0]  += 1;
	}
	else
	{
		tmp_data[tmp_index] = strlen("wps off");
		sprintf(&tmp_data[tmp_index + 1],"wps off");
		tmp_index += 1 + tmp_data[tmp_index];
		
		//tmp_extra->flags  += 1;
		tmp_extra->length = tmp_index;
		tmp_data[0]  += 1;

		goto out2;
	}

	pkt_mem_free(trout_rsp);
	/*************** get wps device mode ****************/
	host_req[0] = WID_WPS_DEV_MODE & 0xFF;
	host_req[1] = (WID_WPS_DEV_MODE & 0xFF00) >> 8;

	trout_rsp = config_if_for_iw(&g_mac,host_req,2,'Q',&trout_rsp_len);

	if( trout_rsp == NULL )
	{
		ret = -1;
		goto out1;
	}

	trout_rsp_len -= MSG_HDR_LEN;
	if( get_trour_rsp_data(&wps_device_mode,(UWORD16)sizeof(wps_device_mode),
					       &trout_rsp[MSG_HDR_LEN],trout_rsp_len,
					       WID_CHAR) == 0 )
	{
		ret = -1;
		goto out2;
	}

	if(wps_device_mode)
	{
		tmp_data[tmp_index] = strlen("wps device mode ") + strlen("host_managed");

		sprintf(&tmp_data[tmp_index + 1],"wps device mode host_managed");
	}
	else
	{
		tmp_data[tmp_index] = strlen("wps device mode ") + strlen("standalone");

		sprintf(&tmp_data[tmp_index + 1],"wps device mode standalone");
	}
	
	tmp_index += 1 + tmp_data[tmp_index];
	
	//tmp_extra->flags  += 1;
	tmp_extra->length = tmp_index;
	tmp_data[0]  += 1;


	pkt_mem_free(trout_rsp);
	/*************** get wps type ****************/
	host_req[0] = WID_WPS_PASS_ID & 0xFF;
	host_req[1] = (WID_WPS_PASS_ID & 0xFF00) >> 8;

	trout_rsp = config_if_for_iw(&g_mac,host_req,2,'Q',&trout_rsp_len);

	if( trout_rsp == NULL )
	{
		ret = -1;
		goto out1;
	}

	trout_rsp_len -= MSG_HDR_LEN;
	if( get_trour_rsp_data((UWORD8 *)(&wps_type),(UWORD16)sizeof(wps_type),
					       &trout_rsp[MSG_HDR_LEN],trout_rsp_len,
					       WID_SHORT) == 0 )
	{
		ret = -1;
		goto out2;
	}
	
	if(wps_type)//pbc
	{
		tmp_data[tmp_index] = strlen("wps type pbc");
		sprintf(&tmp_data[tmp_index + 1],"wps type pbc");
		tmp_index += 1 + tmp_data[tmp_index];
		
		//tmp_extra->flags  += 1;
		tmp_extra->length = tmp_index;
		tmp_data[0]  += 1;

		goto out2;
	}
	else//pin
	{
		tmp_data[tmp_index] = strlen("wps type pin");
		sprintf(&tmp_data[tmp_index + 1],"wps type pin");
		tmp_index += 1 + tmp_data[tmp_index];
		
		//tmp_extra->flags  += 1;
		tmp_extra->length = tmp_index;
		tmp_data[0]  += 1;
	}

	pkt_mem_free(trout_rsp);
	/*************** get wps pin data ****************/
	host_req[0] = WID_WPS_PIN & 0xFF;
	host_req[1] = (WID_WPS_PIN & 0xFF00) >> 8;

	trout_rsp = config_if_for_iw(&g_mac,host_req,2,'Q',&trout_rsp_len);

	if( trout_rsp == NULL )
	{
		ret = -1;
		goto out1;
	}

	trout_rsp_len -= MSG_HDR_LEN;
	if( (tmp_len = get_trour_rsp_data(wps_pin_data,(UWORD16)sizeof(wps_pin_data),
					       &trout_rsp[MSG_HDR_LEN],trout_rsp_len,
					       WID_STR)) < 0 )
	{
		ret = -1;
		goto out2;
	}

	j = strlen("wps pin data ");
	tmp_data[tmp_index] = j + tmp_len/2;
	sprintf(&tmp_data[tmp_index+1],"wps pin data ");
	
	tmp_index += 1 + j;
	for( i = 0 ; i < tmp_len/2 ; i++)
	{
		str_2_hex(&wps_pin_data[i*2],2);
		tmp_data[tmp_index + i] = wps_pin_data[i*2];
	}
	tmp_index += tmp_len/2;
	
	//tmp_extra->flags  += 1;
	tmp_extra->length = tmp_index;
	tmp_data[0]  += 1;

	out2:
	pkt_mem_free(trout_rsp);
	out1:
	//kfree(host_req);
	return ret;
}

#ifdef TROUT_WIFI_NPI
#include <linux/workqueue.h>

//zhuyg add start
//UWORD32 tx_power_level = 0;
volatile int g_tx_flag = 0;
UWORD32 g_rx_right_start_count = 0;
UWORD32 g_rx_error_start_count = 0;
int g_rx_flag = 0;
UWORD8 g_mac_addr[6] = {0};
UWORD8 g_user_tx_rate = 0;
UWORD32 g_save_reg1 = 0;
UWORD32 g_save_reg2 = 0;
UWORD32 g_sys_reg = 0;
int g_npi_scan_flag = 0;
int g_npi_tx_pkt_count = 0;
extern int copy_q_num;
extern int transfer_q_num;

int trout_npi_send_pkt_from_host(int pkt_len,int num, int q_num);
void qmu_cpy_npi_descr(int q_num);
void tx_packet_test(void)
{
	UWORD32 reg_data = 0;
	//int send_flag = 0;
	copy_q_num = -1;
       transfer_q_num = -1;

       //yangke, 2013-10-16, for NPI tx_start use, only call once at the beginning of NPI tx_start function!!
	if (g_tx_flag == 1 /*&& send_flag==0*/)
	{
		reg_data = host_read_trout_reg((UWORD32)rMAC_TSF_CON);
		if ((reg_data & BIT0) == 0)
		{
		    //printk("npi: rMAC_TSF_CON are disable, open it. \n");
			reg_data = reg_data | BIT0;
			host_write_trout_reg(reg_data, (UWORD32)rMAC_TSF_CON);
		}
		//trout_b2b_pkt_from_host(2300);
		//yangke, attention the procedure, first normal queue, next is high queue
		trout_npi_send_pkt_from_host(1500, NPI_TX_PKT_NUM, NORMAL_PRI_Q);
              add_calib_delay(5000); //delay 50ms make sure the first tx finished
              trout_npi_send_pkt_from_host(1500, NPI_TX_PKT_NUM, HIGH_PRI_Q);
        g_npi_tx_pkt_count++;
		//send_flag = 1;
	    //printk("npi: already send one pkt, all = %d. \n", g_npi_tx_pkt_count);
	}
       #if 0
	if(send_flag == 1){
              printk("ERROR: %s: send_flag is set, already init trout description\n", __func__);
              return;
              //refill description in tx complete interrupt
              //qmu_cpy_npi_descr();
	}
       #endif
}

static void set_hardware()
{
	set_machw_lrl(10);
	set_machw_srl(10);
	set_machw_cw_bk(2,2);
	set_machw_cw_be(2,2);
	set_machw_cw_vi(2,2);
	set_machw_cw_vo(2,2);
}

static int itm_tx_data(struct net_device *dev, struct iw_request_info *info,
                                        union iwreq_data *wrqu, char *extra)
{
	int ret = -1;
	int i = 0;
	UWORD8 * extra_data = (UWORD8 * )extra;
	UWORD8 cmd[64] = {0};
	int tmp_index = 0;
	int len = 0;
	static int flag = 1;
	UWORD32 write_sys_value = 0;

    set_hardware();
	static UWORD32  wifi_ram_test[]=
	{
		0x00500050,0x009e009e,0x00e800e8,0x012c012c,
		0x01690169,0x019d019d,0x01c701c7,0x01e601e6,
		0x01f901f9,0x01ff01ff,0x01f901f9,0x01e601e6,
		0x01c701c7,0x019d019d,0x01690169,0x012c012c,
		0x00e800e8,0x009e009e,0x00500050,0x00000000,
		0x00500050,0x009e009e,0x00e800e8,0x012c012c,
		0x01690169,0x019d019d,0x01c701c7,0x01e601e6,
		0x01f901f9,0x01ff01ff,0x01f901f9,0x01e601e6,
		0x01c701c7,0x019d019d,0x01690169,0x012c012c,
		0x00e800e8,0x009e009e,0x00500050,0x00000000
	};

	memcpy(cmd,&extra_data[tmp_index + 1],extra_data[tmp_index]);
	cmd[extra_data[tmp_index]] = '\0';
	tmp_index += 1 + extra_data[tmp_index];

	printk("zhuyg: enter itm_tx_data\n");
	g_npi_scan_flag = 0;
    printk("%s-%d, g_npi_scan_flag = %d\n",__FUNCTION__,__LINE__,g_npi_scan_flag);
	if(!strcasecmp(cmd, "start"))
	{
		restart_mac(&g_mac, 0);
		if (flag == 1)
		{
			g_save_reg1 = host_read_trout_reg((0x4072 <<2));//dft wave len 
			g_save_reg2 = host_read_trout_reg((0x4071 <<2));
			g_sys_reg = host_read_trout_reg((UWORD32)rSYSREG_WIFI_CFG);
			printk("g_save_reg1 :%u\n", g_save_reg1);
			printk("g_save_reg2 :%u\n", g_save_reg2);
			flag = 2;
		}
		if (extra[tmp_index+1] == '1')
		{
			write_sys_value = g_sys_reg | 1 | (1 << 7) | (1 << 8);
			printk("write value is: %x\n", write_sys_value);
			host_write_trout_ram((void*)CW_TX_BEGIN,(void*)wifi_ram_test,sizeof(wifi_ram_test));
			host_write_trout_reg(write_sys_value, (UWORD32)rSYSREG_WIFI_CFG);
			host_write_trout_reg(0x27,(0x4072 <<2));//dft wave len 
			host_write_trout_reg(1,(0x4071 <<2)); //dft controll enable
		}
		else
		{
			printk(" 0\n");
			printk("g_save_reg1 :%u\n", g_save_reg1);
			printk("g_save_reg2 :%u\n", g_save_reg2);
			printk("g_sys_reg :%u\n", g_sys_reg);
			host_write_trout_reg(g_sys_reg, (UWORD32)rSYSREG_WIFI_CFG);
			host_write_trout_reg(g_save_reg1, (0x4072 <<2));
			host_write_trout_reg(g_save_reg2, (0x4071 <<2));
			g_tx_flag = 1;
            //npi_tx_data_qwk();
            tx_packet_test();
		}
        g_npi_tx_pkt_count = 0;
		ret = 0;
	}
	else if(!strcasecmp(cmd, "stop"))
	{
		g_tx_flag = 0;
        //npi_tx_data_cwk();
		ret = 0;
		host_write_trout_reg(g_sys_reg, (UWORD32)rSYSREG_WIFI_CFG);
		host_write_trout_reg(g_save_reg1, (0x4072 <<2));
		host_write_trout_reg(g_save_reg2, (0x4071 <<2));
        g_npi_tx_pkt_count = 0;
	}
	return ret;
}

static int itm_rx_data(struct net_device *dev, struct iw_request_info *info,
                                        union iwreq_data *wrqu, char *extra)
{
	int ret = 0;
	int i = 0;
	UWORD8 * extra_data = (UWORD8 * )extra;
	UWORD8 cmd[64] = {0};
	int tmp_index = 0;
	int len = 0;
	static UWORD32 rx_start_count = 0;
	UWORD32 rx_end_count = 0;
	UWORD32 tmp = 0;
	
	memcpy(cmd,&extra_data[tmp_index + 1],extra_data[tmp_index]);
	cmd[extra_data[tmp_index]] = '\0';
	tmp_index += 1 + extra_data[tmp_index];

	printk("g_mac_stats.pewrx %u\n", g_mac_stats.pewrx);

	printk("cmd is: %c\n", cmd[0]);
	g_npi_scan_flag = 0;
    printk("%s-%d, g_npi_scan_flag = %d\n",__FUNCTION__,__LINE__,g_npi_scan_flag);
	if(!strcasecmp(cmd, "start"))
	{
		printk("rx start\n");
		restart_mac(&g_mac, 0);
		disable_machw_phy_and_pa();
		mdelay(100);
		clr_rx_count_reg();
		mdelay(100);
		enable_machw_phy_and_pa();
		tmp = get_fcs_count();
		g_rx_right_start_count = get_machw_rx_end_count();
		g_rx_right_start_count -= tmp;
		g_rx_error_start_count = get_machw_rx_error_end_count();
		g_rx_error_start_count += tmp;
		g_rx_flag = 1;

		// for cmcc 11n receive test[zhongli 20130925]
		write_dot11_phy_reg(0xFF, 0x00);
    		write_dot11_phy_reg(0xF0, 0x65);
	}
	else if(!strcasecmp(cmd, "stop")){
		g_rx_flag = 0;
		
		// for cmcc 11n receive test[zhongli 20130925]
		write_dot11_phy_reg(0xFF, 0x00);
    		write_dot11_phy_reg(0xF0, 0x01);
	}

	return ret;
}


static int itm_set_tx_rate(struct net_device *dev, struct iw_request_info *info,
                                        union iwreq_data *wrqu, char *extra)
{
	int ret = 0;
	int i = 0;
	UWORD8 * extra_data = (UWORD8 * )extra;
	UWORD8 cmd[64] = {0};
	int tmp_index = 0;
	UWORD8 * host_req  = NULL;
	UWORD16  trout_rsp_len = 0;
	int len = 0;
	UWORD8 rate = 0;

	printk("zhuyg: enter itm_set_tx_rate\n");


	for (i = 0; i < extra_data[0]; i++)
	{
		if (extra_data[i+1] < '0' || extra_data[i+1] > '9')
			break;
		rate = rate * 10 + extra[i+1] - '0';
		printk("zhuyg: rate is: %u\n", rate);
	}
	printk("zhuyg: rate is: %u\n", rate);
	set_tx_rate(rate);

	g_user_tx_rate = rate;
		
	return ret;
}


// debug info
static void print_power_info(void)
{
	UWORD32 val1 = 0;
	UWORD32 val2 = 0;
	UWORD32 val3 = 0;

	read_dot11_phy_reg((UWORD32)rTXPOWER11A, &val1);
	read_dot11_phy_reg((UWORD32)rTXPOWER11B, &val2);
	read_dot11_phy_reg((UWORD32)rTXPOWCALCNTRL, &val3);

	printk("npi %d: read 0x70 = %#x, 0x71 = %#x, 0x7A = %#x\n",
		__LINE__, val1,val2,val3);
}

UWORD32 pwr_level_b_val = 0;
UWORD32 pwr_level_a_val = 0;

static int itm_set_tx_level(struct net_device *dev, struct iw_request_info *info,
                                        union iwreq_data *wrqu, char *extra)
{
	int ret = 0;
	int i = 0;
	UWORD8 * extra_data = (UWORD8 * )extra;
	UWORD8 cmd[64] = {0};
	int tmp_index = 0;
	UWORD8 * host_req  = NULL;
	UWORD16  trout_rsp_len = 0;
	int len = 0;
	UWORD8 level = 0;
	UWORD8 reg_value = 0;
	
	printk("zhuyg: enter %s\n", __FUNCTION__);

	printk("zhuyg: extra_data[0] = %#x \n", extra_data[0]);

	for (i = 0; i < extra_data[0]; i++)
	{
		level = (level * 10) + (extra[i+1] - '0');
		printk("zhuyg: level is: %u\n", level);
	}
	
	read_dot11_phy_reg((UWORD32)0xFF, &reg_value);
	write_dot11_phy_reg((UWORD32)0xFF, (UWORD32)0);


	print_power_info();

	switch(level)
	{
		case 11:  // b rate low power
		{
			pwr_level_b_val = 0x01;
			//write_dot11_phy_reg(rTXPOWER11B, level_val);
			set_phy_tx_power_level_11b((UWORD8)pwr_level_b_val);
			break;
		}
		case 12:  // b rate middle power
		{
			pwr_level_b_val = 0x10;
			//write_dot11_phy_reg(rTXPOWER11B, level_val);
			set_phy_tx_power_level_11b((UWORD8)pwr_level_b_val);
			break;
		}
		case 13:  // b rate high power
		{
			pwr_level_b_val = 0x00;
			//write_dot11_phy_reg(rTXPOWER11B, level_val);
			set_phy_tx_power_level_11b((UWORD8)pwr_level_b_val);
			break;
		}
		case 21:  // g,n rate low power
		{
			pwr_level_a_val = 0x20;
			//write_dot11_phy_reg(rTXPOWER11A, level_val);
			set_phy_tx_power_level_11a((UWORD8)pwr_level_a_val);
			break;
		}
		case 22:  // g,n rate high power
		{
			pwr_level_a_val = 0x30;
			//write_dot11_phy_reg(rTXPOWER11A, level_val);
			set_phy_tx_power_level_11a((UWORD8)pwr_level_a_val);
			break;
		}
		case 0:  // clear user configed b and g/n rate power level
		{
			pwr_level_b_val = TXPOWER11B;
			set_phy_tx_power_level_11b((UWORD8)pwr_level_b_val);
			
			pwr_level_a_val = TXPOWER11A;
			set_phy_tx_power_level_11a((UWORD8)pwr_level_a_val);
			break;
		}
		default:
		{
			printk("npi: set error power level!\n");
			ret = -1;
			break;
		}
	}
	
	print_power_info();

	
	write_dot11_phy_reg((UWORD32)0xFF, (UWORD32)reg_value);

	return ret;
}


static int itm_set_mac_addr(struct net_device *dev, struct iw_request_info *info,
                                        union iwreq_data *wrqu, char *extra)
{
	int ret = 0;
	int i = 0, j = 0;
	UWORD8 * extra_data = (UWORD8 * )extra;
	UWORD8 cmd[64] = {0};
	UWORD8 mac_addr[6] = {0};
	int tmp_index = 0;
	UWORD8 * host_req  = NULL;
	UWORD16  trout_rsp_len = 0;
	int len = 0;
	UWORD8 rate = 0;

	printk("zhuyg: enter itm_set_mac_addr\n");

	memcpy(cmd,&extra_data[tmp_index + 1],extra_data[tmp_index]);
	cmd[extra_data[tmp_index]] = '\0';
	tmp_index += 1 + extra_data[tmp_index];

	for (i = 0; i < tmp_index -1; i++)
	{
		printk("cmd[i]: %c\n", cmd[i]);
		if (cmd[i] >= '0' && cmd[i] <= '9')
			mac_addr[j] = mac_addr[j]*16 + cmd[i] - '0';
		else if(cmd[i] >= 'a' && cmd[i] <= 'f')
			mac_addr[j] = mac_addr[j]*16 + cmd[i] - 87;
		else if(cmd[i] >= 'A' && cmd[i] <= 'F')
			mac_addr[j] = mac_addr[j]*16 + cmd[i] - 55;
		else
		{
			printk("set_mac_addr, xxx\n");
			return -1;
		}
		printk("j is: %d\n", j);
		printk("mac_addr[j]: %x\n", mac_addr[j]);
		if (i % 2 == 1)
			j++;
	}
	
	
	printk("set_mac: %x-%x-%x-%x-%x-%x\n", mac_addr[0], mac_addr[1], mac_addr[2],
						mac_addr[3], mac_addr[4], mac_addr[5]);
	mset_StationID(mac_addr);
	memcpy(g_mac_addr, mac_addr, 6);
	
	return ret;
}


static int itm_set_rx_count(struct net_device *dev, struct iw_request_info *info,
                                        union iwreq_data *wrqu, char *extra)
{
	int ret = 0;
	int i = 0;
	UWORD8 * extra_data = (UWORD8 * )extra;
	UWORD32 fcs_error_count = 0;
	UWORD32 rx_end_count = 0;

	printk("zhuyg: %s\n", __FUNCTION__);

	clr_rx_count_reg();

	rx_end_count = get_machw_rx_end_count();

	fcs_error_count = get_fcs_count();
	printk("zhuyg: fcs_error_count: %u\n", fcs_error_count);
	printk("zhuyg: rx_end count is: %u\n", rx_end_count);
	
	g_rx_right_start_count = rx_end_count;
	g_rx_right_start_count -= fcs_error_count;
	g_rx_error_start_count = get_machw_rx_error_end_count();
	g_rx_error_start_count += fcs_error_count;

	
	return ret;
}

static int itm_set_tx_mode(struct net_device *dev, struct iw_request_info *info,
                                        union iwreq_data *wrqu, char *extra)
{
	int ret = 0;
	int i = 0;
	static int normal_send_flag = 1;
	static UWORD32 bank_num = 0;
	UWORD32 save_value1 = 0;
	UWORD32 save_value2 = 0;
	UWORD32 reg_value = 0;
	UWORD8 * extra_data = (UWORD8 * )extra;

	printk("%s\n", __FUNCTION__);
	printk("extra[0] is: %c\n", extra[0]);


	write_dot11_phy_reg(0x8d, 0x0);
	write_dot11_phy_reg(0xFF, 0x00);

	if (extra[0] == '0') //send normal data
	{
		printk("send normal data \n");
		write_dot11_phy_reg(0x8d, 0);
	}
	else if(extra[0] == '1') //send random data
	{
		printk("send random data \n");
		write_dot11_phy_reg(0x8c, 0x41);
		write_dot11_phy_reg(0x8d, 0x8d);
	}
	else if(extra[0] == '2') //send all number 1
	{
		printk("send all number 1 \n");
		write_dot11_phy_reg(0x8c, 0x1);
		write_dot11_phy_reg(0x8d, 0x8d);
	}
	else if(extra[0] == '3') //send all number 0
	{
		printk("send all number 1 \n");
		write_dot11_phy_reg(0x8c, 0x31);
		write_dot11_phy_reg(0x8d, 0x8d);
	}
	else if(extra[0] == '4') //send all number 01010101
	{
		printk("send all number 01010101 \n");
		write_dot11_phy_reg(0x8c, 0xF1);
		write_dot11_phy_reg(0x8d, 0x8d);
	}
	else
		ret = -1;
	
	return ret;
}
static int itm_set_connect_ap(struct net_device *dev, struct iw_request_info *info,
                                        union iwreq_data *wrqu, char *extra)
{
	UWORD8 * extra_data = (UWORD8 * )extra;
	UWORD8 cmd[64] = {0};
	int tmp_index = 0;
	UWORD8 i = 0;
	UWORD8 retry_count = 10;
	int ret = -1;

	printk("enter: %s\n", __FUNCTION__);

	g_default_scan_limit = 0;
	g_connect_ok_flag = 0;
	memcpy(cmd,&extra_data[tmp_index + 1],extra_data[tmp_index]);

	printk("npi: connect ap is: %s\n", cmd);
	mset_DesiredSSID(cmd);
	g_npi_scan_flag = 1;
    printk("%d, g_npi_scan_flag = %d\n",__LINE__,g_npi_scan_flag);
	restart_mac(&g_mac, 0);
    
	printk("npi: start initiate_scan_procedure\n");

	initiate_scan_procedure(&g_mac);

	for (i = 0; i < retry_count; i++)
	{
		if (g_connect_ok_flag != 0)
		{
			ret = 0;
			break;
		}
		msleep(1000);
		printk("npi: waiting network connect ok, retry count: %u\n", i);
	}
	return 0;
}
static int itm_get_current_channel(struct net_device *dev, struct iw_request_info *info,
                                        union iwreq_data *wrqu, char *extra)
{

        int ret = 0;
        UWORD8 * host_req  = NULL;
        UWORD8 * trout_rsp = NULL;
        UWORD16  trout_rsp_len = 0;
        UWORD8 channel = 0;
        struct iw_point * tmp_extra = (struct iw_point *)wrqu;
		UWORD8 * tmp_data = (UWORD8 * )extra;
		printk("zhuyg : itm_get_current_channel\n");
        host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

        if(host_req == NULL)
        {
                return -ENOMEM;
        }
        host_req[0] = WID_PRIMARY_CHANNEL & 0xFF;
        host_req[1] = (WID_PRIMARY_CHANNEL & 0xFF00) >> 8;

        trout_rsp = config_if_for_iw(&g_mac,host_req,2,'Q',&trout_rsp_len);

        if( trout_rsp == NULL )
        {
                ret = -EINVAL;
                return ret;
        }

        trout_rsp_len -= MSG_HDR_LEN;
        if((ret = get_trour_rsp_data((UWORD8*)(&channel),sizeof(UWORD8),&trout_rsp[MSG_HDR_LEN], trout_rsp_len,WID_CHAR))  ==  0)
        {
                pkt_mem_free(trout_rsp);
                PRINTK_ITMIW("get_trour_rsp_data == fail\n");
                return -1;
        }
        pkt_mem_free(trout_rsp);

		tmp_extra->length = channel;
		
        printk("zhuyg: get channel is: %u\n", channel);

        return ret;
}

static int itm_get_rx_count(struct net_device *dev, struct iw_request_info *info,
                                        union iwreq_data *wrqu, char *extra)
{
	int ret = 0;
	u16 len = 0;
	struct iw_point * tmp_extra = (struct iw_point *)wrqu;
	UWORD8 * tmp_data = (UWORD8 * )extra;
	UWORD32 rx_end_count = 0;
	UWORD32 rx_count = 0;
	UWORD32 fcs_error_count = 0;
	
	printk("zhuyg : itm_get_rx_count\n");

	
rx_end_count = get_machw_rx_end_count();
fcs_error_count = get_fcs_count();
printk("%s: 1 test rx_end %u - fcs_err %u = rx_count %u\n", __FUNCTION__,rx_end_count,
	fcs_error_count,rx_count);
	
	disable_machw_phy_and_pa();
	
	rx_end_count = get_machw_rx_end_count();
	fcs_error_count = get_fcs_count();
	
printk("%s: 2 test rx_end %u - fcs_err %u = rx_count %u\n", __FUNCTION__,rx_end_count,
	fcs_error_count,rx_count);
	
	enable_machw_phy_and_pa();

	if(rx_end_count >= fcs_error_count)
	{
		//rx_end_count = rx_end_count - g_rx_right_start_count - fcs_error_count;
		rx_count = rx_end_count - fcs_error_count;
		printk("%s: rx_end %u - fcs_err %u = rx_count %u\n", __FUNCTION__,rx_end_count,
			fcs_error_count,rx_count);
	}
	else
	{
		rx_count = 0;
		printk("%s: Warning: rx_end %u < fcs_err %u ! so rx_count %u!\n", 
			__FUNCTION__,rx_end_count,fcs_error_count,rx_count);
	}
	len = int_to_str(rx_count, tmp_data);
	//printk("zhuyg: rx end count is: %u\n", rx_end_count);
	//printk("len is: %u\n", len);
	
	printk("ret is: %s\n", tmp_data);
	tmp_extra->length = len;

rx_end_count = get_machw_rx_end_count();
fcs_error_count = get_fcs_count();
printk("%s: 3 test rx_end %u - fcs_err %u = rx_count %u\n", __FUNCTION__,rx_end_count,
	fcs_error_count,rx_count);
	
	return ret;
}

static int itm_get_rx_error(struct net_device *dev, struct iw_request_info *info,
                                        union iwreq_data *wrqu, char *extra)
{
	int ret = 0;
	u16 len = 0;
	struct iw_point * tmp_extra = (struct iw_point *)wrqu;
	UWORD8 * tmp_data = (UWORD8 * )extra;
	UWORD32 fcs_error_count = 0;
	UWORD32 rxerrorpacket = get_machw_rx_error_end_count();

	printk("zhuyg : itm_get_rx_error\n");

	//rxerrorpacket = rxerrorpacket - g_rx_error_start_count;
	disable_machw_phy_and_pa();
	fcs_error_count = get_fcs_count();
	enable_machw_phy_and_pa();
	rxerrorpacket += fcs_error_count;
	
	len = int_to_str(rxerrorpacket, tmp_data);
	printk("zhuyg: rx_error is: %d\n", rxerrorpacket);
	tmp_extra->length = len;
	return ret;
}

static int itm_get_rx_status(struct net_device *dev, struct iw_request_info *info,
                                        union iwreq_data *wrqu, char *extra)
{
	int ret = 0;
	u16 len = 0;
	u16 all_len = 0;
	
	struct iw_point * tmp_extra = (struct iw_point *)wrqu;
	UWORD8 * tmp_data = (UWORD8 * )extra;
	u32 fcs_fail_cnt = 0, frame_filter = 0, frame_header_filter = 0,
		rx_q_lost_cnt = 0, rx_q_cnt = 0, rx_end_cnt = 0, rx_end_error_cnt = 0, 
		duplicate_cnt = 0, ampdu_rx_cnt = 0, mpdu_in_ampdu_cnt = 0, crc_pass_cnt = 0, rx_all_cnt = 0;

	printk("zhuyg : enter: %s\n", __FUNCTION__);
	disable_machw_phy_and_pa();
	clr_rx_count_reg();
	enable_machw_phy_and_pa();
	msleep(5000);
	fcs_fail_cnt = host_read_trout_reg((UWORD32)rMAC_FCS_FAIL_COUNT);
	frame_filter = host_read_trout_reg((UWORD32)rMAC_RX_FRAME_FILTER_COUNTER);
	frame_header_filter = host_read_trout_reg((UWORD32)rMAC_RX_MAC_HEADER_FILTER_COUNTER);
	rx_q_lost_cnt = host_read_trout_reg((UWORD32)rMAC_RXQ_FULL_COUNTER);
	rx_q_cnt = host_read_trout_reg((UWORD32)rMAC_RX_RAM_PACKET_COUNTER);
	rx_end_cnt = host_read_trout_reg((UWORD32)rMAC_RX_END_COUNT);
	rx_end_error_cnt = host_read_trout_reg((UWORD32)rMAC_RX_ERROR_END_COUNT);
	duplicate_cnt = host_read_trout_reg((UWORD32)rMAC_DUP_DET_COUNT);
	ampdu_rx_cnt = host_read_trout_reg((UWORD32)rMAC_AMPDU_RXD_COUNT);
	mpdu_in_ampdu_cnt = host_read_trout_reg((UWORD32)rMAC_RX_MPDUS_IN_AMPDU_COUNT);

	printk("fcs_fail_cnt:%u\n", fcs_fail_cnt);
	printk("frame_filter:%u\n", frame_filter);
	printk("frame_header_filter:%u\n", frame_header_filter);
	printk("rx_q_lost_cnt:%u\n", rx_q_lost_cnt);
	printk("rx_q_cnt:%u\n", rx_q_cnt);
	printk("rx_end_cnt:%u\n", rx_end_cnt);
	printk("rx_end_error_cnt:%u\n", rx_end_error_cnt);
	printk("duplicate_cnt:%u\n", duplicate_cnt);
	printk("ampdu_rx_cnt:%u\n", ampdu_rx_cnt);
	printk("mpdu_in_ampdu_cnt:%u\n", mpdu_in_ampdu_cnt);
	len = int_to_str(fcs_fail_cnt, tmp_data+all_len+1);
	tmp_data[all_len] = len + '0';
	all_len = all_len + len + 1;
	printk("len is: %u\n", len);

	len = int_to_str(frame_filter, tmp_data+all_len+1);
	tmp_data[all_len] = len + '0';
	all_len = all_len + len + 1;
	printk("len is: %u\n", len);

	len = int_to_str(frame_header_filter, tmp_data+all_len+1);
	tmp_data[all_len] = len + '0';
	all_len = all_len + len + 1;
	printk("len is: %u\n", len);

	len = int_to_str(rx_q_lost_cnt, tmp_data+all_len+1);
	tmp_data[all_len] = len + '0';
	all_len = all_len + len + 1;
	printk("len is: %u\n", len);

	len = int_to_str(rx_q_cnt, tmp_data+all_len+1);
	tmp_data[all_len] = len + '0';
	all_len = all_len + len + 1;
	printk("len is: %u\n", len);

	len = int_to_str(rx_end_cnt, tmp_data+all_len+1);
	tmp_data[all_len] = len + '0';
	all_len = all_len + len + 1;
	printk("len is: %u\n", len);

	len = int_to_str(rx_end_error_cnt, tmp_data+all_len+1);
	tmp_data[all_len] = len + '0';
	all_len = all_len + len + 1;
	printk("len is: %u\n", len);

	len = int_to_str(duplicate_cnt, tmp_data+all_len+1);
	tmp_data[all_len] = len + '0';
	all_len = all_len + len + 1;
	printk("len is: %u\n", len);

	len = int_to_str(ampdu_rx_cnt, tmp_data+all_len+1);
	tmp_data[all_len] = len + '0';
	all_len = all_len + len + 1;
	printk("len is: %u\n", len);

	len = int_to_str(mpdu_in_ampdu_cnt, tmp_data+all_len+1);
	tmp_data[all_len] = len + '0';
	all_len = all_len + len + 1;
	printk("len is: %u\n", len);

	
	return ret;
}
//get code from lihuai
//reg addr
//For 128k mem get
char *share_ram;
struct completion share_ram_completion;
static struct task_struct *thread_IQ = NULL;

static int caculate_rssi(u32 *valueI, u32 *valueQ, int index) {
	unsigned int i, M, G=0;
	u64 sum = 0;
	//\u5e73\u65b9\u548c
	for(i = 0; i < index; i++) {
		sum += valueI[i] * valueI[i] + valueQ[i] * valueQ[i];
	}
	sum = sum >> 10;		// * 1/N
	int tmp = sum;
	M = 0;
	//\u83b7\u53d6M \u503c
	if(tmp > 15) {
		while(tmp > 15) {
			M++;
			tmp = tmp >> 1;
		}
	} else {
		printk("sinal too weak\n");
		return -1;
	}
	//R\u5bf9\u5e94\u7684\u503c\u8868
	int R_table[8];
	R_table[0] = -2730;
	R_table[1] = -2220;
	R_table[2] = -1760;
	R_table[3] = -1347;
	R_table[4] = -970;
	R_table[5] = -620;
	R_table[6] = -300;
	R_table[7] = 0;
	
	return (R_table[tmp - 8] + 11760 + 3010 * M - 39500 - G * 1000);
}

static int start_thread_IQ(void *data)
{
     int status = 0;
     while(1) {
	    //get status reg
	    u32 reg_data;
	    reg_data = host_read_trout_reg(COM_REG_CAPTURE_CTRL);
	    status = reg_data & 0x400;
		printk("zhouxw:status=%d,reg_data=%d\n",status,reg_data);
	    if(status == 0x400) {
			//The share ram is prepared. read to host mem.
			host_read_trout_ram(share_ram,0,16 * 1024);
			break;
	    } else {
			ssleep(1);
	    }
     }
     complete(&share_ram_completion);
     return 0;
}

/* zhouxw mod it for rssi changing, 0609 */
#if 0
static int itm_get_rssi_value(struct net_device *dev, struct iw_request_info *info,
                                        union iwreq_data *wrqu, char *extra)
{
	int ret = 0;
	u32 reg_data;
	u16 len = 0;
	struct iw_point * tmp_extra = (struct iw_point *)wrqu;
	UWORD8 * tmp_data = (UWORD8 * )extra;

	printk("zhuyg : %s\n", __FUNCTION__);
		
	reg_data = 0x82;
	host_write_trout_reg(reg_data, COM_REG_CAPTURE_CTRL);		
		
	//alloc buf
	share_ram = kmalloc(16 * 1024 *sizeof(char), GFP_KERNEL);
	if(NULL == share_ram) {
		printk("mem alloc failed\n");
		return -ENOMEM;
	}
	mdelay(5);
	//start thread 
	init_completion(&share_ram_completion);
	thread_IQ = kthread_run(start_thread_IQ, NULL, "thread_IQ");	
	wait_for_completion(&share_ram_completion);

	u32 *tmp_ram = (u32*)share_ram;

	u32*valueI = kmalloc(1024 *sizeof(u32), GFP_KERNEL);
	u32 *valueQ = kmalloc(1024 *sizeof(u32), GFP_KERNEL);
	if(valueI == NULL || valueQ == NULL) {
		printk("alloc mem error\n");
		return -ENOMEM;
	}
		
	int index = 0;
	int i;
	for(i = 0; i < 1024; i++) {
		int value = *tmp_ram;
		valueI[index] = value & 0x3ff;
		valueQ[index++] = value & 0xffc00 >> 10;
		tmp_ram++;
	}
		
	unsigned int rssi = caculate_rssi(valueI, valueQ, 1024);

	len = int_to_str(rssi, tmp_data);
	tmp_extra->length = len;
	
	reg_data = 0x01;   
    host_write_trout_reg(reg_data, COM_REG_CAPTURE_CTRL);
    kfree(share_ram);
	kfree(valueI);
	kfree(valueQ); 
	
	return ret;
}
#else

/* example: after cal is 149, tmp_data = 195        */
/*          195 - 100 = 95, 95 --> 0x95, 0x95 = 149 */
/* example: after cal is 117, tmp_data = 175 */
/* example: after cal is 137, tmp_data = 189 */
u32 convert_to_10(u32 tmp_data)
{
    u32 bit[3] = {0};

    if(tmp_data <= 100)
    {
        printk("convert_to_10: data <= 100\n");
        return 0;
    }
    
    tmp_data = tmp_data - 100;
    
    if(tmp_data>=100 && tmp_data<1000)
    {
        bit[0] = tmp_data%10;
        bit[1] = (tmp_data/10)%10;
        bit[2] = tmp_data/100;
        tmp_data = bit[0] + (bit[1]<<4) + (bit[2]<<8);
    }
    else if(tmp_data>10 && tmp_data<100)
    {
        bit[0] = tmp_data%10;
        bit[1] = tmp_data/10;
        tmp_data = bit[0] + (bit[1]<<4);
    }
    else
        tmp_data = 0;
    
    return tmp_data;
}

u16 rssi_to_str(int i, char *string)
{
	u16 len = 0;
	int power, j;
	j = i;
	
	//*string++='-';
	//len++;
	
	for (power=1; j >=10; j /= 10)
		power *= 10;
	
	for (; power > 0; power /= 10)
	{
		*string++ = '0' + i/power;
		i %= power;
		len++;
	}
	
	/*
	*string++='d';
	len++;
	*string++='B';
	len++;
	*string++='m';
	len++;
	*/
	*string = '\0';

	return len;
}

static int itm_get_rssi_value(struct net_device *dev, struct iw_request_info *info,
                                        union iwreq_data *wrqu, char *extra)
{
	int ret = 0;
	u32 reg_data;
	u16 len = 0;
	struct iw_point * tmp_extra = (struct iw_point *)wrqu;
	UWORD8 * tmp_data = (UWORD8 * )extra;
	unsigned int rssi = 0;

    write_dot11_phy_reg(0xff, 0x0);
    read_dot11_phy_reg(0x78, &reg_data);
    
    printk("zhouxw: read 0x78 is %d\n",reg_data);	

#if 0
    rssi = convert_to_10(reg_data);
	printk("zhouxw: read 0x78 after cal is %d, reg_data = %d\n",rssi,reg_data);
    if(rssi == 0)
    {
        printk("%s: rssi value get error!\n", __FUNCTION__);
        return -1;
    }

	len = int_to_str(rssi, tmp_data);
	tmp_extra->length = len;
#else // calculate rssi for dbm
    if(reg_data<256) {
        rssi=256-reg_data;
        len=rssi_to_str(rssi, tmp_data);
		tmp_extra->length = len;
		printk("zhouxw:return rssi is -%s dbm\n",(char*)tmp_data);
	}
	else {
        rssi=reg_data-256;
		len = int_to_str(rssi, tmp_data);
		tmp_extra->length = len;
    }
	
#endif

	return ret;
}
#endif
//get code from lihuai end


static int itm_get_tx_power_level(struct net_device *dev, struct iw_request_info *info,
                                        union iwreq_data *wrqu, char *extra)
{
    int ret = 0;
    u16 len = 0;
    struct iw_point * tmp_extra = (struct iw_point *)wrqu;
    UWORD8 * tmp_data = (UWORD8 * )extra;
    UWORD8 reg_value = 0;
    UWORD8 power_level_b = 0;
    UWORD8 power_level_a = 0;
    int    power_level = 0;
    int    level_a = 0, level_b = 0;

    printk("zhuyg : enter :%s\n", __FUNCTION__);

	
	read_dot11_phy_reg((UWORD32)0xFF, &reg_value);
	write_dot11_phy_reg((UWORD32)0xFF,(UWORD32)0);

	
	read_dot11_phy_reg(rTXPOWCALCNTRL, &power_level);
	printk("rTXPOWCALCNTRL = %#x\n",power_level);

	read_dot11_phy_reg(rTXPOWER11B, &power_level_b);
	printk("rTXPOWER11B power_level = %#x\n",power_level_b);
	
	read_dot11_phy_reg(rTXPOWER11A, &power_level_a);
	printk("rTXPOWER11A power_level = %#x\n",power_level_a);


	switch(power_level_b)
	{
		case 0x01:  // b rate low power
		{
			level_b = 11;
			break;
		}
		case 0x10:  // b rate middle power
		{
			level_b = 12;
			break;
		}
		case 0x00:  // b rate high power
		{
			level_b = 13;
			break;
		}
		default:
		{
			level_b = 0;
			break;
		}
	}
	
	switch(power_level_a)
	{
		case 0x20:  // g,n rate low power
		{
			level_a = 21;
			break;
		}
		case 0x30:  // g,n rate high power
		{
			level_a = 22;
			break;
		}
		default:
		{
			level_a = 0;
			break;
		}
	}

	write_dot11_phy_reg((UWORD32)0xFF, (UWORD32)reg_value);

	sprintf(tmp_data, "b: %d, g/n: %d", level_b,level_a);
	len = strlen(tmp_data);
	
	printk("power level value is: %u,%u (%#x, %#x), ret=%s.\n", 
		level_b, level_a,
		power_level_b, power_level_a, tmp_data);
	
    //len = int_to_str(power_level, tmp_data);
    tmp_extra->length = len;

	
    return ret;
}

//zhuyg add end
#endif

#ifdef IBSS_BSS_STATION_MODE

// add for cmcc test [zhongli wang 20130911] start
extern UWORD32 g_cmcc_set_max_pwr ;
extern UWORD32 g_cmcc_cfg_tx_rate ;
extern UWORD32 g_cmcc_set_aci ;
#endif
static int itm_atoi(char ascii){
	return ((ascii >= '0') && (ascii <= '9'))?ascii-'0':-1;
}

static int itm_strtoi(const char* str, int* ret_num){
	int i = 0;
	*ret_num = 0;
	
	if((NULL ==  str) || (NULL == ret_num)){
		return -1;
	}
	for(i=0; '\0' != str[i]; i++){
		if(-1 != itm_atoi(str[i])){
			*ret_num = (*ret_num)*10+itm_atoi(str[i]);
		}
		else{
			return -1;
		}
	}
	return 0;
}

#ifdef IBSS_BSS_STATION_MODE

static int
itm_set_cmcc_test(struct net_device *dev, struct iw_request_info *info,
        			union iwreq_data *wrqu, char *extra)
{
	UWORD8 * extra_data = (UWORD8 * )extra;
	UWORD8 cmd[64] = {0,};
	//UWORD8 val[64] = {0,};
	int tmp_index = 0;

	//int ret = 0;
	//UWORD8 * host_req  = NULL;
	//UWORD16  trout_rsp_len = 0;
	//int len = 0;

	PRINTK_ITMIW("itm_set_cmcc_test fuc \n");

       /*\B4\D3\C9ϲ㴫\CF\C2\C0\B4\B5\C4\CAǿ\E9buf\A3\AC\C0\EF\C3\E6\B5\C4\CA\FD\BEݽӿ\DA\CAǣ\BA| cmd len\A3\AC1 byte | cmd\A3\AClen byte |*/
	memcpy(cmd, extra_data+1, extra_data[0]);
	tmp_index = 1 + extra_data[0]; // index to next cmd

	if(0 == strcasecmp(cmd, "set_unsleep"))
	{
		printk("set_unsleep for cmcc test\n");
		//todo:unsleep
	}
	else if(0 == strcasecmp(cmd, "unset_unsleep")){
		printk("unset_unsleep for cmcc test\n");
		//todo:
	}
	else if(0 == strcasecmp(cmd, "set_max_power")){
		printk("set_max_power for cmcc test\n");
		g_cmcc_set_max_pwr = 1;
#if 0
#ifdef IBSS_BSS_STATION_MODE		
		g_cmcc_test_mode = 1;
		set_machw_cw_vi(2, 2);
		set_machw_cw_vo(2, 2);
		set_machw_cw_be(2, 2);
		set_machw_cw_bk(2, 2);
#endif	
#endif
	}
	else if(0 == strcasecmp(cmd, "unset_max_power")){
		printk("unset_max_power for cmcc test\n");
		// enforce to use nv tx power
		g_cmcc_set_max_pwr = 0; 
#if 0
#ifdef IBSS_BSS_STATION_MODE		
		g_cmcc_test_mode = 0;
		set_machw_cw_bk(mget_EDCATableCWmax(AC_BK),
                                        mget_EDCATableCWmin(AC_BK));
        	set_machw_cw_be(mget_EDCATableCWmax(AC_BE),
                                        mget_EDCATableCWmin(AC_BE));
        	set_machw_cw_vi(mget_EDCATableCWmax(AC_VI),
                                        mget_EDCATableCWmin(AC_VI));
       	 set_machw_cw_vo(mget_EDCATableCWmax(AC_VO),
                                        mget_EDCATableCWmin(AC_VO));
#endif
#endif
	}
	else if(0 == strcasecmp(cmd, "set_rate")){
		int cfg_rate = 0;
		memset(cmd, 0, sizeof(cmd));
		memcpy(cmd, extra_data+tmp_index+1, extra_data[tmp_index]);
		if(0 == itm_strtoi(cmd, &cfg_rate)){
			if((cfg_rate >= 0) && (cfg_rate <= 65)){
				printk("set_rate %d for cmcc test\n", cfg_rate);
				g_cmcc_cfg_tx_rate = cfg_rate;
			}
		}
		else{
			printk("Error:[%s] set rate %s is not a validate number\n", __FUNCTION__, cmd);
		}
	}
	else if(0 == strcasecmp(cmd, "set_aci")){
		printk("set_aci for cmcc test\n");
		g_cmcc_set_aci = 1;
	}
	else if(0 == strcasecmp(cmd, "unset_aci")){
		printk("unset_aci for cmcc test\n");
		g_cmcc_set_aci = 0;
	}
	else{
		printk("Error:[%s] unsupport command %s\n", __FUNCTION__, cmd);
		return -1;
	}	
	return 0;
}
// add for cmcc test [zhongli wang 20130911] end
#endif

static int
itm_siw_trout(struct net_device *dev, struct iw_request_info *info,
        			union iwreq_data *wrqu, char *extra)
{
	int flags       = wrqu->data.flags;

	switch(flags)
	{
		case 0:
			return itm_swdirect(dev,info,wrqu,extra);
		case 1:
			return itm_sampdu(dev,info,wrqu,extra);
		case 2:
			return itm_samsdu(dev,info,wrqu,extra);
		case 3:
			return itm_sblockack(dev,info,wrqu,extra);
		case 4:	
			return itm_sshortgi(dev,info,wrqu,extra);
		case 5:
			return itm_smu_domain(dev,info,wrqu,extra);
		case 6:
			return itm_swps(dev,info,wrqu,extra);
#ifdef TROUT_WIFI_NPI
		case 7:
			return itm_tx_data(dev, info, wrqu, extra);
		case 8:
			return itm_set_tx_rate(dev, info, wrqu, extra);
		case 9:
			return itm_set_tx_level(dev, info, wrqu, extra);
		case 10:
			return itm_rx_data(dev, info, wrqu, extra);
		case 11:
			return itm_set_mac_addr(dev, info, wrqu, extra);
		case 12:
			return itm_set_rx_count(dev, info, wrqu, extra);
		case 13:
			return itm_set_tx_mode(dev, info, wrqu, extra);
		case 14:
			return itm_set_connect_ap(dev, info, wrqu, extra);
#endif
#ifdef IBSS_BSS_STATION_MODE
		case 15: 
			return itm_set_cmcc_test(dev, info, wrqu, extra);
#endif
		default:
			return -1;
	}

	return -1;
}

static int
itm_giw_trout(struct net_device *dev, struct iw_request_info *info,
        			union iwreq_data *wrqu, char *extra)
{
	int flags       = wrqu->data.flags;

	switch(flags)
	{
		case 0:
			return itm_gwdirect(dev,info,wrqu,extra);
		case 1:
			return itm_gampdu(dev,info,wrqu,extra);
		case 2:
			return itm_gamsdu(dev,info,wrqu,extra);
		case 3:
			return itm_gblockack(dev,info,wrqu,extra);
		case 4:	
			return itm_gshortgi(dev,info,wrqu,extra);
		case 5:
			return itm_gmu_domain(dev,info,wrqu,extra);
		case 6:
			return itm_gwps(dev,info,wrqu,extra);	
#ifdef TROUT_WIFI_NPI
		case 7:
			return itm_get_current_channel(dev, info, wrqu, extra);
		case 8:
			return itm_get_rx_count(dev, info, wrqu, extra);
		case 9:
			return itm_get_rx_error(dev, info, wrqu, extra);
		case 10:
			return itm_get_rssi_value(dev, info, wrqu, extra);
		case 11:
			return itm_get_rx_status(dev, info, wrqu, extra);
		case 12:
			 return itm_get_tx_power_level(dev, info, wrqu, extra);
#endif		
		default:
			return -1;
	}

	return -1;


}

int wpa_supplicant_ioctl(struct net_device *dev, struct iw_point *p)
{
	PRINTK_ITMIW("wpa_supplicant_ioctl fuc \n");
	return -ESRCH;
}

//chenq add 2012-11-14
#define ITM_IOCTL_HOSTAPD (SIOCIWFIRSTPRIV + 28)

//chenq add 2012-11-14
/* ITM_IOCTL_HOSTAPD ioctl() cmd: */
enum {
	ITM_HOSTAPD_FLUSH = 1,
	ITM_HOSTAPD_ADD_STA = 2,
	ITM_HOSTAPD_REMOVE_STA = 3,
	ITM_HOSTAPD_GET_INFO_STA = 4,
	/* REMOVED: PRISM2_HOSTAPD_RESET_TXEXC_STA = 5, */
	ITM_HOSTAPD_GET_WPAIE_STA = 5,
	ITM_SET_ENCRYPTION = 6,
	ITM_GET_ENCRYPTION = 7,
	ITM_HOSTAPD_SET_FLAGS_STA = 8,
	ITM_HOSTAPD_GET_RID = 9,
	ITM_HOSTAPD_SET_RID = 10,
	ITM_HOSTAPD_SET_ASSOC_AP_ADDR = 11,
	ITM_HOSTAPD_SET_GENERIC_ELEMENT = 12,
	ITM_HOSTAPD_MLME = 13,
	ITM_HOSTAPD_SCAN_REQ = 14,
	ITM_HOSTAPD_STA_CLEAR_STATS = 15,
	ITM_HOSTAPD_SET_BEACON=16,
	ITM_HOSTAPD_SET_WPS_BEACON = 17,
	ITM_HOSTAPD_SET_WPS_PROBE_RESP = 18,
	ITM_HOSTAPD_SET_WPS_ASSOC_RESP = 19,
};

#define RSN_SELECTOR_LEN 4
#define WPA_SELECTOR_LEN 4

#define _WPA_IE_ID_	  0xdd
#define _WPA2_IE_ID_  0x30
#define _WAPI_IE_ID_  0x44

UWORD8 RSN_CIPHER_SUITE_NONE[]   = { 0x00, 0x0f, 0xac, 0 };
UWORD8 RSN_CIPHER_SUITE_WEP40[]  = { 0x00, 0x0f, 0xac, 1 };
UWORD8 RSN_CIPHER_SUITE_TKIP[]   = { 0x00, 0x0f, 0xac, 2 };
UWORD8 RSN_CIPHER_SUITE_WRAP[]   = { 0x00, 0x0f, 0xac, 3 };
UWORD8 RSN_CIPHER_SUITE_CCMP[]   = { 0x00, 0x0f, 0xac, 4 };
UWORD8 RSN_CIPHER_SUITE_WEP104[] = { 0x00, 0x0f, 0xac, 5 };

UWORD8 WPA_CIPHER_SUITE_NONE[] = { 0x00, 0x50, 0xf2, 0 };
UWORD8 WPA_CIPHER_SUITE_WEP40[] = { 0x00, 0x50, 0xf2, 1 };
UWORD8 WPA_CIPHER_SUITE_TKIP[] = { 0x00, 0x50, 0xf2, 2 };
UWORD8 WPA_CIPHER_SUITE_WRAP[] = { 0x00, 0x50, 0xf2, 3 };
UWORD8 WPA_CIPHER_SUITE_CCMP[] = { 0x00, 0x50, 0xf2, 4 };
UWORD8 WPA_CIPHER_SUITE_WEP104[] = { 0x00, 0x50, 0xf2, 5 };

#define WPA_CIPHER_NONE 	0
#define WPA_CIPHER_WEP40 	BIT1
#define WPA_CIPHER_WEP104   BIT2
#define WPA_CIPHER_TKIP 	BIT3
#define WPA_CIPHER_CCMP 	BIT4

#define ITM_GET_LE16(a) ((u16) (((a)[1] << 8) | (a)[0]))

#define IEEE_CRYPT_ALG_NAME_LEN 16

typedef struct ieee_param {
	u32 cmd;
	u8 sta_addr[ETH_ALEN];
        union {
		struct {
			u8 name;
			u32 value;
		} wpa_param;
		struct {
			u32 len;
			u8 reserved[32];
			u8 data[0];
		} wpa_ie;
	        struct{
			int command;
    			int reason_code;
		} mlme;
		struct {
			u8 alg[IEEE_CRYPT_ALG_NAME_LEN];
			u8 set_tx;
			u32 err;
			u8 idx;
			u8 seq[8]; /* sequence counter (set: RX, get: TX) */
			u16 key_len;
			u8 key[0];
		} crypt;
//#ifdef CONFIG_AP_MODE
#if 0
		struct {
			u16 aid;
			u16 capability;
			int flags;
			u8 tx_supp_rates[16];		
			struct ieee80211_ht_cap ht_cap;
		} add_sta;
#endif
		struct {
			u8	reserved[2];//for set max_num_sta
			u8	buf[0];
		} bcn_ie;
//#endif

	} u;	   
}ieee_param;

int itm_get_wpa2_cipher_suite(UWORD8 *s)
{
	if (memcmp(s, RSN_CIPHER_SUITE_NONE, RSN_SELECTOR_LEN) == 0)
		return WPA_CIPHER_NONE;
	if (memcmp(s, RSN_CIPHER_SUITE_WEP40, RSN_SELECTOR_LEN) == 0)
		return WPA_CIPHER_WEP40;
	if (memcmp(s, RSN_CIPHER_SUITE_TKIP, RSN_SELECTOR_LEN) == 0)
		return WPA_CIPHER_TKIP;
	if (memcmp(s, RSN_CIPHER_SUITE_CCMP, RSN_SELECTOR_LEN) == 0)
		return WPA_CIPHER_CCMP;
	if (memcmp(s, RSN_CIPHER_SUITE_WEP104, RSN_SELECTOR_LEN) == 0)
		return WPA_CIPHER_WEP104;

	return 0;
}

int itm_parse_wpa2_ie(UWORD8 * rsn_ie, int rsn_ie_len, int * cipher, int *is_8021x)
{
	int i, ret= 0;
	int left, count;
	UWORD8 *pos;
	UWORD8 SUITE_1X[4] = {0x00,0x0f, 0xac, 0x01};
	
	//*is_8021x = 0;
	
	if (rsn_ie_len <= 0) 
	{
		/* No RSN IE - fail silently */
		return -1;
	}

	if ((*rsn_ie!= _WPA2_IE_ID_) || (*(rsn_ie+1) != (u8)(rsn_ie_len - 2)))
	{		
		return -1;
	}
	
	pos = rsn_ie;
	pos += 4;
	left = rsn_ie_len - 4;	

	//group_cipher
	if (left >= RSN_SELECTOR_LEN) 
	{

		*cipher |= itm_get_wpa2_cipher_suite(pos);
		
		pos += RSN_SELECTOR_LEN;
		left -= RSN_SELECTOR_LEN;
		
	} 
	else if (left > 0) 
	{
		PRINTK_ITMIW("%s: ie length mismatch, %u too much", __FUNCTION__, left);
		return -1;
	}

	//pairwise_cipher
	if (left >= 2)
	{		
	    //count = le16_to_cpu(*(u16*)pos);
		count = ITM_GET_LE16(pos);
		pos += 2;
		left -= 2;

		if (count == 0 || left < count * RSN_SELECTOR_LEN) 
		{
			PRINTK_ITMIW("%s: ie count botch (pairwise), "
				  		 "count %u left %u", __FUNCTION__, count, left);
			return -1;
		}
		
		for (i = 0; i < count; i++)
		{			
			*cipher |= itm_get_wpa2_cipher_suite(pos);
			
			pos  += RSN_SELECTOR_LEN;
			left -= RSN_SELECTOR_LEN;
		}

	} 
	else if (left == 1)
	{
		PRINTK_ITMIW("%s: ie too short (for key mgmt)",  __FUNCTION__);
		
		return -1;
	}

	if (left >= 6) 
	{
		pos += 2;
		if (memcmp(pos, SUITE_1X, 4) == 0) 
		{
			PRINTK_ITMIW("%s (): there has 802.1x auth\n", __FUNCTION__);
			*is_8021x = 1;
		}
	}
	
	return ret;
	
}

int itm_get_wpa_cipher_suite(u8 *s)
{
	if (memcmp(s, WPA_CIPHER_SUITE_NONE, WPA_SELECTOR_LEN) == 0)
		return WPA_CIPHER_NONE;
	if (memcmp(s, WPA_CIPHER_SUITE_WEP40, WPA_SELECTOR_LEN) == 0)
		return WPA_CIPHER_WEP40;
	if (memcmp(s, WPA_CIPHER_SUITE_TKIP, WPA_SELECTOR_LEN) == 0)
		return WPA_CIPHER_TKIP;
	if (memcmp(s, WPA_CIPHER_SUITE_CCMP, WPA_SELECTOR_LEN) == 0)
		return WPA_CIPHER_CCMP;
	if (memcmp(s, WPA_CIPHER_SUITE_WEP104, WPA_SELECTOR_LEN) == 0)
		return WPA_CIPHER_WEP104;

	return 0;
}

int itm_parse_wpa_ie(UWORD8 * wpa_ie, int wpa_ie_len, int *cipher, int *is_8021x)
{
	int i, ret=0;
	int left, count;
	UWORD8 *pos;
	UWORD8 SUITE_1X[4] = {0x00, 0x50, 0xf2, 1};
	UWORD8 ITM_WPA_OUI_TYPE[4] = {0x00, 0x50, 0xf2, 1};
	
	if (wpa_ie_len <= 0) 
	{
		/* No WPA IE - fail silently */
		return -1;
	}

	
	if ((*wpa_ie != _WPA_IE_ID_) || (*(wpa_ie+1) != (u8)(wpa_ie_len - 2)) ||
	   (memcmp(wpa_ie+2, ITM_WPA_OUI_TYPE, WPA_SELECTOR_LEN) != 0) )
	{		
		return -1;
	}

	pos = wpa_ie;

	pos += 8;
	left = wpa_ie_len - 8;	


	//group_cipher
	if (left >= WPA_SELECTOR_LEN) {

		*cipher |= itm_get_wpa_cipher_suite(pos);
		
		pos += WPA_SELECTOR_LEN;
		left -= WPA_SELECTOR_LEN;
		
	} 
	else if (left > 0)
	{
		PRINTK_ITMIW("%s: ie length mismatch, %u too much", __FUNCTION__, left);
		
		return -1;
	}


	//pairwise_cipher
	if (left >= 2)
	{
		count = ITM_GET_LE16(pos);
		pos += 2;
		left -= 2;
		
		if (count == 0 || left < count * WPA_SELECTOR_LEN) 
		{
			PRINTK_ITMIW("%s: ie count botch (pairwise), "
				   		"count %u left %u", __FUNCTION__, count, left);
			return -1;
		}
		
		for (i = 0; i < count; i++)
		{
			*cipher |= itm_get_wpa_cipher_suite(pos);
			
			pos += WPA_SELECTOR_LEN;
			left -= WPA_SELECTOR_LEN;
		}
		
	} 
	else if (left == 1)
	{
		PRINTK_ITMIW("%s: ie too short (for key mgmt)",   __FUNCTION__);
		return -1;
	}

	if (left >= 6) {
		pos += 2;
		if (memcmp(pos, SUITE_1X, 4) == 0) 
		{
			PRINTK_ITMIW("%s : there has 802.1x auth\n", __FUNCTION__);
			*is_8021x = 1;
		}
	}
	
	return ret;
	
}

UWORD8 * itm_get_ie(u8 * pbuf, int index, int * len, int limit)
{
	int tmp,i;
	UWORD8 * p;

	if (limit < 1)
	{
		return NULL;
	}

	p = pbuf;
	i = 0;
	*len = 0;
	while(1)
	{
		if (*p == index)
		{
			*len = *(p + 1);
			return (p);
		}
		else
		{
			tmp = *(p + 1);
			p += (tmp + 2);
			i += (tmp + 2);
		}
		if (i >= limit)
			break;
	}
	
	return NULL;
}

//chenq add for trout wifi cfg 2013-01-10
extern trout_wifi_info_t trout_wifi_info;

int itm_set_ap_info(UWORD8 * pbuf,  int len)
{
	UWORD8 *p = NULL;

	UWORD32 ie_len = 0;
	int cipher   = 0;
	int use_wpa  = 0;
	int use_wpa2 = 0;
	
	UWORD8 WPA_OUI1[4] = {0x00, 0x50, 0xf2, 0x01};
	//UWORD8 wps_oui[4]  = {0x00, 0x50, 0xf2, 0x04};
	//UWORD8 WMM_PARA_IE[] = {0x00, 0x50, 0xf2, 0x02, 0x01, 0x01};	

	UWORD8 *ie = pbuf;
	int is_8021x = 0;
	int MAX_IE_SZ = 768;
	
	int _BEACON_IE_OFFSET_ = 12;
	int _SSID_IE_ = 0;
	int _DSSET_IE_ = 3;

	UWORD8 * essid     = NULL;
	UWORD8   essid_len = 0;
	UWORD8   channel   = 0;
	UWORD8   enc_type  = 0;
	

	/* SSID */ //will do
	/* Supported rates */ //not do
	/* DS Params */ //not do
	/* WLAN_EID_COUNTRY */ //not do
	/* ERP Information element */ //not do
	/* Extended supported rates */ //not do
	/* WPA/WPA2 */ //will do
	/* Wi-Fi Wireless Multimedia Extensions */ //not do
	/* ht_capab, ht_oper */ //not do
	/* WPS IE */ //not do

	PRINTK_ITMIW("%s, len=%d\n", __FUNCTION__, len);


	if(len>MAX_IE_SZ)
		return -1;

	//SSID
	p = itm_get_ie(ie + _BEACON_IE_OFFSET_, _SSID_IE_, &ie_len, (len -_BEACON_IE_OFFSET_));
	if(p && ie_len>0)
	{
		essid     = (p + 2);
		essid_len = ie_len;
	}	

	//chnnel
	channel = 0;
	p = itm_get_ie(ie + _BEACON_IE_OFFSET_, _DSSET_IE_, &ie_len, (len - _BEACON_IE_OFFSET_));
	if(p && ie_len>0)
		channel = *(p + 2);
	

	//wpa2
	p = itm_get_ie(ie + _BEACON_IE_OFFSET_, _WPA2_IE_ID_, &ie_len, (len - _BEACON_IE_OFFSET_));		
	if(p && ie_len>0)
	{
		if(itm_parse_wpa2_ie(p, ie_len+2, &cipher, &is_8021x) == 0)
		{
			use_wpa2 = 1;
		}	
	}

	//wpa
	ie_len = 0;
	for (p = ie + _BEACON_IE_OFFSET_; ;p += (ie_len + 2))
	{
		p = itm_get_ie(p, _WPA_IE_ID_, &ie_len, (len - _BEACON_IE_OFFSET_ - (ie_len + 2)));		
		if ((p) && (memcmp(p+2, WPA_OUI1, 4) == 0))
		{
			if(itm_parse_wpa_ie(p, ie_len+2, &cipher, &is_8021x) == 0)
			{
				use_wpa = 1;
			}

			break;
			
		}
			
		if ((p == NULL) || (ie_len == 0))
		{
				break;
		}
		
	}

	if(is_8021x)
	{
		PRINTK_ITMIW("itm ap mode not support 802.1x type\n");
		return -1;
	}

	if( (cipher & BIT1) && (cipher & BIT2) )
	{
		PRINTK_ITMIW("wep40 and wep104 can not valid at same time\n");
		return -1;
	}

	if(cipher & WPA_CIPHER_WEP40)
	{
		enc_type |= BIT1;
	}

	if(cipher & WPA_CIPHER_WEP104)
	{
		enc_type |= BIT1 | BIT2;
	}

	if(cipher & WPA_CIPHER_TKIP)
	{
		enc_type |= BIT6;
	}

	if(cipher & WPA_CIPHER_CCMP)
	{
		enc_type |= BIT5;
	}

	if( use_wpa == 1 )
	{
		enc_type |= BIT3;
	}

	if( use_wpa2 == 1 )
	{
		enc_type |= BIT4;
	}

	if( ((cipher & WPA_CIPHER_WEP40) || (cipher & WPA_CIPHER_WEP104))
	  &&((cipher & WPA_CIPHER_TKIP)  || (cipher & WPA_CIPHER_CCMP)) )
	{
		enc_type |= BIT7;
	}

	if( enc_type != WPA_CIPHER_NONE )
	{
		enc_type |= BIT0;
	}

	if( itm_set_Encryption_Type(enc_type) < 0 )
	{
		return -1;
	}

	#ifdef BSS_ACCESS_POINT_MODE
	if(trout_wifi_info.ap_channel != 0)
	{
		channel = trout_wifi_info.ap_channel;
	}
	#endif

	if( itm_set_channel(channel) < 0 )
	{
		return -1;
	}


	if( itm_set_essid(essid,essid_len) < 0 )
	{
		return -1;
	}

	return 0;

}

static int itm_set_beacon(struct ieee_param *param, int len)
{
	int ret=0;
	unsigned char *pbuf = param->u.bcn_ie.buf;

	PRINTK_ITMIW("%s, len=%d\n", __FUNCTION__, len);

	#ifdef BSS_ACCESS_POINT_MODE

	if(itm_set_ap_info(pbuf,  (len-12-2)) == 0)// 12 = param header, 2:no packed
		ret = 0;
	else
		ret = -EINVAL;

	#else

		ret = -EINVAL;

	#endif

	return ret;
	
}

/*leon liu exported itm_set_encryption to global*/
int itm_set_encryption(struct ieee_param *param, u32 param_len)
{
	int ret = 0;
	//u32 wep_key_idx, wep_key_len,wep_total_len;
	/*struct sta_info*/void *psta = NULL,*pbcmc_sta = NULL;	

	ITMIW_FUNC_ENTER;

	param->u.crypt.err = 0;
	param->u.crypt.alg[IEEE_CRYPT_ALG_NAME_LEN - 1] = '\0';

	#if 0
	//sizeof(struct ieee_param) = 64 bytes;
	//if (param_len !=  (u32) ((u8 *) param->u.crypt.key - (u8 *) param) + param->u.crypt.key_len)
	if (param_len !=  sizeof(struct ieee_param) + param->u.crypt.key_len)
	{
		PRINTK_ITMIW("chenq debug 0\n");
		ret =  -EINVAL;
		goto exit;
	}
	#endif

	if (param->sta_addr[0] == 0xff && param->sta_addr[1] == 0xff &&
	    param->sta_addr[2] == 0xff && param->sta_addr[3] == 0xff &&
	    param->sta_addr[4] == 0xff && param->sta_addr[5] == 0xff) 
	{
		if (param->u.crypt.idx >= 4)
		{
			PRINTK_ITMIW("chenq debug 1\n");
			ret = -EINVAL;
			goto exit;
		}	
	}
	else 
	{		
		//psta = rtw_get_stainfo(pstapriv, param->sta_addr);
		if(!psta)
		{
			//ret = -EINVAL;
			PRINTK_ITMIW("itm_set_encryption(), sta has already been removed or never been added\n");
			goto exit;
		}			
	}

	if (strcmp(param->u.crypt.alg, "none") == 0 && (psta==NULL))
	{
		//todo:clear default encryption keys

		PRINTK_ITMIW("clear default encryption keys, keyid=%d\n", param->u.crypt.idx);

		if( itm_set_Auth_Type( (UWORD8)(ITM_OPEN_SYSTEM & 0xFF) ) < 0 )
		{
			ret = -EINVAL;
		}

		if( itm_set_Encryption_Type(No_ENCRYPTION) < 0 )
		{
			ret = -EINVAL;
		}
		
		goto exit;
	}	

		if(strcmp(param->u.crypt.alg, "TKIP") == 0)
		{
				UWORD8 * key = param->u.crypt.key;

				if( itm_set_Auth_Type( (UWORD8)(ITM_OPEN_SYSTEM & 0xFF) ) < 0 )
				{
						return -EINVAL;
				}

				if( itm_set_Encryption_Type( (UWORD8)WPA_TKIP_PSK ) < 0)
				{
						return -EINVAL;
				}


				if( itm_set_psk(key,strlen(key)) < 0)
				{
						return -EINVAL;
				}

				PRINTK_ITMIW("%s, set group_key, TKP\n", __FUNCTION__);

		#if 0
		if( itm_set_Auth_Type( (UWORD8)(ITM_OPEN_SYSTEM & 0xFF) ) < 0 )
		{
			return -EINVAL;
		}
		
        if( itm_set_Encryption_Type( (UWORD8)WPA2_AES_PSK ) < 0)
        {
            return -EINVAL;
        }

		PRINTK_ITMIW("key len = %d\n",param->u.crypt.key_len);

		for(i=0;i<32;i++)
        {
           g_psk_value[i] = (16*hex_look_up(key[j]))+hex_look_up(key[j+1]);
           j+=2;
        }
		#endif
		
	}
		else if(strcmp(param->u.crypt.alg, "CCMP") == 0)
		{
				UWORD8 * key = param->u.crypt.key;

				if( itm_set_Auth_Type( (UWORD8)(ITM_OPEN_SYSTEM & 0xFF) ) < 0 )
				{
						return -EINVAL;
				}

				if( itm_set_Encryption_Type( (UWORD8)WPA2_AES_PSK ) < 0)
				{
						return -EINVAL;
				}

				if( itm_set_psk(key,strlen(key)) < 0)
				{
						return -EINVAL;
				}
		}
		else if(strcmp(param->u.crypt.alg, "itm_set_ccmp") == 0)
	{
		UWORD8 * key = param->u.crypt.key;

		if( itm_set_Auth_Type( (UWORD8)(ITM_OPEN_SYSTEM & 0xFF) ) < 0 )
		{
			return -EINVAL;
		}
		
        if( itm_set_Encryption_Type( (UWORD8)WPA2_AES_PSK ) < 0)
        {
            return -EINVAL;
        }

		if( itm_set_psk_value(key,strlen(key)) < 0)
		{
			return -EINVAL;
		}
	}
		else if(strcmp(param->u.crypt.alg,"itm_set_tkip") == 0)
		{
				UWORD8 * key = param->u.crypt.key;

				if( itm_set_Auth_Type( (UWORD8)(ITM_OPEN_SYSTEM & 0xFF) ) < 0 )
				{
						return -EINVAL;
				}

				if( itm_set_Encryption_Type( (UWORD8)WPA_TKIP_PSK ) < 0)
				{
						return -EINVAL;
				}

				if( itm_set_psk_value(key,strlen(key)) < 0)
				{
						return -EINVAL;
				}
		}

exit:

	ITMIW_FUNC_EXIT;
	return ret;
	
}

int itm_hostapd_ioctl(struct iw_point *p)
{
	struct ieee_param *param;
	int ret=0;

	PRINTK_ITMIW("itm_hostapd_ioctl fuc \n");

	param = (struct ieee_param *)kmalloc(p->length,GFP_KERNEL);
	if (param == NULL)
	{
		ret = -ENOMEM;
		goto out;
	}

	if (copy_from_user((UWORD8 *)param, p->pointer, p->length))
	{
		kfree(param);
		ret = -EFAULT;
		goto out;
	}

	switch (param->cmd) 
	{	
		case ITM_HOSTAPD_FLUSH:

			PRINTK_ITMIW( "ITM_HOSTAPD_FLUSH enter\n" );
			//ret = rtw_hostapd_sta_flush(dev);
			PRINTK_ITMIW( "ITM_HOSTAPD_FLUSH exit\n" );
			ret = -EOPNOTSUPP;
			break;
	
		case ITM_HOSTAPD_ADD_STA:	

			PRINTK_ITMIW( "ITM_HOSTAPD_ADD_STA enter\n" );
			//ret = rtw_add_sta(dev, param);					
			PRINTK_ITMIW( "ITM_HOSTAPD_ADD_STA exit\n" );
			ret = -EOPNOTSUPP;
			break;

		case ITM_HOSTAPD_REMOVE_STA:

			PRINTK_ITMIW( "ITM_HOSTAPD_REMOVE_STA enter\n" );
			//ret = rtw_del_sta(dev, param);
			PRINTK_ITMIW( "ITM_HOSTAPD_REMOVE_STA exit\n" );
			ret = -EOPNOTSUPP;
			break;
	
		case ITM_HOSTAPD_SET_BEACON:

			PRINTK_ITMIW( "ITM_HOSTAPD_SET_BEACON enter\n" );
			ret = itm_set_beacon( param, p->length);
			PRINTK_ITMIW( "ITM_HOSTAPD_SET_BEACON exit\n" );
			break;
			
		case ITM_SET_ENCRYPTION:

			PRINTK_ITMIW( "ITM_SET_ENCRYPTION enter\n" );
			ret = itm_set_encryption(param, p->length);
			PRINTK_ITMIW( "ITM_SET_ENCRYPTION exit\n" );
			break;
			
		case ITM_HOSTAPD_GET_WPAIE_STA:

			PRINTK_ITMIW( "ITM_HOSTAPD_GET_WPAIE_STA enter\n" );
			//ret = rtw_get_sta_wpaie(dev, param);
			PRINTK_ITMIW( "ITM_HOSTAPD_GET_WPAIE_STA exit\n" );
			ret = -EOPNOTSUPP;
			break;
			
		case ITM_HOSTAPD_SET_WPS_BEACON:

			PRINTK_ITMIW( "ITM_HOSTAPD_SET_WPS_BEACON enter\n" );
			//ret = rtw_set_wps_beacon(dev, param, p->length);
			PRINTK_ITMIW( "ITM_HOSTAPD_SET_WPS_BEACON exit\n" );
			ret = -EOPNOTSUPP;
			break;

		case ITM_HOSTAPD_SET_WPS_PROBE_RESP:

			PRINTK_ITMIW( "ITM_HOSTAPD_SET_WPS_PROBE_RESP enter\n" );
			//ret = rtw_set_wps_probe_resp(dev, param, p->length);
			PRINTK_ITMIW( "ITM_HOSTAPD_SET_WPS_PROBE_RESP exit\n" );
			ret = -EOPNOTSUPP;
	 		break;
			
		case ITM_HOSTAPD_SET_WPS_ASSOC_RESP:

			PRINTK_ITMIW( "ITM_HOSTAPD_SET_WPS_ASSOC_RESP enter\n" );
			//ret = rtw_set_wps_assoc_resp(dev, param, p->length);
			PRINTK_ITMIW( "ITM_HOSTAPD_SET_WPS_ASSOC_RESP exit\n" );
			ret = -EOPNOTSUPP;
	 		break;
			
		default:
			PRINTK_ITMIW( "Unknown hostapd request: %d\n", param->cmd );
			ret = -EOPNOTSUPP;
			break;
		
	}

	if (ret == 0 && copy_to_user(p->pointer, param, p->length))
		ret = -EFAULT;


	kfree(param);
	
out:
		
	return ret;
}

struct iw_statistics *
itm_get_wireless_stats(struct net_device *dev)
{
	int ret = 0;
	UWORD8 * host_req  = NULL;
	UWORD8 * trout_rsp = NULL;
	UWORD16  trout_rsp_len = 0;
	WORD8  rssi_db = 0;
	WORD8  snr_db  = 0;
	int signal, noise, snr;

	PRINTK_ITMIW("itm_get_wireless_stats fuc \n");

#ifdef IBSS_BSS_STATION_MODE
	if( (get_mac_state() != ENABLED) && (g_keep_connection != BTRUE) )
    {
    	PRINTK_ITMIW("get_mac_state() = %d ,will not get stats \n",get_mac_state());
		memset(&g_itm_iw_statistics,0x00,sizeof(g_itm_iw_statistics));
        return &g_itm_iw_statistics;
    }
#endif

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if(host_req == NULL)
	{
		return NULL;
	}

	host_req[0] = WID_RSSI & 0xFF;
	host_req[1] = (WID_RSSI & 0xFF00) >> 8;
	trout_rsp = config_if_for_iw(&g_mac,host_req,2,'Q',&trout_rsp_len);

	if( trout_rsp == NULL )
	{
		PRINTK_ITMIW("trout_rsp == NULL\n");
		return NULL;
	}

	trout_rsp_len -= MSG_HDR_LEN;
	if((ret = get_trour_rsp_data((UWORD8*)(&rssi_db),sizeof(UWORD8),&trout_rsp[MSG_HDR_LEN],
					      trout_rsp_len,WID_CHAR))  ==  0)
	{	
		pkt_mem_free(trout_rsp);
		PRINTK_ITMIW("get_trour_rsp_data == fail\n");
		return NULL;
	}
	pkt_mem_free(trout_rsp);

	
	host_req[0] = WID_SNR & 0xFF;
	host_req[1] = (WID_SNR & 0xFF00) >> 8;
	trout_rsp = config_if_for_iw(&g_mac,host_req,2,'Q',&trout_rsp_len);

	if( trout_rsp == NULL )
	{
		PRINTK_ITMIW("trout_rsp == NULL\n");
		return NULL;
	}

	trout_rsp_len -= MSG_HDR_LEN;
	if((ret = get_trour_rsp_data((UWORD8*)(&snr_db),sizeof(UWORD8),&trout_rsp[MSG_HDR_LEN],
					      trout_rsp_len,WID_CHAR))  ==  0)
	{	
		pkt_mem_free(trout_rsp);
		PRINTK_ITMIW("get_trour_rsp_data == fail\n");
		return NULL;
	}
	pkt_mem_free(trout_rsp);

	signal = -(0xFF - rssi_db);
    /* Clip range of snr */
    snr    = (snr_db > 0) ? snr_db : 0; /* In dB relative, from 0 - 255 */
    snr    = (snr < 255) ? snr : 255;
    noise  = signal - snr;

    /* Clip range of signal */
    signal = (signal < 63) ? signal : 63;
    signal = (signal > -192) ? signal : -192;

    /* Clip range of noise */
    noise = (noise < 63) ? noise : 63;
    noise = (noise > -192) ? noise : -192;

    /* Make u8 */
    signal = ( signal < 0 ) ? signal + 0x100 : signal;
    noise = ( noise < 0 ) ? noise + 0x100 : noise;

    g_itm_iw_statistics.qual.level   = (UWORD8)signal; /* -192 : 63 */
    g_itm_iw_statistics.qual.noise   = (UWORD8)noise;  /* -192 : 63 */
    g_itm_iw_statistics.qual.qual    = snr;         /* 0 : 255 */
    g_itm_iw_statistics.qual.updated = 0;

	PRINTK_ITMIW("rssi = %d,snr = %d\n",
		g_itm_iw_statistics.qual.level,g_itm_iw_statistics.qual.qual);

#if WIRELESS_EXT > 16
    g_itm_iw_statistics.qual.updated |= IW_QUAL_LEVEL_UPDATED |
                                        IW_QUAL_NOISE_UPDATED |
                                        IW_QUAL_QUAL_UPDATED;
#if WIRELESS_EXT > 18
    g_itm_iw_statistics.qual.updated |= IW_QUAL_DBM;
#endif
#endif

#ifdef IBSS_BSS_STATION_MODE
	g_itm_iw_statistics.discard.nwid = g_wrong_bssid;/* Rx : Wrong nwid/essid */
	g_itm_iw_statistics.discard.code = g_unable2decode;/* Rx : Unable to code/decode (WEP) */
	g_itm_iw_statistics.discard.fragment = g_perform_mac_reassembly_err;/* Rx : Can't perform MAC reassembly */
	g_itm_iw_statistics.discard.retries = g_tx_retries;	/* Tx : Max MAC retries num reached */
	g_itm_iw_statistics.discard.misc = g_rx_misc;		/* Others cases */
	g_itm_iw_statistics.miss.beacon = g_missed_beacons;		/* Missed beacons/superframe */
#endif

    return &g_itm_iw_statistics;
} /* unifi_get_wireless_stats() */

#ifdef CONFIG_TROUT_WEXT
static const struct iw_priv_args itm_private_args[] = {
    /*{ cmd(Number of the ioctl to issue),
        set_args(Type and number of args),
        get_args(Type and number of args),
        name(Name of the extension) 
      }*/

	//chenq add for IC test
	{ SIOCIWSICTEST, IW_PRIV_TYPE_BYTE | IW_PRIV_SIZE_FIXED | 256,
        IW_PRIV_TYPE_NONE, "itm_ICtest" },


	//chenq add for debug
	{ SIOCIWG_FORDEBUG, IW_PRIV_TYPE_BYTE | (IW_ESSID_MAX_SIZE+1),
        IW_PRIV_TYPE_BYTE | IW_PRIV_SIZE_FIXED | (IW_ESSID_MAX_SIZE+1),"itm_gfordebug" },

	//chenq add for debug
	{ SIOCIWS_FORDEBUG, IW_PRIV_TYPE_BYTE | (IW_ESSID_MAX_SIZE+1),
        IW_PRIV_TYPE_NONE, "itm_sfordebug" },


    //chenq add for WID
    { SIOCIWS_FORWID, IW_PRIV_TYPE_BYTE | sizeof(itm_wid_config_t), IW_PRIV_TYPE_NONE, "itm_sforwid"},


	//chenq add for iwcossnfig trout
	{ SIOCIWS_IWCONFIG_TROUT, IW_PRIV_TYPE_BYTE | IW_PRIV_SIZE_FIXED | 128, IW_PRIV_TYPE_NONE, "itm_siw_trout"},
	
	{ SIOCIWG_IWCONFIG_TROUT, IW_PRIV_TYPE_NONE, IW_PRIV_TYPE_BYTE | 128, "itm_giw_trout"},

};

static const iw_handler itm_handler[] =
{
    (iw_handler) NULL,                    /* SIOCSIWCOMMIT */
    (iw_handler) itm_giwname,             /* SIOCGIWNAME   */
    (iw_handler) dummy,        			  /* SIOCSIWNWID   */
    (iw_handler) dummy,                   /* SIOCGIWNWID   */
    (iw_handler) itm_siwfreq,             /* SIOCSIWFREQ   */
    (iw_handler) itm_giwfreq,             /* SIOCGIWFREQ   */
    (iw_handler) itm_siwmode,             /* SIOCSIWMODE   */
    (iw_handler) itm_giwmode,             /* SIOCGIWMODE   */
    (iw_handler) dummy,                   /* SIOCSIWSENS   */
    (iw_handler) itm_giwsens,             /* SIOCGIWSENS   */
    (iw_handler) dummy,                   /* SIOCSIWRANGE  */
    (iw_handler) itm_giwrange,            /* SIOCGIWRANGE  */
    (iw_handler) itm_siwpriv,             /* SIOCSIWPRIV   */
    (iw_handler) NULL,                    /* SIOCGIWPRIV   */
    (iw_handler) NULL,                    /* SIOCSIWSTATS  */
    (iw_handler) NULL,                    /* SIOCGIWSTATS  */
    (iw_handler) dummy,                   /* SIOCSIWSPY    */
    (iw_handler) dummy,                   /* SIOCGIWSPY    */
    (iw_handler) NULL,                    /* SIOCSIWTHRSPY */
    (iw_handler) NULL,                    /* SIOCGIWTHRSPY */
    (iw_handler) itm_siwap,               /* SIOCSIWAP     */
    (iw_handler) itm_giwap,	              /* SIOCGIWAP  get access point MAC addresses */

    /* WPA : IEEE 802.11 MLME requests */
    (iw_handler)itm_siwmlme,              /* SIOCSIWMLME, request MLME operation */

    (iw_handler) dummy,                   /* SIOCGIWAPLIST */
   
    (iw_handler) itm_siwscan,             /* SIOCSIWSCAN trigger scanning (list cells)*/
    (iw_handler) itm_giwscan,             /* SIOCGIWSCAN  */

    (iw_handler) itm_siwessid,            /* SIOCSIWESSID  */
    (iw_handler) itm_giwessid,		      /* SIOCGIWESSID  get ESSID*/
    (iw_handler) dummy,                   /* SIOCSIWNICKN  */
    (iw_handler) itm_giwnick,             /* SIOCGIWNICKN  */
    (iw_handler) NULL,                    /* -- hole --    */
    (iw_handler) NULL,                    /* -- hole --    */
    (iw_handler)itm_siwrate,              /* SIOCSIWRATE   */
    (iw_handler)itm_giwrate,              /* SIOCGIWRATE   */
    (iw_handler)itm_siwrts,               /* SIOCSIWRTS    */
    (iw_handler)itm_giwrts,               /* SIOCGIWRTS    */
    (iw_handler)itm_siwfrag,              /* SIOCSIWFRAG   */
    (iw_handler)itm_giwfrag,              /* SIOCGIWFRAG   */
    (iw_handler) dummy,                   /* SIOCSIWTXPOW  */
    (iw_handler) dummy,                   /* SIOCGIWTXPOW  */
    (iw_handler) dummy,                   /* SIOCSIWRETRY  */
    (iw_handler) itm_giwretry,            /* SIOCGIWRETRY  */
    (iw_handler)itm_siwencode,            /* SIOCSIWENCODE */
    (iw_handler)itm_giwencode,            /* SIOCGIWENCODE */
    (iw_handler)itm_siwpower,             /* SIOCSIWPOWER  */
    (iw_handler)itm_giwpower,             /* SIOCGIWPOWER  */

    (iw_handler) NULL,                    /* -- hole -- */
    (iw_handler) NULL,                    /* -- hole -- */
    /* WPA : Generic IEEE 802.11 informatiom element (e.g., for WPA/RSN/WMM). */
    (iw_handler)itm_siwgenie,             /* SIOCSIWGENIE */      /* set generic IE */
    (iw_handler)NULL,                     /* SIOCGIWGENIE */      /* get generic IE */
    /* WPA : Authentication mode parameters */
    (iw_handler)itm_siwauth,              /* SIOCSIWAUTH */       /* set authentication mode params */
    (iw_handler)NULL,                     /* SIOCGIWAUTH */       /* get authentication mode params */
    /* WPA : Extended version of encoding configuration  */
    (iw_handler)itm_siwencodeext,         /* SIOCSIWENCODEEXT */  /* set encoding token & mode */
    (iw_handler)NULL,                     /* SIOCGIWENCODEEXT */  /* get encoding token & mode */
    /* WPA2 : PMKSA cache management */
    (iw_handler)itm_siwpmksa,             /* SIOCSIWPMKSA */      /* PMKSA cache operation */
    (iw_handler) NULL,                    /* -- hole --   */

};


static const iw_handler itm_private_handler[] =
{
    (iw_handler) itm_ICtest,/* SIOCIWFIRSTPRIV + 0*/
    (iw_handler) NULL,/* SIOCIWFIRSTPRIV + 1*/
    (iw_handler) itm_sfordebug,/* SIOCIWFIRSTPRIV + 2*/
    (iw_handler) itm_gfordebug,/* SIOCIWFIRSTPRIV + 3*/
 
    (iw_handler) itm_sforwid,/* SIOCIWFIRSTPRIV + 4*/
    (iw_handler) NULL,/* SIOCIWFIRSTPRIV + 5*/
    (iw_handler) itm_siw_trout,/* SIOCIWFIRSTPRIV + 6*/
 
    (iw_handler) itm_giw_trout,/* SIOCIWFIRSTPRIV + 7*/
    (iw_handler) NULL,/* SIOCIWFIRSTPRIV + 8*/

	(iw_handler) NULL,/* SIOCIWFIRSTPRIV + 9 */
    (iw_handler) NULL,/* SIOCIWFIRSTPRIV + 10*/
    (iw_handler) NULL,/* SIOCIWFIRSTPRIV + 11*/
    (iw_handler) NULL,/* SIOCIWFIRSTPRIV + 12*/
    (iw_handler) NULL,/* SIOCIWFIRSTPRIV + 13*/
    (iw_handler) NULL,/* SIOCIWFIRSTPRIV + 14*/
    (iw_handler) NULL,/* SIOCIWFIRSTPRIV + 15*/
    (iw_handler) NULL,/* SIOCIWFIRSTPRIV + 16*/
    (iw_handler) NULL,/* SIOCIWFIRSTPRIV + 17*/
    (iw_handler) NULL,/* SIOCIWFIRSTPRIV + 18*/
    (iw_handler) NULL,/* SIOCIWFIRSTPRIV + 19*/
    (iw_handler) NULL,/* SIOCIWFIRSTPRIV + 20*/
 
	(iw_handler) NULL,/* SIOCIWFIRSTPRIV + 21*/
	(iw_handler) NULL,/* SIOCIWFIRSTPRIV + 22*/
	(iw_handler) NULL,/* SIOCIWFIRSTPRIV + 23*/
 

	//chenq add for IC test
    (iw_handler)NULL,//itm_ICtest,//24
	(iw_handler)NULL,//25

	

	(iw_handler)NULL,//itm_siw_trout,//26
	(iw_handler)NULL,//itm_giw_trout,//27
	
    (iw_handler)NULL,//itm_sfordebug, //SIOCIWS_FORDUMY 28
    (iw_handler)NULL,//itm_gfordebug, //SIOCIWG_FORDUMY 29
    (iw_handler)NULL,//itm_sforwid,//SIOCIWS_FORWID 30
};


struct iw_handler_def itm_iw_handler_def =
{
    .num_standard         =  sizeof(itm_handler) / sizeof(iw_handler),
    .num_private           =   sizeof(itm_private_handler) / sizeof(iw_handler),
    .num_private_args   =   sizeof(itm_private_args) / sizeof(struct iw_priv_args),
    .standard                =   (iw_handler *) itm_handler,
    .private                   =  (iw_handler *) itm_private_handler,
    .private_args          =   (struct iw_priv_args *) itm_private_args,
#if IW_HANDLER_VERSION >= 6
    .get_wireless_stats = itm_get_wireless_stats,
#endif
};
#endif
