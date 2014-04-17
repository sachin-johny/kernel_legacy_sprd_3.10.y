#ifdef CONFIG_CFG80211

#include "trout_cfg80211.h"

/* TODO: 
 * 1.modify printk to TROUT_DBGx
 * 2.consider lock for g_itm_config_buf
 */

/*#define NULL_DEBUG*/
#ifndef NULL_DEBUG

/*leon liu added scan competion on 2013-07-12*/
DECLARE_COMPLETION(scan_completion);
#define SCAN_COMPLETION_TIMEOUT		(3 * HZ)

#ifdef MAC_WAPI_SUPP
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

extern int itm_set_wapimode(int mode);
extern int iwprivswapikey(wapi_key_t * inKey);
#endif
//----------Debug codes----------
static inline void print_mac(u8 *mac)
{
	int i;

	for (i = 0; i < 5; i++)
	{
		printk("%02X:", mac[i]);
	}

	printk("%02X ", mac[5]);
}

//----------Trou configuration macros and routines----------
//Special CIPHER for ITM passed down by hostapd
#define WLAN_CIPHER_SUITE_ITM_CCMP	0x1234ABCD
#define WLAN_CIPHER_SUITE_ITM_TKIP	0x1234ABCE

typedef struct
{
	UWORD8       rate_index;
	UWORD8       rate;
	UWORD8       point5;
} rate_table_t;

#define CHECK_MAC_RESET_IN_CFG80211_HANDLER do{\
	if( (BOOL_T)atomic_read(&g_mac_reset_done) == BFALSE )\
	{\
		TROUT_DBG3("%s not do,in reset process\n",__func__);\
		return -EBUSY;\
	}\
}while(0)

extern UWORD8 g_wpa_rsn_ie_buf_3[MAX_SITES_FOR_SCAN][512];
extern UWORD8 g_psk_value[40];
extern int itm_set_ap_info(UWORD8 * pbuf,  int len);
extern int itm_add_wpa_wpa2_value(UWORD8 * s_addr, UWORD8 * key_data,UWORD8 key_len,UWORD8 key_type, UWORD8 key_id,UWORD8 * key_rsc);
extern int itm_add_wep_key(UWORD8 * key,int key_len,int key_index);
extern int itm_set_encryption(struct ieee_param *param, u32 param_len);
extern int itm_set_wep_key_index(int index);
int itm_get_essid(UWORD8 * essid,int essid_len);
int itm_get_devmac(UWORD8 * dev_mac);
#ifdef NMAC_1X1_MODE
extern rate_table_t rate_table[20];
#else
extern rate_table_t rate_table[28];
#endif

static int cur_cipher;

/*junbinwang add wps 20130812*/
#define WPS_IE_BUFF_LEN (255 + 2) //body length + length of len field + length of elemID field
/* WPS fields */
#define WPS_IE_ID 221
#define WPS_IE_OUI_BYTE0 0x00
#define WPS_IE_OUI_BYTE1 0x50
#define WPS_IE_OUI_BYTE2 0xF2
#define WPS_IE_OUI_TYPE 0x04
typedef struct
{
        WORD32 needwps;
        WORD32 needWPSlen;
        UWORD8 wpsIe[WPS_IE_BUFF_LEN];
}TROUT_WPS_INFO;

static WORD32 wps_sec_type_flag;
static TROUT_WPS_INFO probe_req_wps_ie;
static TROUT_WPS_INFO asoc_req_wps_ie;

WORD32 trout_is_probe_req_wps_ie(void)
{
        return probe_req_wps_ie.needwps;
}

void trout_set_probe_req_wps_ie(WORD32 flag)
{
       probe_req_wps_ie.needwps = flag;
       return;
}

void trout_save_probe_req_wps_ie(UWORD8* wpsie, WORD32 len)
{
       if(NULL == wpsie || len == 0)
                return;
       probe_req_wps_ie.needWPSlen = len;
       memset(probe_req_wps_ie.wpsIe, 0x00, WPS_IE_BUFF_LEN);
       memcpy(probe_req_wps_ie.wpsIe, wpsie, len);
       return;
}

void trout_clear_probe_req_wps_ie(void)
{
      memset(&probe_req_wps_ie, 0x00, sizeof(TROUT_WPS_INFO));
}

UWORD8 * trout_get_probe_req_wps_ie_addr(void)
{
      return probe_req_wps_ie.wpsIe;
}

WORD32 trout_get_probe_req_wps_ie_len(void)
{
       return probe_req_wps_ie.needWPSlen;
}

int trout_is_asoc_req_wps_ie(void)
{
      return asoc_req_wps_ie.needwps;
}
void trout_set_asoc_req_wps_ie(WORD32 flag)
{
      asoc_req_wps_ie.needwps = flag;
      return;
}

void trout_save_asoc_req_wps_ie(UWORD8* wpsie, WORD32 len)
{
       if(NULL == wpsie || len == 0)
              return;

      asoc_req_wps_ie.needWPSlen = len;
      memset(asoc_req_wps_ie.wpsIe, 0x00, WPS_IE_BUFF_LEN);
      memcpy(asoc_req_wps_ie.wpsIe, wpsie, len);
      return;
}

UWORD8 * trout_get_asoc_req_wps_ie_addr(void)
{
      return asoc_req_wps_ie.wpsIe;
}

WORD32 trout_get_asoc_req_wps_ie_len(void)
{
      return asoc_req_wps_ie.needWPSlen;
}

void trout_clear_asoc_req_wps_ie(void)
{
      memset(&asoc_req_wps_ie, 0x00, sizeof(TROUT_WPS_INFO));
}

WORD32 trout_is_wps_sec_type_flag(void)
{
       return wps_sec_type_flag;
}

void trout_set_wps_sec_type_flag(WORD32 flag)
{
      wps_sec_type_flag = flag;
      return;
}

BOOL_T trout_find_wps_ie(UWORD8 * ie, size_t ie_len, UWORD8 * wps_ie, UWORD16 * wps_ie_len)
{
       UWORD16 index = 0;
       if (NULL == ie || ie_len <=0 || NULL == wps_ie || NULL == wps_ie_len)
       {
               return BFALSE;
       }

       while (index < ie_len)
       {
             if (WPS_IE_ID == ie[index])
             {
                   *wps_ie_len = ie[index + 1];
                    //ie_len >= wps_ie_len: at least one IE.
                    if (ie_len >= *wps_ie_len && WPS_IE_OUI_BYTE0 == ie[index + 2]
                          && WPS_IE_OUI_BYTE1 == ie[index + 3] && WPS_IE_OUI_BYTE2 == ie[index + 4]
                          && WPS_IE_OUI_TYPE == ie[index + 5])
                    {
                              memcpy(wps_ie, ie + index, *wps_ie_len + 2);
                              return BTRUE;
                    }
             }
             index++;
       }
       return BFALSE;
}

static int itm_get_rssi(int * rssi)
{
	int ret = 0;
	UWORD8 * host_req  = NULL;
	UWORD8 * trout_rsp = NULL;
	UWORD16  trout_rsp_len = 0;
	WORD8  rssi_db = 0;
	int signal = 0;

	CHECK_MAC_RESET_IN_CFG80211_HANDLER;

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
		return -1;
	}

	trout_rsp_len -= MSG_HDR_LEN;
	if((ret = get_trour_rsp_data((UWORD8*)(&rssi_db),sizeof(UWORD8),&trout_rsp[MSG_HDR_LEN],
					trout_rsp_len,WID_CHAR))  ==  0)
	{	
		pkt_mem_free(trout_rsp);
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

	CHECK_MAC_RESET_IN_CFG80211_HANDLER;

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

static int itm_set_channel(int channel)
{
	UWORD8 * host_req  = NULL;
	UWORD16  trout_rsp_len = 0;
	int len = 0;

	CHECK_MAC_RESET_IN_CFG80211_HANDLER;

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

	return 0;
}

static int itm_remove_wep_key(void)
{
	int ret = 0;
	UWORD8 * host_req  = NULL;
	UWORD16  trout_rsp_len = 0;
	int len=0;

	CHECK_MAC_RESET_IN_CFG80211_HANDLER;

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

out1:
	return ret;
}		

#ifdef IBSS_BSS_STATION_MODE
static UWORD8 *get_scan_ap_info(bss_dscr_t * bss_dscr,UWORD8 * srcbuf)
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
	/*
	 * leon liu stripped & BIT0 for cap_info
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

static int itm_get_device_mode(void)
{
	int ret = -1;
	UWORD8 * host_req  = NULL;
	UWORD8 * trout_rsp = NULL;
	UWORD16  trout_rsp_len = 0;
	UWORD8    mode = 0;

	CHECK_MAC_RESET_IN_CFG80211_HANDLER;

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

	ret = mode & 0xFF;

out2:
	pkt_mem_free(trout_rsp);
out1:
	//kfree(host_req);
	return ret;
}


static int itm_get_Encryption_Type(void)
{
	int ret = -1;
	UWORD8 * host_req  = NULL;
	UWORD8 * trout_rsp = NULL;
	UWORD16  trout_rsp_len = 0;
	UWORD8    encryp_type = 0;

	CHECK_MAC_RESET_IN_CFG80211_HANDLER;

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

out2:
	pkt_mem_free(trout_rsp);
out1:
	//kfree(host_req);
	return ret;
}

static int itm_get_Auth_Type(void)
{
	int ret = -1;
	UWORD8 * host_req  = NULL;
	UWORD8 * trout_rsp = NULL;
	UWORD16  trout_rsp_len = 0;
	UWORD8    auth_type = 0;

	CHECK_MAC_RESET_IN_CFG80211_HANDLER;

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

out2:
	pkt_mem_free(trout_rsp);
out1:
	//kfree(host_req);
	return ret;
}

static int itm_set_Encryption_Type(UWORD8 type)
{
	int ret = -1;
	UWORD8 * host_req  = NULL;
	UWORD16  trout_rsp_len = 0;
	int len = 0;

	CHECK_MAC_RESET_IN_CFG80211_HANDLER;

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
out1:
	//kfree(host_req);
	return ret;
}

static int itm_set_Auth_Type(UWORD8 type)
{
	int ret = -1;
	UWORD8 * host_req  = NULL;
	UWORD16  trout_rsp_len = 0;
	int len = 0;

	CHECK_MAC_RESET_IN_CFG80211_HANDLER;

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
out1:
	//kfree(host_req);
	return ret;
}

static int itm_set_wpa_version(int *encry_type, UWORD32 wpa_version)
{
	int encry_val;

	encry_val = *encry_type;

#ifdef MAC_WAPI_SUPP
	/*leon liu added, disable wapi mode at first*/
	itm_set_wapimode(0);
#endif

	if (wpa_version == 0)
	{
		//Disable
		return 0;

	}
	else if (wpa_version & NL80211_WPA_VERSION_2)
	{
		encry_val &= (BIT7 | BIT6 | BIT5);
		encry_val |= (BIT4 | BIT0);
	}
	else if (wpa_version & NL80211_WPA_VERSION_1)
	{
		encry_val &= (BIT7 | BIT6 | BIT5);
		encry_val |= (BIT3 | BIT0);
	}
#ifdef MAC_WAPI_SUPP
	else if (wpa_version & NL80211_WAPI_VERSION_1)
	{
		if (itm_set_wapimode(1) < 0)
		{
			return -EINVAL;
		}
	}
#endif
	else
	{
		/*Wrong WPA version*/
		printk("Unsupported WPA version %d\n", wpa_version);
		return -ENOTSUPP;
	}

	*encry_type = encry_val;

	if (itm_set_Encryption_Type((UWORD8)encry_val) < 0)
	{
		return -EINVAL;
	}

	return 0;
}

static int itm_set_auth_type(int *auth_type, enum nl80211_auth_type type)
{
	*auth_type = 0;

	switch (type)
	{
		case NL80211_AUTHTYPE_OPEN_SYSTEM:
			*auth_type = ITM_OPEN_SYSTEM & 0xFF;
			break;
		case NL80211_AUTHTYPE_SHARED_KEY:
			*auth_type = ITM_SHARED_KEY & 0xFF;
			break;
		case NL80211_AUTHTYPE_NETWORK_EAP:
		case NL80211_AUTHTYPE_AUTOMATIC:
		default:
			printk("Authentication type 0x%x is not supported\n", type);
			break;
	}

	if (itm_set_Auth_Type((UWORD8)*auth_type) < 0)
	{
		return -EINVAL;
	}

	return 0;
}

static int itm_set_unicast_cipher(int *encry_type, u32 cipher)
{
	int encry_val = *encry_type;

	printk("%s: Encryption type is 0x%x\n", __func__, cipher);

	switch (cipher)
	{
		case WLAN_CIPHER_SUITE_WEP40:
		case WLAN_CIPHER_SUITE_WEP104:
			break;
		case WLAN_CIPHER_SUITE_TKIP:
			encry_val |= BIT6;
			break;
		case WLAN_CIPHER_SUITE_CCMP:
			encry_val |= BIT5;
			break;
		default:
			printk("Unicast cipher suite 0x%x is not supported\n", cipher);
			return -ENOTSUPP;
	}

	*encry_type = encry_val;

	if (itm_set_Encryption_Type((UWORD8)encry_val) < 0)
	{
		printk("itm_set_Encryption_Type() failed\n");
		return -EINVAL;
	}

	return 0;
}

static int itm_set_group_cipher(int *encry_type, u32 cipher)
{
	int encry_val = *encry_type;

	printk("%s: Encryption type is 0x%x\n", __func__, cipher);

	switch (cipher)
	{
		case 0:
			//No encryption
			if (itm_set_Encryption_Type((UWORD8)(No_ENCRYPTION)) < 0)
			{
				return -EINVAL;
			}	

			return 0;
			break;
		case WLAN_CIPHER_SUITE_WEP40:
			if (itm_set_Encryption_Type((UWORD8)(WEP64)) < 0)
			{
				return -EINVAL;
			}

			return 0;
			break;
		case WLAN_CIPHER_SUITE_WEP104:
			if (itm_set_Encryption_Type((UWORD8)(WEP128)) < 0)
			{
				return -EINVAL;
			}

			return 0;
			break;
		case WLAN_CIPHER_SUITE_TKIP:
			encry_val |= BIT6;
			break;
		case WLAN_CIPHER_SUITE_CCMP:
			encry_val |= BIT5;
			break;
		default:
			printk("Group cipher suite 0x%x is not supported\n", cipher);
			return -ENOTSUPP;
	}

	*encry_type = encry_val;

	if (itm_set_Encryption_Type((UWORD8)encry_val) < 0)
	{
		return -EINVAL;
	}

	return 0;
}

static int itm_set_auth_key_management(int *encry_type, u32 key_mgmt)
{
	printk("%s: key_mgmt = 0x%x\n", __func__, key_mgmt);

	if( ((*encry_type & (BIT0 | BIT3)) == (BIT0 | BIT3)) 
			||((*encry_type & (BIT0 | BIT4)) == (BIT0 | BIT4)))
	{
		printk("Encryption type check ok\n");

		if (key_mgmt == WLAN_AKM_SUITE_PSK)
		{
			if (itm_set_Auth_Type((UWORD8)ITM_OPEN_SYSTEM) < 0)
			{
				printk("Failed to set authentication type: OPEN\n");
				return -EINVAL;
			}
		}
		else if (key_mgmt == WLAN_AKM_SUITE_8021X)
		{
			if (itm_set_Auth_Type((UWORD8)ITM_WIFI_ROUTER_PORT_ACTION_8021X_PORT) < 0)
			{
				printk("Failed to set authentication type: 8021X\n");
				return -EINVAL;
			}
		}
		else
		{
			//Not supported
			printk("Unsupported authentication key management type\n");
		}
	}

	return 0;
}

static int itm_set_psk(struct cfg80211_connect_params *sme)
{
	int device_mode;

	CHECK_MAC_RESET_IN_CFG80211_HANDLER;

	if ((device_mode = itm_get_device_mode()) < 0)
	{
		return -EINVAL;
	}

	if (device_mode == DEVICE_MODE_Access_Point || 
			device_mode == DEVICE_MODE_P2P_GO)
	{
		return -EPERM;
	}

	if (sme->key_len == 0)
	{
		printk("PSK length is 0\n");
		return 0;
	}

	if (sme->key_len > 32)
	{
		printk("PSK length is larger than 32\n");
		return -EINVAL;
	}

	printk("Setting PSK: ");

	{
		int i;

		for (i = 0; i < 32; i++)
		{
			printk("[%c(0x%x)]", sme->key[i], sme->key[i]);
		}
	}

	printk("\n");

	memcpy(g_psk_value, sme->key, sme->key_len);
	g_psk_value[39] = 1;

	return 0;
}

static int itm_set_essid(UWORD8 * essid,int essid_len)
{
	int ret = 0;
	UWORD8 * host_req  = NULL;
	UWORD16  trout_rsp_len = 0;
	//int index = 0;
	int len = 0;
	static UWORD8 ap_name[MAX_SSID_LEN] = {0};

	CHECK_MAC_RESET_IN_CFG80211_HANDLER;

	host_req = g_itm_config_buf;//(UWORD8*)kmalloc(ITM_CONFIG_BUF_SIZE, GFP_KERNEL);

	if(host_req == NULL)
	{
		return -ENOMEM;
	}	

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
		printk("<%s> is already config,and wifi enable,we will return!\n",ap_name);
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

		printk("itm_set_essid fuc set essid <");
		for( i = 0 ; i < essid_len ; i ++)
			printk("%c",essid[i]);
		printk(">\n");

		//printk("itm_set_essid fuc set essid <%s>  \n", essid );
	}	
	else
		printk("itm_set_essid fuc clear essid \n");

out1:
	//kfree(host_req);
	return ret;
}

static int itm_set_bssid(UWORD8 * addr)
{
	int ret = 0;
	int len = 0;
	UWORD8 * host_req  = NULL;
	UWORD16  trout_rsp_len = 0;

	CHECK_MAC_RESET_IN_CFG80211_HANDLER;

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

	printk("itm_set_bssid fuc set mac:%02x:%02x:%02x:%02x:%02x:%02x \n",
			addr[0],addr[1],addr[2],
			addr[3],addr[4],addr[5]);

out1:
	//kfree(host_req);
	return ret;
}

static int itm_ap_set_encryption(int key_index, bool pairwise, u8 *mac_addr, struct key_params *params)
{
	int ret = 0;
	int param_len;
	struct ieee_param *in_param = NULL;

	param_len = sizeof(struct ieee_param) + params->key_len;
	in_param = kzalloc(param_len, in_interrupt() ? GFP_ATOMIC: GFP_KERNEL);

	if (in_param == NULL)
	{
		return -ENOMEM;
	}

	//param->cmd = IEEE_CMD_SET_ENCRYPTION;
	memset(in_param->sta_addr, 0xff, ETH_ALEN);

	switch (params->cipher)
	{
		case WLAN_CIPHER_SUITE_WEP40:
		case WLAN_CIPHER_SUITE_WEP104:
			strcpy((char *)in_param->u.crypt.alg, "WEP");
			break;
		case WLAN_CIPHER_SUITE_TKIP:
			strcpy((char *)in_param->u.crypt.alg, "TKIP");
			break;
		case WLAN_CIPHER_SUITE_CCMP:
			strcpy((char *)in_param->u.crypt.alg, "CCMP");
			break;
#ifdef BSS_ACCESS_POINT_MODE
		case WLAN_CIPHER_SUITE_ITM_TKIP:
			printk("AP Mode setting passphrase to driver...\n");
			strcpy((char *)in_param->u.crypt.alg, "itm_set_tkip");
			break;

		case WLAN_CIPHER_SUITE_ITM_CCMP:
			printk("AP Mode setting passphrase to driver...\n");
			strcpy((char *)in_param->u.crypt.alg, "itm_set_ccmp");
			break;
#endif
		default:
			printk("Unsupported cipher suite\n");
			ret = -ENOTSUPP;
			goto out;
			break;
	}


	if (!mac_addr || is_broadcast_ether_addr(mac_addr))
	{
		in_param->u.crypt.set_tx = 0;
	}
	else
	{
		in_param->u.crypt.set_tx = 1;
	}

	in_param->u.crypt.idx = key_index;

	if (params->seq_len && params->seq)
	{
		memcpy(in_param->u.crypt.seq, params->seq, params->seq_len);
	}

	if (params->key_len && params->key)
	{
		in_param->u.crypt.key_len = params->key_len;
		memcpy(in_param->u.crypt.key, params->key, params->key_len);
	}

	if (mac_addr)
	{
		memcpy(in_param->sta_addr, (void *)mac_addr, ETH_ALEN);
	}

	printk("%s: setting key%d: %s\n", __func__, key_index, params->key);
	ret = itm_set_encryption(in_param, param_len);
out:
	if (in_param)
	{
		kfree(in_param);
	}

	return ret;
}

static int itm_sta_set_encryption(int key_index, bool pairwise, const u8 *mac_addr, struct key_params *params)
{
	UWORD8 tkip_key[32] = {0};
	//UWORD8 key_type = 0;
	UWORD8 *keydata = NULL;
	UWORD8 s_addr[ETH_ALEN] = {0};
	int i;
	UWORD16 trout_rsp_len = 0;
	int len = 0;

	cur_cipher = params->cipher;

	switch (params->cipher)
	{
		case WLAN_CIPHER_SUITE_WEP40:
		case WLAN_CIPHER_SUITE_WEP104:
			//Check for valid key length
			if (!((params->key_len == SMALL_KEY_SIZE) || (params->key_len == LARGE_KEY_SIZE)))
			{
				printk("Invalid length for WEP key: %d\n", params->key_len);
				return -EINVAL;
			}

			printk("Setting WEP key %d\n", key_index);

			if (itm_add_wep_key(params->key, params->key_len, key_index) < 0)
			{
				printk("Setting WEP key fail\n");
				return -EINVAL;
			}

			//Set WEP Length
			if (params->key_len == SMALL_KEY_SIZE)
			{
				g_itm_config_buf[len+0] = WID_11I_MODE & 0xFF;
				g_itm_config_buf[len+1] = (WID_11I_MODE & 0xFF00) >> 8;
				g_itm_config_buf[len+2] = 1;
				g_itm_config_buf[len+3] = (UWORD8)WEP64;
				len += WID_CHAR_CFG_LEN;
			}
			else
			{
				g_itm_config_buf[len+0] = WID_11I_MODE & 0xFF;
				g_itm_config_buf[len+1] = (WID_11I_MODE & 0xFF00) >> 8;
				g_itm_config_buf[len+2] = 1;
				g_itm_config_buf[len+3] = (UWORD8)WEP128;
				len += WID_CHAR_CFG_LEN;
			}

			//Set WEP key index
			g_itm_config_buf[len + 0] = WID_KEY_ID & 0xFF;
			g_itm_config_buf[len + 1] = (WID_KEY_ID & 0xFF) >> 8;
			g_itm_config_buf[len + 2] = 1;
			g_itm_config_buf[len + 3] = key_index;
			len += WID_CHAR_CFG_LEN;
			//Set WEP key value
			if (params->key_len)
			{
				UWORD8 *ppkey = params->key;

				g_itm_config_buf[len + 0] = WID_WEP_KEY_VALUE & 0xFF;
				g_itm_config_buf[len + 1] = (WID_WEP_KEY_VALUE & 0xFF) >> 8;
				g_itm_config_buf[len + 2] = (params->key_len & 0xFF) * 2;

				memcpy(&g_itm_config_buf[len + 3], params->key, params->key_len);

				for (i = 0; i < params->key_len; i++)
				{
					g_itm_config_buf[len + 3 + 2 * i] = hex_2_char((UWORD8)((*ppkey & 0xF0) >> 4 ));
					g_itm_config_buf[len + 3 + 2 * i + 1] = hex_2_char((UWORD8)(*ppkey & 0x0F));
					ppkey++;
				}

				len += WID_HEAD_LEN + params->key_len * 2;
			}

			g_itm_config_buf[len+0] = WID_RESET & 0xFF;
			g_itm_config_buf[len+1] = (WID_RESET & 0xFF00) >> 8;
			g_itm_config_buf[len+2] = 1;
			g_itm_config_buf[len+3] = 0; //DONT_RESET
			len += WID_CHAR_CFG_LEN;
			config_if_for_iw(&g_mac,g_itm_config_buf,len,'W',&trout_rsp_len);

			if( trout_rsp_len != 1 )
			{
				printk("Failed to set WEP key %d: ", key_index);

				for (i = 0; i < params->key_len; i++)
				{
					printk("%c(0x%x)", params->key[i], params->key[i]);
				}

				printk("\n");

				return -EINVAL;
			}

			break;
		case WLAN_CIPHER_SUITE_TKIP:
		case WLAN_CIPHER_SUITE_CCMP:
			//Check for valid key length
			if (params->key_len > 32 || params->key_len <= 0)
			{
				printk("Invalid length %d for WPA/WPA2 keys\n", params->key_len);
				return -EINVAL;
			}

			keydata = params->key;

			if (params->cipher == WLAN_CIPHER_SUITE_TKIP && params->key_len == 32)
			{
				memcpy(tkip_key, params->key, 16);
				memcpy(tkip_key + 16, params->key + 24, 8);
				memcpy(tkip_key + 24, params->key + 16, 8);
				keydata = tkip_key;
			}

		//	printk("key type = %d, key index = %d, key_len = %d\n", pairwise, key_index, params->key_len);
		//	printk("key_seq = %p, key_seq_len = %d, keydata = %p\n", params->seq, params->seq_len, keydata);

			/*junbin.wang add 20131206*/
			{
				UWORD8 TmpKey[16];
				UWORD8 i;
				
				machw_ce_read_key(RX_GTK0_NUM, (UWORD8)0, (UWORD8)0, TmpKey);
				printk("[wjb]get before [RX_GTK0_NUM], "); 
				for(i = 0; i < 16; i++)
					printk("%d:0x%x, ", i, TmpKey[i]);
				printk("\n");
				
				machw_ce_read_key(RX_GTK1_NUM, (UWORD8)0, (UWORD8)0, TmpKey);
				printk("[wjb]get before [RX_GTK1_NUM], "); 
				for(i = 0; i < 16; i++)
					printk("%d:0x%x, ", i, TmpKey[i]);
				printk("\n");

				printk("[wjb]set keydata: "); 
				for(i = 0; i < 16; i++)
					printk("%d:0x%x, ", i, keydata[i]);
				printk("\n");
				
			}

			printk("[wjb]set key type = %d, key index = %d, key_len = %d\n", pairwise, key_index, params->key_len);
			printk("[wjb]set key_seq = %p, key_seq_len = %d, keydata = %p\n", params->seq, params->seq_len, keydata);
			
			return itm_add_wpa_wpa2_value(s_addr, keydata, (UWORD8)params->key_len
					, (UWORD8)pairwise, key_index, params->seq);	
			break;
#ifdef MAC_WAPI_SUPP
		case WLAN_CIPHER_SUITE_SMS4:
			{
				wapi_key_t wapi_key;
				UWORD8 WapiASUEPNInitialValueSrc[16] = {0x5C,0x36,0x5C,0x36,0x5C,0x36,0x5C,0x36,0x5C,0x36,0x5C,0x36,0x5C,0x36,0x5C,0x36} ;
				UWORD8 WapiAEMultiCastPNInitialValueSrc[16] = {0x5C,0x36,0x5C,0x36,0x5C,0x36,0x5C,0x36,0x5C,0x36,0x5C,0x36,0x5C,0x36,0x5C,0x36} ;	

				if (params->key_len != 32)
				{
					pr_err("Invalid Key Length for WAPI Key\n");
					return -EINVAL;
				}

				if(pairwise == false)
				{/* Group Key */
					wapi_key.unicastKey = 0;

					memcpy(wapi_key.keyRsc,WapiAEMultiCastPNInitialValueSrc,sizeof(WapiAEMultiCastPNInitialValueSrc));
				}
				else
				{/* Pairwise Key */
					wapi_key.unicastKey = 1;

					memcpy(wapi_key.keyRsc,WapiASUEPNInitialValueSrc,sizeof(WapiASUEPNInitialValueSrc));
				}

				wapi_key.keyIndex   = key_index ;

				if (pairwise == false)
				{
					memset(s_addr, 0xFF, 6);
				}
				else
				{
					memcpy(s_addr, mac_addr, 6);
				}

				memcpy(wapi_key.address, s_addr, 6);

				memcpy(wapi_key.key, params->key, params->key_len);

				return iwprivswapikey(&wapi_key);

			}
			break;
#endif
		default:
			printk("Unknown cipher suite\n");
			break;
	}

	return 0;
}
//----------CFG80211 macros and variables----------
#define RATETAB_ENT(_rate, _rateid, _flags) \
{								\
	.bitrate	= (_rate),				\
	.hw_value	= (_rateid),				\
	.flags		= (_flags),				\
}

#define CHAN2G(_channel, _freq, _flags) {			\
	.band			= IEEE80211_BAND_2GHZ,		\
	.center_freq		= (_freq),			\
	.hw_value		= (_channel),			\
	.flags			= (_flags),			\
	.max_antenna_gain	= 0,				\
	.max_power		= 30,				\
}

#define CHAN5G(_channel, _flags) {				\
	.band			= IEEE80211_BAND_5GHZ,		\
	.center_freq		= 5000 + (5 * (_channel)),	\
	.hw_value		= (_channel),			\
	.flags			= (_flags),			\
	.max_antenna_gain	= 0,				\
	.max_power		= 30,				\
}

static struct ieee80211_rate trout_rates[] = {
	RATETAB_ENT(10,  0x1,   0),
	RATETAB_ENT(20,  0x2,   0),
	RATETAB_ENT(55,  0x4,   0),
	RATETAB_ENT(110, 0x8,   0),
	RATETAB_ENT(60,  0x10,  0),
	RATETAB_ENT(90,  0x20,  0),
	RATETAB_ENT(120, 0x40,  0),
	RATETAB_ENT(180, 0x80,  0),
	RATETAB_ENT(240, 0x100, 0),
	RATETAB_ENT(360, 0x200, 0),
	RATETAB_ENT(480, 0x400, 0),
	RATETAB_ENT(540, 0x800, 0),
};


#define TROUT_G_RATE_NUM	12
#define trout_g_rates		(trout_rates)
#define TROUT_A_RATE_NUM	8
#define trout_a_rates		(trout_rates + 4)

static struct ieee80211_channel trout_2ghz_channels[] = {
	CHAN2G(1, 2412, 0),
	CHAN2G(2, 2417, 0),
	CHAN2G(3, 2422, 0),
	CHAN2G(4, 2427, 0),
	CHAN2G(5, 2432, 0),
	CHAN2G(6, 2437, 0),
	CHAN2G(7, 2442, 0),
	CHAN2G(8, 2447, 0),
	CHAN2G(9, 2452, 0),
	CHAN2G(10, 2457, 0),
	CHAN2G(11, 2462, 0),
	CHAN2G(12, 2467, 0),
	CHAN2G(13, 2472, 0),
	CHAN2G(14, 2484, 0),
};

static struct ieee80211_channel trout_5ghz_channels[] = {
	CHAN5G(34, 0),		CHAN5G(36, 0),
	CHAN5G(38, 0),		CHAN5G(40, 0),
	CHAN5G(42, 0),		CHAN5G(44, 0),
	CHAN5G(46, 0),		CHAN5G(48, 0),
	CHAN5G(52, 0),		CHAN5G(56, 0),
	CHAN5G(60, 0),		CHAN5G(64, 0),
	CHAN5G(100, 0),		CHAN5G(104, 0),
	CHAN5G(108, 0),		CHAN5G(112, 0),
	CHAN5G(116, 0),		CHAN5G(120, 0),
	CHAN5G(124, 0),		CHAN5G(128, 0),
	CHAN5G(132, 0),		CHAN5G(136, 0),
	CHAN5G(140, 0),		CHAN5G(149, 0),
	CHAN5G(153, 0),		CHAN5G(157, 0),
	CHAN5G(161, 0),		CHAN5G(165, 0),
	CHAN5G(184, 0),		CHAN5G(188, 0),
	CHAN5G(192, 0),		CHAN5G(196, 0),
	CHAN5G(200, 0),		CHAN5G(204, 0),
	CHAN5G(208, 0),		CHAN5G(212, 0),
	CHAN5G(216, 0),
};

static struct ieee80211_supported_band trout_band_2ghz = {
	.n_channels = ARRAY_SIZE(trout_2ghz_channels),
	.channels = trout_2ghz_channels,
	.n_bitrates = TROUT_G_RATE_NUM,
	.bitrates = trout_g_rates,
};

static struct ieee80211_supported_band trout_band_5ghz = {
	.n_channels = ARRAY_SIZE(trout_5ghz_channels),
	.channels = trout_5ghz_channels,
	.n_bitrates = TROUT_A_RATE_NUM,
	.bitrates = trout_a_rates,
};

static const u32 trout_cipher_suites[] = {
	WLAN_CIPHER_SUITE_WEP40,
	WLAN_CIPHER_SUITE_WEP104,
	WLAN_CIPHER_SUITE_TKIP,
	WLAN_CIPHER_SUITE_CCMP,
#ifdef BSS_ACCESS_POINT_MODE
	WLAN_CIPHER_SUITE_ITM_CCMP,
	WLAN_CIPHER_SUITE_ITM_TKIP,
#endif
#ifdef MAC_WAPI_SUPP
	WLAN_CIPHER_SUITE_SMS4,
#endif
};


/* Supported mgmt frame types to be advertised to cfg80211 */
static const struct ieee80211_txrx_stypes trout_mgmt_stypes[NUM_NL80211_IFTYPES] = {
	[NL80211_IFTYPE_STATION] = {
		.tx = BIT(IEEE80211_STYPE_ACTION >> 4) |
			BIT(IEEE80211_STYPE_PROBE_RESP >> 4),
		.rx = BIT(IEEE80211_STYPE_ACTION >> 4) |
			BIT(IEEE80211_STYPE_PROBE_REQ >> 4),
	},
	[NL80211_IFTYPE_AP] = {
		.tx = BIT(IEEE80211_STYPE_ACTION >> 4) |
			BIT(IEEE80211_STYPE_PROBE_RESP >> 4),
		.rx = BIT(IEEE80211_STYPE_ACTION >> 4) |
			BIT(IEEE80211_STYPE_PROBE_REQ >> 4),
	},
	[NL80211_IFTYPE_P2P_CLIENT] = {
		.tx = BIT(IEEE80211_STYPE_ACTION >> 4) |
			BIT(IEEE80211_STYPE_PROBE_RESP >> 4),
		.rx = BIT(IEEE80211_STYPE_ACTION >> 4) |
			BIT(IEEE80211_STYPE_PROBE_REQ >> 4),
	},
	[NL80211_IFTYPE_P2P_GO] = {
		.tx = BIT(IEEE80211_STYPE_ACTION >> 4) |
			BIT(IEEE80211_STYPE_PROBE_RESP >> 4),
		.rx = BIT(IEEE80211_STYPE_ACTION >> 4) |
			BIT(IEEE80211_STYPE_PROBE_REQ >> 4),
	},
};

//----------CFG80211 operations callbacks----------
/*Trigger scan*/
static int trout_cfg80211_scan(struct wiphy *wiphy, struct net_device *dev, struct cfg80211_scan_request *request)
{

#ifdef IBSS_BSS_STATION_MODE
	struct trout_wdev_priv *wdev_priv = NULL;
	struct trout_private *stp = NULL;
	struct in_device *my_ip_ptr = NULL;
	int ret = 0;
	UWORD8 *host_req = NULL;
	UWORD16 trout_rsp_len = 0;
	int len = 0;
	int scan_type = 1; //Active scanning
	UWORD8 cur_essid[IW_ESSID_MAX_SIZE+1] = {0};
	int cur_essid_len = 0;
       UWORD8 wps_ie[WPS_IE_BUFF_LEN] = {0};
       UWORD16 wps_ie_len = 0;

	TRACE_FUNC();
	CHECK_MAC_RESET_IN_CFG80211_HANDLER;

	wdev_priv = wiphy_priv(wiphy);
	stp = wdev_priv->trout_priv;
	my_ip_ptr = stp->dev->ip_ptr;
	spin_lock_bh(&wdev_priv->scan_req_lock);
	wdev_priv->scan_request = request;
	spin_unlock_bh(&wdev_priv->scan_req_lock);

#if 0
        /*junbinwang modify for wps 20130812*/
        if(request->ie_len > 0){
                trout_set_probe_req_wps_ie(1);
                trout_save_probe_req_wps_ie(request->ie, request->ie_len);
                printk("[wjb]trout_cfg80211_scan setting ok\n");
        }
#else
        trout_clear_probe_req_wps_ie(); //clean extra ie
        trout_set_wps_sec_type_flag(0);
        //for p2p in the future, so use trout_find_wps_ie
        if (BTRUE == trout_find_wps_ie(request->ie, request->ie_len, wps_ie, &wps_ie_len))
        {
                trout_save_probe_req_wps_ie(request->ie, request->ie_len);
                trout_set_probe_req_wps_ie(1);
        }
#endif

	/*leon liu modified get_mac_state() >= WAIT_JOIN*/
	if (get_mac_state() == ENABLED || g_keep_connection == BTRUE)
	{
		if (my_ip_ptr != NULL)
		{
			struct in_ifaddr *my_ifa_list = my_ip_ptr->ifa_list;

			if (my_ifa_list != NULL)
			{
				goto CHECK_SCAN;
			}
		}

		printk("Skip scanning <ip addr less>\n");
		//chenq mod 2013-05-23
		is_scanlist_report2ui = 2;
		send_mac_status(MAC_SCAN_CMP);
		trout_cfg80211_report_scan_done(dev, 1);
		return 0;
	}

	cur_essid_len = itm_get_essid(cur_essid,IW_ESSID_MAX_SIZE);
	/*
	 * leon liu added MAC STATE judgement here,
	 * if essid is set and mac state is not wait_scan, skip this scan
	 */
	if(cur_essid_len && get_mac_state() != WAIT_SCAN)
	{
		printk("skip scan op <%s> \n",
				(cur_essid_len < 0) ? "get cur essid err":"wifi linking" );
		trout_cfg80211_report_scan_done(dev, 1);
		return -EAGAIN;
	}

CHECK_SCAN:
	if (itm_scan_flag == 1)
	{
		printk("Skip scanning <already in scan>\n");
		//chenq add 2013-06-09
		//is_scanlist_report2ui = 2;
//		send_mac_status(MAC_SCAN_CMP);
		trout_cfg80211_report_scan_done(dev, 1);
		return 0;
	}
	else if (g_BusyTraffic == BTRUE)
	{
		printk("Skip scanning <BusyTraffic>\n");
		//chenq add 2013-06-09
		//is_scanlist_report2ui = 2;
//		send_mac_status(MAC_SCAN_CMP);
		trout_cfg80211_report_scan_done(dev, 1);
		return 0;
	}

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifdef WIFI_SLEEP_POLICY
       //Bug#229353
        if(!wake_lock_active(&scan_ap_lock)){
    	        wake_lock(&scan_ap_lock);
    	 printk("@@@: acquire scan_ap_lock in %s\n", __func__);
        }
#endif
#endif

	/*leon liu added for combo scan support*/
#ifdef COMBO_SCAN
	{
		int i,j,flag = 0;
		int scan_list_index = 0;

		g_ap_combo_scan_index = 0;
		g_ap_combo_scan_cnt = 0;
		memset(g_ap_combo_scan_list,0x00,sizeof(g_ap_combo_scan_list));//libing add for fix buger:211215
		for(j=0;j<MAX_AP_COMBO_SCAN_LIST;j++)
		{
			g_combo_aplist[j].cur_flag = 0;
			if(g_combo_aplist[j].cnt > 0)
			g_combo_aplist[j].cnt--;
			if(0 == g_combo_aplist[j].cnt)
			memset(g_combo_aplist[j].ssid,0x00,sizeof(g_combo_aplist[j].ssid));
		}

		for (i = 0; i < request->n_ssids && i < MAX_AP_COMBO_SCAN; i++)
		{
			if (request->ssids[i].ssid_len == 0)
			{
				continue;
			}

			memcpy(g_ap_combo_scan_list[scan_list_index], request->ssids[i].ssid, request->ssids[i].ssid_len);
			for(j=0;j<MAX_AP_COMBO_SCAN_LIST;j++)
			{
				if(g_combo_aplist[j].cnt  !=0)
				{
					if(strcmp(request->ssids[i].ssid,g_combo_aplist[j].ssid) == 0)
					{
						g_combo_aplist[j].cur_flag = 1;
						g_combo_aplist[j].cnt = 10;
						g_combo_aplist[j].ssid_len = request->ssids[i].ssid_len;						
						flag = 1;
						printk("g_combo_aplist with j %d last ssid:%s\n",j,g_combo_aplist[j].ssid);
						break;
					}
				}
				if(j == MAX_AP_COMBO_SCAN_LIST-1)
				printk("g_combo_aplist last ssid no find in combo ap list\n");
			}
			if(0 == flag)
			{
				for(j=0;j<MAX_AP_COMBO_SCAN_LIST;j++)
				{
					if((0 == g_combo_aplist[j].cur_flag) && (0 == g_combo_aplist[j].cnt))
					{
						g_combo_aplist[j].cur_flag = 1;
						g_combo_aplist[j].cnt = 10;
						g_combo_aplist[j].ssid_len = request->ssids[i].ssid_len;
						memcpy(g_combo_aplist[j].ssid, request->ssids[i].ssid, request->ssids[i].ssid_len);
						g_combo_aplist[j].ssid[request->ssids[i].ssid_len] = 0;
						printk("g_combo_aplist with j %d ssid:%s\n",j,g_combo_aplist[j].ssid);
						break;
					}
					if(j == MAX_AP_COMBO_SCAN_LIST-1)
					printk("g_combo_aplist is full\n");
				}
			}
			g_ap_combo_scan_list[scan_list_index][request->ssids[i].ssid_len] = 0;
			scan_list_index++;
		}

		g_ap_combo_scan_cnt = scan_list_index;
		
	}

#endif

	host_req = g_itm_config_buf;

	if (host_req == NULL)
	{
		trout_cfg80211_report_scan_done(dev, 1);
		return -ENOMEM;
	}

	/*junbinwang add 20131027*/
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

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifdef WIFI_SLEEP_POLICY
	wake_lock(&scan_ap_lock); /*Keep awake when scan ap, by caisf 20130929*/
	pr_info("%s-%d: acquire wake_lock %s\n", __func__, __LINE__, scan_ap_lock.name);
#endif
#endif

	config_if_for_iw(&g_mac,host_req,len,'W',&trout_rsp_len);

	if( trout_rsp_len != 1 )
	{
		printk("%s: config_if_for_iw() failed\n", __func__);
		ret = -EINVAL;
		trout_cfg80211_report_scan_done(dev, 1);
		goto out1;
	}


out1:
	if (itm_scan_flag == 1)
	{

		if (!ret)
		{
			is_scanlist_report2ui = 1;
		}
		else
		{
			//chenq add 2013-06-09ss
			is_scanlist_report2ui = 2;
			send_mac_status(MAC_SCAN_CMP);
			ret = 0; //-EBUSY;
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
	wake_unlock(&scan_ap_lock); /*Keep awake when scan ap, by caisf 20130929*/
	pr_info("%s-%d: release wake_lock %s\n", __func__, __LINE__, scan_ap_lock.name);
#endif
#endif

	return ret;
#else
	TRACE_FUNC();
	return -EOPNOTSUPP;
#endif

}

/*Connect*/
static int trout_cfg80211_connect(struct wiphy *wiphy, struct net_device *dev,
		struct cfg80211_connect_params *sme)
{
	int ret = 0;
	int encry_type = 0;
	int auth_type = 0;

	//begin by modified junwei.jiang bug232001 2013-12-16
	struct trout_wdev_priv *wdev_priv;
	struct trout_private *t_private;
	
	if(0 == strlen(sme->ssid) || (NULL == sme->bssid) ||(NULL == sme->ssid)){
		printk("trout_cfg80211_connect ssid is null\n");
		ret = -EINVAL;
	}
	else
	{
		wdev_priv = wiphy_priv(wiphy);
		t_private = wdev_priv->trout_priv;
		memset(t_private->ssid,0x00,sizeof(t_private->ssid));
		memcpy(t_private->ssid, sme->ssid, sme->ssid_len);
		printk("trout_cfg80211_connect with ssid:%s\n",t_private->ssid);

		memcpy(t_private->bssid, sme->bssid, ETH_ALEN);
		printk("trout_cfg80211_connect with bssid:%02x:%02x:%02x:%02x:%02x:%02x\n",
			t_private->bssid[0],t_private->bssid[1],t_private->bssid[2],t_private->bssid[3],t_private->bssid[4],t_private->bssid[5]);
		t_private->ssid_len = sme->ssid_len;
	}
	//end by modified junwei.jiang bug232001 2013-12-16
	TRACE_FUNC();
	CHECK_MAC_RESET_IN_CFG80211_HANDLER;

#ifdef IBSS_BSS_STATION_MODE
        //yangke, 2013-10-16, set to default value when request connect
        g_default_scan_limit = 0;
#endif

	//TODO: get mac state and decide whether we should go on

	/*junbin.wang modify 20131126.*/
/*	if ((encry_type = itm_get_Encryption_Type()) < 0)
	{
		return -EINVAL;
	}

	if ((auth_type = itm_get_Auth_Type()) < 0)
	{
		return -EINVAL;
	}*/

        /*junbinwang modify for wps 20130812*/
        trout_clear_asoc_req_wps_ie(); //clean extra ie
        trout_set_wps_sec_type_flag(0);
        if(sme->ie_len > 0){
                trout_set_asoc_req_wps_ie(1);
                trout_save_asoc_req_wps_ie(sme->ie, sme->ie_len);
                printk("[wjb]trout_cfg80211_connect wps ok\n");
	}

	//FIXME:Set appending ie,currently not supported
	//Set WPA version
	ret = itm_set_wpa_version(&encry_type, sme->crypto.wpa_versions);

	if (ret < 0)
	{
		printk("itm_set_wpa_version(0x%x, %d) failed\n", encry_type, sme->crypto.wpa_versions);
		goto err_out;
	}
	
	//Set Auth Type
	ret = itm_set_auth_type(&auth_type, sme->auth_type);

	if (ret < 0)
	{
		printk("itm_set_auth_type(0x%x, %d) failed\n", auth_type, sme->auth_type);
		goto err_out;
	}

	/*leon liu added, if WAPI is used, skip configuring UNICAST and MULTICAST cipher suite*/
	if (sme->crypto.wpa_versions & NL80211_WAPI_VERSION_1)
	{
		goto set_akm_suite;
	}

	//Set cipher - pairewise and group
	if (sme->crypto.n_ciphers_pairwise)
	{
		ret = itm_set_unicast_cipher(&encry_type, sme->crypto.ciphers_pairwise[0]);	
	}
	else
	{
		//No pairewise cipher
	}

	if (ret < 0)
	{
		printk("itm_set_unicast_cipher() failed\n");
		goto err_out;
	}

	ret = itm_set_group_cipher(&encry_type, sme->crypto.cipher_group);

	if (ret < 0)
	{
		printk("itm_set_group_cipher() failed\n");
		goto err_out;
	}

set_akm_suite:
	//Set auth key management
	if (sme->crypto.n_akm_suites)
	{
		ret = itm_set_auth_key_management(&encry_type, sme->crypto.akm_suites[0]);
	}

	if (ret < 0)
	{
		printk("itm_set_auth_key_management() failed\n");
		goto err_out;
	}

	//Auth privacy invoked is not implemented, do nothing
	//Set PSK
	if (sme->crypto.cipher_group == WLAN_CIPHER_SUITE_WEP40 
			|| sme->crypto.cipher_group == WLAN_CIPHER_SUITE_WEP104
			|| sme->crypto.ciphers_pairwise[0] == WLAN_CIPHER_SUITE_WEP40
			|| sme->crypto.ciphers_pairwise[0] == WLAN_CIPHER_SUITE_WEP104)
	{
		printk("Don't need to set PSK since driver is using WEP\n");
	}
	else
	{
		ret = itm_set_psk(sme);
	}

	if (ret < 0)
	{
		printk("itm_set_psk() failed\n");
		goto err_out;
	}

	//Auth RX unencrypted EAPOL is not implemented, do nothing
	//Set channel
	if (sme->channel != NULL)
	{
		printk("Settting channel to %d\n", ieee80211_frequency_to_channel(sme->channel->center_freq));
		ret = itm_set_channel(ieee80211_frequency_to_channel(sme->channel->center_freq));
	}
	else
	{
		printk("Channel is not specified\n");
	}

	if (ret < 0)
	{
		printk("itm_set_channel() failed\n");
		goto err_out;
	}

	//Set BSSID
	if (sme->bssid != NULL)
	{
		ret = itm_set_bssid(sme->bssid);
	}
	else
	{
		printk("BSSID is not specified\n");
	}

	if (ret < 0)
	{
		printk("itm_set_bssid() failed\n");
		goto err_out;
	}

	cur_cipher = sme->crypto.cipher_group;
	//Special process for WEP(WEP key must be set before itm_set_essid)
	if (sme->crypto.cipher_group == WLAN_CIPHER_SUITE_WEP40 || sme->crypto.cipher_group == WLAN_CIPHER_SUITE_WEP104)
	{
		//int i;
		struct key_params key;

		printk("Setting WEP group cipher\n");

		if (sme->key_len <= 0)
		{
			printk("No key is specified\n");
		}
		else
		{
			u8 keydata[32] = {0};
#if 0
			printk("Setting WEP key %d: ", sme->key_idx);

			for (i = 0; i < sme->key_len; i++)
			{
				printk("%x(0x%x)-", sme->key[i]);
			}

			printk("\n");
#endif

			if (sme->key_len != LARGE_KEY_SIZE && sme->key_len != SMALL_KEY_SIZE)
			{
				printk("Invalid key length for WEP\n");
				return -EINVAL;
			}

			//sme->key is const u8 *, copy the data for safety
			memcpy(keydata, sme->key, sme->key_len);
			key.key = keydata;
			key.cipher = sme->crypto.cipher_group;
			key.key_len = sme->key_len;

			itm_sta_set_encryption(sme->key_idx, 0, NULL, &key);
		}
	}
	//Set ESSID
	ret = itm_set_essid(sme->ssid, (int)sme->ssid_len);

	printk("Encryption Type: 0x%x Authentication Type: 0x%x\n", itm_get_Encryption_Type(), itm_get_Auth_Type());
	//Debug
	//ret = -EINVAL;

	return ret;
err_out:
	return ret;
}

/*Disconnect*/
int trout_cfg80211_disconnect(struct wiphy *wiphy, struct net_device *dev,
		u16 reason_code)
{
	int timeout_left = 0;
	TRACE_FUNC();
	CHECK_MAC_RESET_IN_CFG80211_HANDLER;

#ifdef IBSS_BSS_STATION_MODE
	//During scanning
	if (itm_scan_flag == 1)
	{

		/*
		 *leon liu added, wait for completion of scanning on 2013-07-12
		 *Fix bug: when disconnecting from AP,
		 *if driver is in scanning, we should not just return error.
		 *Must update nl80211 layer to keep device status updated.
		 */
		timeout_left = wait_for_completion_interruptible_timeout(
				&scan_completion, SCAN_COMPLETION_TIMEOUT);

		if (unlikely(!timeout_left)){
			/*Timed out, just inform upper layer that scan is aborted*/
			pr_err("Scanning is not done within specified time!\n");
			trout_cfg80211_report_scan_done(dev, 1);
			return -EAGAIN;
		}
	}
#endif
	restart_mac_plus(&g_mac, 0);

	return 0;
}

/*Wiphy parameters settings*/
static int trout_cfg80211_set_wiphy_params(struct wiphy *wiphy, u32 changed)
{
	struct trout_wdev_priv *wdev_priv;
	struct trout_private *stp;
	UWORD8 *host_req  = NULL;
	UWORD16 trout_rsp_len = 0;
	int val = 0;

	TRACE_FUNC();
	CHECK_MAC_RESET_IN_CFG80211_HANDLER;

	wdev_priv = wiphy_priv(wiphy);
	stp = wdev_priv->trout_priv;

	host_req = g_itm_config_buf;

	if (host_req == NULL)
	{
		printk("g_itm_config_buf is NULL\n");
		return -ENOMEM;
	}

	if (changed & WIPHY_PARAM_RTS_THRESHOLD)
	{
		//Change RTS threshold
		val = wiphy->rts_threshold;

		if (val == -1)
		{
			//Disable RTS
			val = 65535;
		}

		if (val < 256 || val > 65535)
		{
			//Modify printk to TROUT_DBGx
			printk("Invalid RTS threshold(%d) to set\n", val);
			return -EINVAL;
		}

		printk("Setting RTS to %d\n", val);

		host_req[0] = WID_RTS_THRESHOLD & 0xFF;
		host_req[1] = (WID_RTS_THRESHOLD & 0xFF00) >> 8;

		host_req[2] = (WID_SHORT + 1) & 0xFF;

		host_req[3] = val & 0xFF;
		host_req[4] = (val & 0xFF00) >> 8;
		config_if_for_iw(&g_mac,host_req,(UWORD16)WID_SHORT_CFG_LEN,
				'W',&trout_rsp_len);

		if(trout_rsp_len != 1)
		{
			printk("Failed to set RTS threshold\n");
			return -EINVAL;
		}

#if 0
		//Debug, retrieve RTS to see if RTS is configured
		{
			UWORD8 *trout_rsp = NULL;
			UWORD16 rts = 0;

			host_req[0] = WID_RTS_THRESHOLD & 0xFF;
			host_req[1] = (WID_RTS_THRESHOLD & 0xFF00) >> 8;

			trout_rsp = config_if_for_iw(&g_mac,host_req,2,'Q',&trout_rsp_len);

			if( trout_rsp == NULL )
			{
				printk("Cannot get RTS from trout\n");
				return 0;
			}

			trout_rsp_len -= MSG_HDR_LEN;
			if(get_trour_rsp_data((UWORD8 *)(&rts),sizeof(rts),&trout_rsp[MSG_HDR_LEN],
						trout_rsp_len,WID_SHORT)  ==  0)
			{
				printk("Cannot get response data from trout\n");
				pkt_mem_free(trout_rsp);
				return 0;
			}

			printk("RTS read from Trout: %d\n", rts);
			pkt_mem_free(trout_rsp);
		}
#endif
	}

	if (changed & WIPHY_PARAM_FRAG_THRESHOLD)
	{
		//Change frag threshold
		val = wiphy->frag_threshold;

		if (val == -1)
		{
			//Disable frag threshold
			val = 2346;
		}

		if (val < 256 || val > 7936)
		{
			printk("Invalid fragmentation threshold(%d) to set\n", val);
			return -EINVAL;
		}

		printk("Setting fragmentation threshold to %d\n", val);

		host_req[0] = WID_FRAG_THRESHOLD & 0xFF;
		host_req[1] = (WID_FRAG_THRESHOLD & 0xFF00) >> 8;

		host_req[2] = (WID_SHORT + 1) & 0xFF;

		host_req[3] = val & 0xFF;
		host_req[4] = (val & 0xFF00) >> 8;

		config_if_for_iw(&g_mac,host_req,WID_SHORT_CFG_LEN,'W',&trout_rsp_len);

		if(trout_rsp_len != 1)
		{
			printk("Failed to set fragmentation threshold\n");
			return -EINVAL;
		}
	}

	//TODO: consider configuring long/short retry and coverage class


	return 0;
}

/*Tx power setting*/
static int trout_cfg80211_set_txpower(struct wiphy *wiphy, enum nl80211_tx_power_setting type,
		int mbm)
{
	TRACE_FUNC();

	//Currently not implemented
	return 0;
}

/*Tx power retrieving*/
static int trout_cfg80211_get_txpower(struct wiphy *wiphy, int *dbm)
{
	TRACE_FUNC();

	//Currently not implemented
	return 0;
}

/*Station information retrieving*/
static int trout_cfg80211_get_station(struct wiphy *wiphy, struct net_device *dev,
		u8 *mac, struct station_info *sinfo)
{
	struct trout_private *stp;
	int signal, rate;
	int j;

	TRACE_FUNC();
	stp = netdev_priv(dev);
	/*No information filled*/
	sinfo->filled = 0;

	/*TODO: compare mac with bssid*/
#ifdef IBSS_BSS_STATION_MODE
	if ( (get_mac_state() != ENABLED) && (g_keep_connection != BTRUE))
	{
		//TROUT_DBG5("MAC state = %d, no information can be retrieved\n", get_mac_state());
		printk("MAC state = %d, no information can be retrieved\n", get_mac_state());
		return 0;
	}
#endif	
	/*Get RSSI*/
	if (itm_get_rssi(&signal) < 0)
	{
		TROUT_DBG5("itm_get_rssi() failed\n");
		return 0;
	}

	sinfo->filled |= STATION_INFO_SIGNAL;
	sinfo->signal = signal;

	/*Get TX RATE*/
	/*junbin.wang modify 20131128, if rate < 0, will report default value.*/
	rate = itm_get_rate();
	printk("trout_cfg80211_get_station rate = 0x%x\n", rate);
	if (rate  < 0)
	{
		TROUT_DBG5("itm_get_rate() failed\n");
		sinfo->txrate.mcs = 0x02;
		sinfo->txrate.legacy = 5 * 10;
		return 0;
	}
	sinfo->filled |= STATION_INFO_TX_BITRATE;

	if (IS_RATE_MCS(rate) == BFALSE)
	{
		sinfo->txrate.legacy = rate * 10; //legacy unit is 100kbits/s
	}
	else
	{
		for (j = 0; j < sizeof(rate_table) / sizeof(rate_table_t); j++)
		{
			if (rate_table[j].rate_index == rate)
			{
				//yangke, 2013-10-05, workaround for display tx rate for 11n, ultimate fix need in wpa_supplicant

				/*junbin.wang modify for report data rate on n mode.20131125.*/
				
				sinfo->txrate.mcs = rate_table[j].rate_index;			
				sinfo->txrate.legacy = rate_table[j].rate * 10;
				break;
			}
		}		

		if (j >= sizeof(rate_table) / sizeof(rate_table_t))
		{
			sinfo->txrate.legacy = 10;
		}
	}

	//TODO: fill up tx/rx bytes and other information needed in enum station_info_flags

	return 0;
}

/*Set PMKSA*/
static int trout_cfg80211_set_pmksa(struct wiphy *wiphy, struct net_device *dev, struct cfg80211_pmksa *pmksa)
{
	int ret = 0;
	UWORD8 *host_req = NULL;
	UWORD16 trout_rsp_len = 0;
	int len = 0;

	TRACE_FUNC();
	CHECK_MAC_RESET_IN_CFG80211_HANDLER;

	host_req = g_itm_config_buf;

	if (host_req == NULL)
	{
		return -ENOMEM;
	}

	host_req[len++] = WID_PMKID_INFO & 0xFF;
	host_req[len++] = (WID_PMKID_INFO & 0xFF) >> 8;
	host_req[len++] = 1 + 6 + 16;

	/*NumEntries*/
	host_req[len++] = 1;

	/* BSSID */
	memcpy(&host_req[len], pmksa->bssid, 6);
	len += 6;

	/* PMKID */
	memcpy(&host_req[len], pmksa->pmkid, 16);
	len += 16;

	/*leon liu added, no reset mac*/
	host_req[len+0] = WID_RESET & 0xFF;
	host_req[len+1] = (WID_RESET & 0xFF00) >> 8;
	host_req[len+2] = 1;
	host_req[len+3] = 0;//( (g_mac.state == ENABLED) || (g_keep_connection == BTRUE) ) ? 0 : 2; //DONT_RESET
	len += WID_CHAR_CFG_LEN;
	config_if_for_iw(&g_mac,host_req, len, 'W', &trout_rsp_len);

	if( trout_rsp_len != 1 )
	{
		ret = -EINVAL;
	}

	return ret;
}

/*Delete PMKSA*/
static int trout_cfg80211_del_pmksa(struct wiphy *wiphy, struct net_device *dev, struct cfg80211_pmksa *pmksa)
{
	return 0;
}

/*Flush PMKSA*/
static int trout_cfg80211_flush_pmksa(struct wiphy *wiphy, struct net_device *dev)
{
	return 0;
}

/*Set channel*/
static int trout_cfg80211_set_channel(struct wiphy *wiphy, struct net_device *ndev, struct ieee80211_channel *chan,
		enum nl80211_channel_type channel_type)
{
	int channel_num;

	TRACE_FUNC();

	channel_num = ieee80211_frequency_to_channel(chan->center_freq);

	return itm_set_channel(channel_num);
}

/*Add key*/
static int trout_cfg80211_add_key(struct wiphy *wiphy, struct net_device *dev, u8 key_index, bool pairwise,
		const u8 *mac_addr, struct key_params *params)
{
	int ret = 0;
	int device_mode = 0;

	TRACE_FUNC();
	CHECK_MAC_RESET_IN_CFG80211_HANDLER;

	if ((device_mode = itm_get_device_mode()) < 0)
	{
		return -EINVAL;
	}

	switch (device_mode)
	{
		case DEVICE_MODE_Access_Point:
		case DEVICE_MODE_P2P_GO:
			//AP set encryption
			printk("AP mode set encryption\n");
			ret = itm_ap_set_encryption(key_index, pairwise, mac_addr, params);
			break;
		case DEVICE_MODE_BSS_STA:
		case DEVICE_MODE_IBSS_STA:
			//STA set encryption
			//WEP should be set in trout_cfg80211_connect, no need to do it again
			ret = itm_sta_set_encryption(key_index, pairwise, mac_addr, params);
			break;
		default:
			printk("Unsupported device mode: %d\n", device_mode);
			ret = -EINVAL;
			break;
	}

	return ret;
}

/*Delete key*/
static int trout_cfg80211_del_key(struct wiphy *wiphy, struct net_device *dev, u8 key_index, bool pairwise,
		const u8 *mac_addr)
{
	int ret = 0;
	int device_mode = 0;

	TRACE_FUNC();
	CHECK_MAC_RESET_IN_CFG80211_HANDLER;

	if ((device_mode = itm_get_device_mode()) < 0)
	{
		return -EINVAL;
	}

	switch (device_mode)
	{
		case DEVICE_MODE_Access_Point:
		case DEVICE_MODE_P2P_GO:
			ret = itm_set_Auth_Type((UWORD8)(ITM_OPEN_SYSTEM & 0xFF));

			if (ret < 0)
			{
				break;
			}

			ret = itm_set_Encryption_Type(No_ENCRYPTION);
			break;
		case DEVICE_MODE_BSS_STA:
		case DEVICE_MODE_IBSS_STA:
			ret = itm_remove_wep_key();
			if (ret < 0)
			{
				break;
			}

			ret = itm_set_Encryption_Type(No_ENCRYPTION);
			break;
		default:
			printk("Unsupported device mode: %d\n", device_mode);
			ret = -EINVAL;
			break;
	}

	if (ret)
	{
		printk("Delete key failed with %d\n", ret);
	}

	return ret;
}

/*Get key*/
static int trout_cfg80211_get_key(struct wiphy *wiphy, struct net_device *ndev, u8 key_index, bool pairwise,
		const u8 *mac_addr, void *cookie, void (*callback) (void *cookie, struct key_params *))
{
	return 0;
}


static int trout_cfg80211_set_default_key(struct wiphy *wiphy, struct net_device *ndev, u8 key_index, bool unicast,
		bool multicast)
{
	int ret = 0;
	//int privacy = -1;
	//int tmp_key_index = 0;
	//int len = 0;
	//UWORD16 trout_rsp_len = 0;

	TRACE_FUNC();
	CHECK_MAC_RESET_IN_CFG80211_HANDLER;

	if (cur_cipher != WLAN_CIPHER_SUITE_WEP40 && cur_cipher != WLAN_CIPHER_SUITE_WEP104)
	{
		printk("%s set none WEP key, nothing to do\n", __func__);
		return 0;
	}

	if (key_index < 0 || key_index > 3)
	{
		printk("Invalid key index %d\n", key_index);
		return -EINVAL;
	}

	ret = itm_set_wep_key_index(key_index);

	if (ret < 0)
	{
		printk("itm_set_wep_key_index() failed\n");
	}

	return ret;
}

/*Add/Set beacon*/
static int trout_add_beacon(const u8 *head, int head_len, const u8 *tail, int tail_len)
{
#ifdef BSS_ACCESS_POINT_MODE
	int ret;
	u8 *bcn_ie_buf = NULL;

	TRACE_FUNC();

	bcn_ie_buf = kzalloc(head_len + tail_len, GFP_KERNEL);

	if (bcn_ie_buf == NULL)
	{
		return -ENOMEM;
	}

	//Construct beacon ie
	memcpy(bcn_ie_buf, head + 24, head_len - 24);

	if (tail != NULL)
	{
		memcpy(bcn_ie_buf + head_len - 24, tail, tail_len);
	}

	ret = itm_set_ap_info(bcn_ie_buf, head_len + tail_len - 24);

	if (ret != 0)
	{
		ret = -EINVAL;
	}

	kfree(bcn_ie_buf);

	return ret;
#else
	TRACE_FUNC();
	return -EINVAL;
#endif
}

static int trout_cfg80211_add_beacon(struct wiphy *wiphy, struct net_device *dev, struct beacon_parameters *info)
{

	return trout_add_beacon(info->head, info->head_len, info->tail, info->tail_len);
}

static int trout_cfg80211_set_beacon(struct wiphy *wiphy, struct net_device *dev, struct beacon_parameters *info)
{
	return trout_cfg80211_add_beacon(wiphy, dev, info);
}

static int trout_cfg80211_del_station(struct wiphy *wiphy, struct net_device *dev, u8 *mac)
{
	return 0;
}

static int trout_cfg80211_add_station(struct wiphy *wiphy, struct net_device *dev, u8 *mac, struct station_parameters *params)
{
	return 0;
}

static int trout_cfg80211_change_bss(struct wiphy *wiphy, struct net_device *dev, struct bss_parameters *params)
{
	return 0;
}

static int trout_cfg80211_set_power_mgmt(struct wiphy *wiphy, struct net_device *dev, bool enabled, int timeout)
{
	return 0;
}


/*CFG80211 operations for trout*/
struct cfg80211_ops trout_cfg80211_ops = {
	//.change_virtual_intf = trout_cfg80211_change_iface,
	.scan = trout_cfg80211_scan,
	.connect = trout_cfg80211_connect,
	.disconnect = trout_cfg80211_disconnect,
	.set_wiphy_params = trout_cfg80211_set_wiphy_params,
	.set_tx_power = trout_cfg80211_set_txpower,
	.get_tx_power = trout_cfg80211_get_txpower,
	.get_station = trout_cfg80211_get_station,
	.set_pmksa = trout_cfg80211_set_pmksa,
	.del_pmksa = trout_cfg80211_del_pmksa,
	.flush_pmksa = trout_cfg80211_flush_pmksa,
	//set_channel is stripped in future kernel
	.set_channel = trout_cfg80211_set_channel,
	.add_key = trout_cfg80211_add_key,
	.del_key = trout_cfg80211_del_key,
	.get_key = trout_cfg80211_get_key,
	.set_default_key = trout_cfg80211_set_default_key,
	.add_beacon = trout_cfg80211_add_beacon,
	.set_beacon = trout_cfg80211_set_beacon,
	.del_station = trout_cfg80211_del_station,
	.add_station = trout_cfg80211_add_station,
	.change_bss = trout_cfg80211_change_bss,
	.set_power_mgmt = trout_cfg80211_set_power_mgmt,
};

/*Wiphy open - init wiphy ht capabilities (called by netdev_open)*/
void trout_wiphy_open(struct trout_private *trout_priv)
{
	//TODO: fill ht capabilities
	struct ieee80211_supported_band *bands;
	struct wireless_dev *wdev = trout_priv->wdev;
	struct wiphy *wiphy = wdev->wiphy;

	bands = wiphy->bands[IEEE80211_BAND_2GHZ];

	if (bands)
	{
		//trout_wiphy_init_ht_cap();
	}

	bands = wiphy->bands[IEEE80211_BAND_5GHZ];

	if (bands)
	{
		//trout_wiphy_init_ht_cap();
	}

}

/*Init wiphy parameters*/
static void init_wiphy_parameters(struct trout_private *trout_priv, struct wiphy *wiphy)
{
	wiphy->signal_type = CFG80211_SIGNAL_TYPE_MBM;
	wiphy->mgmt_stypes = trout_mgmt_stypes;

	wiphy->max_scan_ssids = MAX_SITES_FOR_SCAN;
	wiphy->max_scan_ie_len = SCAN_IE_LEN_MAX;	
	wiphy->max_num_pmkids = MAX_NUM_PMKIDS;

	/*TODO:consider AP mode*/
#ifdef IBSS_BSS_STATION_MODE
	wiphy->interface_modes = BIT(NL80211_IFTYPE_STATION);
#endif
#ifdef BSS_ACCESS_POINT_MODE
	printk("%s: AP mode\n", __func__);
	wiphy->interface_modes = BIT(NL80211_IFTYPE_AP);
#endif
	//Attach cipher suites
	wiphy->cipher_suites = trout_cipher_suites;
	wiphy->n_cipher_suites = ARRAY_SIZE(trout_cipher_suites);
	//Attach bands
	wiphy->bands[IEEE80211_BAND_2GHZ] = &trout_band_2ghz;
	wiphy->bands[IEEE80211_BAND_5GHZ] = &trout_band_5ghz;

	//Default not in powersave state
	wiphy->flags &= ~WIPHY_FLAG_PS_ON_BY_DEFAULT;
}

//----------Public interfaces for cfg80211----------
//Debug 
#if 0
const u8 *cfg80211_find_ie(u8 eid, const u8 *ies, int len)
{
	while (len > 2 && ies[0] != eid) {
		len -= ies[1] + 2;
		ies += ies[1] + 2;
	}
	if (len < 2)
		return NULL;
	if (len < 2 + ies[1])
		return NULL;
	return ies;
}

static bool is_bss(struct cfg80211_bss *a,
		const u8 *bssid,
		const u8 *ssid, size_t ssid_len)
{
	const u8 *ssidie;

	if (bssid && compare_ether_addr(a->bssid, bssid))
		return false;

	if (!ssid)
		return true;

	ssidie = cfg80211_find_ie(WLAN_EID_SSID,
			a->information_elements,
			a->len_information_elements);
	if (!ssidie)
		return false;
	if (ssidie[1] != ssid_len)
		return false;
	return memcmp(ssidie + 2, ssid, ssid_len) == 0;
}

static void dump_ie(u8 *ie, int ie_len)
{
	int i;

	printk("Information Elements:\n");
	for (i = 0;i < ie_len; i++)
	{
		printk("0x%x ", ie[i]);
	}
	printk("\n");
}
#endif
#define IEEE80211_SCAN_RESULT_EXPIRE (3 * HZ)
/*****************************************************************************/
/*                                                                           */
/*  Function Name : trout_cfg80211_report_connect_result                     */
/*                                                                           */
/*  Description   : This function reports connect result to upper layer      */
/*                                                                           */
/*  Inputs        : dev - pointer to struct net_device                       */
/*                  bssid - BSSID connected to                               */
/*                  req_ie - pointer to association request IE               */
/*                  req_ie_len - association request IE length               */
/*                  resp_ie - pointer to association response IE             */
/*                  resp_ie_len - association response IE length             */
/*                  status - connection result                               */
/*  Globals       : None                                                     */
/*  Returns       : 0 on success, error code if failed                       */
/*  Issues        : Debugging                                                */
/*                                                                           */
/*****************************************************************************/
int trout_cfg80211_report_connect_result(struct net_device *dev, u8 *bssid, u8 *req_ie, u32 req_ie_len
		, u8* resp_ie, u32 resp_ie_len, u16 status)
{
	//struct cfg80211_bss *bss;
	//struct trout_private *stp;
	//struct cfg80211_internal_bss *rbss, *res;
	//struct wiphy *wiphy;
	//unsigned long now = jiffies;

	if (dev == NULL)
	{
		return -EINVAL;
	}

	TRACE_FUNC();
	//Since IEEE80211_SCAN_RESULT_EXPIRE is 3 seconds(later kernel is 30 seconds) which is too short
	// report all BSSes to CFG80211 again to keep cfg80211_get_bss() happy
	//TODO: consider reporting only one BSS with the specific BSSID 
	trout_cfg80211_report_scan_done(dev, 0);


	//cfg80211_connect_result() will handle all cases for NULL pointers
	//TODO: Check current context to see if we're in interrupt and decide GFP_XXX
	cfg80211_connect_result(dev, bssid, req_ie, req_ie_len, resp_ie, resp_ie_len, status, GFP_KERNEL);

	return 0;
}

//----------Static routines for trout_cfg80211_android_priv_cmd----------

static const char *android_wifi_cmd_str[ANDROID_WIFI_CMD_MAX] = {
	"START",
	"STOP",
	"SCAN-ACTIVE",
	"SCAN-PASSIVE",
	"RSSI",
	"LINKSPEED",
	"RXFILTER-START",
	"RXFILTER-STOP",
	"RXFILTER-ADD",
	"RXFILTER-REMOVE",
	"BTCOEXSCAN-START",
	"BTCOEXSCAN-STOP",
	"BTCOEXMODE",
	"SETSUSPENDOPT",
	"P2P_DEV_ADDR",
	"SETFWPATH",
	"SETBAND",
	"GETBAND",
	"COUNTRY",
	"P2P_SET_NOA",
	"P2P_GET_NOA",
	"P2P_SET_PS",
	"SET_AP_WPS_P2P_IE",
#ifdef PNO_SUPPORT
	"PNOSSIDCLR",
	"PNOSETUP ",
	"PNOFORCE",
	"PNODEBUG",
#endif

	"MACADDR",

	"BLOCK",
	"WFD-ENABLE",
	"WFD-DISABLE",
	"WFD-SET-TCPPORT",
	"WFD-SET-MAXTPUT",
	"WFD-SET-DEVTYPE",

	//yangke, 2013-10-07, add for TPC power control
	"SET_MAX_POWER",
	"UNSET_MAX_POWER",	
	"DHCP_STATUS", 
};

static int trout_android_cmdstr_to_num(char *cmdstr)
{
	int cmd_num;
	for(cmd_num=0 ; cmd_num<ANDROID_WIFI_CMD_MAX; cmd_num++)
		if(0 == strnicmp(cmdstr , android_wifi_cmd_str[cmd_num], strlen(android_wifi_cmd_str[cmd_num])) )
			break;

	return cmd_num;
}

#ifdef IBSS_BSS_STATION_MODE
extern UWORD32 g_pwr_tpc_switch; //add for TPC switch mode
#endif

/*****************************************************************************/
/*                                                                           */
/*  Function Name : trout_cfg80211_android_priv_cmd                          */
/*                                                                           */
/*  Description   : This function handles android private commands           */
/*                                                                           */
/*  Inputs        : dev - pointer to struct net_device                       */
/*                  req - pointer to struct ifreq                            */
/*  Globals       : None                                                     */
/*  Returns       : 0 on success,otherwise error code is returned            */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
int trout_cfg80211_android_priv_cmd(struct net_device *dev, struct ifreq *req)
{
	int ret = 0;
	int len = 0;
	char *command = NULL;
	int cmd_num;
	android_wifi_priv_cmd priv_cmd;

	/*leon liu added, we don't use private command for now*/
	//mark by yangke, 2013-10-08, reopen private command interface
	//return 0;

	if (req->ifr_data == NULL)
	{
		printk("No android private command found\n");
		ret = -EINVAL;
		goto exit;
	}

	if (copy_from_user(&priv_cmd, req->ifr_data, sizeof(android_wifi_priv_cmd)))
	{
		printk("copy_from_user() failed\n");
		ret = -EFAULT;
		goto exit;
	}

	command = kzalloc(priv_cmd.total_len, GFP_KERNEL);

	if (command == NULL)
	{
		printk("No free memory\n");
		ret = -ENOMEM;
		goto exit;
	}

	if (!access_ok(VERIFY_READ, priv_cmd.buf, priv_cmd.total_len))
	{
		printk("Failed to access android private command buffer\n");
		ret = -EFAULT;
		goto exit;
	}

	if (copy_from_user(command, (void *)priv_cmd.buf, priv_cmd.total_len))
	{
		printk("copy_from_user() failed: command buffer\n");
		ret = -EFAULT;
		goto exit;
	}

	/*junbinwang add for set dhcp or dhcp renew to driver. 20131223*/
	if(0 == strncmp(command , android_wifi_cmd_str[ANDROID_WIFI_CMD_DHCP_STATUS], 
			strlen(android_wifi_cmd_str[ANDROID_WIFI_CMD_DHCP_STATUS])) )
	{
		if(0 == strncmp(command , "DHCP_STATUS 1",  strlen("DHCP_STATUS 1")))
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
		sprintf(command, "OK");
		if (copy_to_user(priv_cmd.buf, command, min(priv_cmd.total_len, (u16)(strlen(command)+1)) ) )
			return -EFAULT;
		return 0;
	}
	
	cmd_num = trout_android_cmdstr_to_num(command);

	printk("%s, Android private command: %s(%d)\n", __func__, command, cmd_num);

	switch (cmd_num)
	{
		case ANDROID_WIFI_CMD_START:
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifdef WIFI_SLEEP_POLICY
			if(g_wifi_suspend_status == wifi_suspend_suspend) { // deep sleep
				g_wifi_suspend_status = wifi_suspend_nosuspend;
			}
#endif
#endif

#if 0
			//xuanyang 2013.4.24 supplicant timeout handle
			CHECK_MAC_RESET_IN_IW_HANDLER;
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
			/*leon liu marked, check if this event should post to host and how it should be done*/
			/*indicate_custom_event("START");*/
			break;
		case ANDROID_WIFI_CMD_STOP:
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifdef WIFI_SLEEP_POLICY
			pr_info("======== STOP ========\n");
			//xuanyang 2013.4.24 supplicant timeout handle
			CHECK_MAC_RESET_IN_IW_HANDLER(LPM_ACCESS);

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
			/*leon liu marked, check if this event should post to host and how it should be done*/
			/*indicate_custom_event("STOP");*/

			break;
		case ANDROID_WIFI_CMD_RSSI:
			{
				UWORD8 essid[IW_ESSID_MAX_SIZE + 1] = {0};
				int rssi = 0;
				
				//xuanyang 2013.4.24 supplicant timeout handle
				if( (BOOL_T)atomic_read(&g_mac_reset_done) == BFALSE )
				{
					PRINTK_ITMIW("%s not do,in reset process\n",__func__);
					sprintf(command, "OK");
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
					sprintf(command, "OK");
					break;
				}

				if(itm_get_essid(essid,IW_ESSID_MAX_SIZE) < 0)
				{
					sprintf(command, "OK");
					break;
				}
				#endif
				
				if(itm_get_rssi(&rssi) < 0)
				{
					sprintf(command, "OK");
					break;
				}

				sprintf(command, "%s rssi %d", essid, rssi);

			}
			break;
		case ANDROID_WIFI_CMD_LINKSPEED:
			{
				int mbps;

				//xuanyang 2013.4.24 supplicant timeout handle
				if( (BOOL_T)atomic_read(&g_mac_reset_done) == BFALSE )
				{
					PRINTK_ITMIW("%s not do,in reset process\n",__func__);
					sprintf(command, "LINKSPEED 0");
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
					sprintf(command, "LINKSPEED 0");
				}
				else if(IS_RATE_MCS(mbps) == BFALSE)
				{
					sprintf(command, "LINKSPEED %d",mbps);
				}
				else
				{
					int j=0;
					for( j = 0; j < sizeof(rate_table)/sizeof(rate_table_t); j++ )
					{
						if(rate_table[j].rate_index == mbps)
						{
							sprintf(command, "LINKSPEED %d", rate_table[j].rate);
							if(rate_table[j].point5)
							{
								strcat(command,".5");
							}
							break;
						}
					}

					if( j >= sizeof(rate_table)/sizeof(rate_table_t) )
					{
						sprintf(command, "LINKSPEED 0");
					}
				}				
			}
			break;
		case ANDROID_WIFI_CMD_MACADDR:
			{
				UWORD8 dev_addr[6] = {0};
			
				//xuanyang 2013.4.24 supplicant timeout handle
				if( (BOOL_T)atomic_read(&g_mac_reset_done) == BFALSE )
				{
					PRINTK_ITMIW("%s not do,in reset process\n",__func__);
				    sprintf(command, "MACADDR = " ITM_MAC_FMT, ITM_MAC_ARG(mget_StationID()));
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
			
				sprintf(command, "MACADDR = " ITM_MAC_FMT, ITM_MAC_ARG(dev_addr));
			}
			break;
		case ANDROID_WIFI_CMD_SCAN_ACTIVE:
			{
				sprintf(command, "OK");
			}
			break;
		case ANDROID_WIFI_CMD_SCAN_PASSIVE:
			{
				sprintf(command, "OK");
			}
			break;
		case ANDROID_WIFI_CMD_COUNTRY:
			{
				//TODO: impelement set country later
				sprintf(command, "OK");
			}
			break;
#ifdef IBSS_BSS_STATION_MODE
		//yangke, 2013-10-07, add for TPC power switch
		case ANDROID_WIFI_CMD_SET_MAX_POWER:
			{
				g_pwr_tpc_switch = 0;
				printk("request set max power, disable TPC feature.\n");
				//yangke, 2013-1010, for cmcc 11n signalling test
				write_dot11_phy_reg(0xFF, 0x00);
				write_dot11_phy_reg(0xF0, 0x65);
                            //yangke, 2013-10-24, switch off Q_empty check
                            disable_rx_buff_based_ack();
				sprintf(command, "OK");
			}
			break;
		case ANDROID_WIFI_CMD_UNSET_MAX_POWER:
			{
				g_pwr_tpc_switch = 1;
				printk("request unset max power, enable TPC feature.\n");
				//yangke, 2013-1010, for cmcc 11n signalling test
				write_dot11_phy_reg(0xFF, 0x00);
				write_dot11_phy_reg(0xF0, 0x01);
                            //yangke, 2013-10-24, switch on Q_empty check
                            enable_rx_buff_based_ack();
				sprintf(command, "OK");
			}
			break;
#endif
		default:
			sprintf(command, "OK");
			break;
	}

	if (strlen(command) == 0 && priv_cmd.total_len > 0)
	{
		command[0] = '\0';
		len = 1;
	}

	if (strlen(command) >= priv_cmd.total_len)
	{
		printk("Wrote %d bytes: larger than android private command buffer\n", strlen(command));
		len = priv_cmd.total_len;
	}
	else
	{
		len = strlen(command) + 1;
	}

	priv_cmd.used_len = len;

	if (copy_to_user((void *)priv_cmd.buf, command, len))
	{
		printk("copy_to_user() failed\n");
		ret = -EFAULT;
	}

exit:
	if (command != NULL)
	{
		kfree(command);
	}

	return ret;
}

/*junbinwang add for new report scan results. 2013-10-20*/
static int new_report_scan_results_fn(struct wiphy *wiphy)
{
	#ifdef IBSS_BSS_STATION_MODE
	int i =0;
	int ret = 0;
	u64 timestamp;
	UWORD8 *ie;
	UWORD16 ie_len;
	int signal = 0;
	//struct wiphy *wiphy;
	struct cfg80211_bss *i802_bss = NULL;
	struct ieee80211_channel *chan = NULL;
	bss_link_dscr_t* bss = NULL;
		
	if(NULL == wiphy){
		TROUT_DBG4("new_report_scan_results_fn: wiphy is null\n");
		return -1;
	}
	
	TROUT_DBG4("new_report_scan_results_fn fuc (sta mode)\n");

	bss = g_user_getscan_aplist;
	
	if(NULL == g_user_getscan_aplist )
	{
		TROUT_DBG4("linklist is null\n");
		return -E2BIG;
	}

	if(g_merge_aplist_flag == 1)
	{
		TROUT_DBG4("new_report_scan_results_fn is mergeing.\n");
		return 0;
	}

	while(bss != NULL)
	{
		timestamp = bss->bss_curr->time_stamp_msb;
		timestamp <<= 32;
		timestamp |= bss->bss_curr->time_stamp_lsb;
		
		ie = bss->generic_ie;
		ie_len = *(UWORD16 *)(&ie[0]);
		/*leon liu added, first two bytes of g_wpa_rsn_ie_buf_3 is length
		 * fixed IE retrieving
		*/
		ie += 2;
		chan = ieee80211_get_channel(wiphy, ieee80211_dsss_chan_to_freq(bss->bss_curr->channel));
		//Format signal level
		signal = -(0xFF - bss->bss_curr->rssi);
		signal = (signal < 63) ? signal : 63;
		signal = (signal > -192) ? signal : -192;
		signal = signal * 100;

		if (chan == NULL)
		{
			pr_info("Invalid channel %d\n", bss->bss_curr->channel);
			/*junbinwang modify 20131028, need to point next*/
	    	bss=bss->bss_next;
			continue;
		}
		
		pr_info("BSS %s -- %02x:%02x:%02x:%02x:%02x:%02x\n", bss->bss_curr->ssid,
				bss->bss_curr->bssid[0], bss->bss_curr->bssid[1], bss->bss_curr->bssid[2], 
				bss->bss_curr->bssid[3], bss->bss_curr->bssid[4], bss->bss_curr->bssid[5]);

		i802_bss = cfg80211_inform_bss(wiphy, chan,
				bss->bss_curr->bssid, timestamp, bss->bss_curr->cap_info, bss->bss_curr->beacon_period,
				ie, ie_len, signal, GFP_KERNEL);

		if (i802_bss != NULL)
		{
			cfg80211_put_bss(i802_bss);
		}
		else
		{
			printk("report ap fail\n");
		}
	    i++;
	    bss=bss->bss_next;
	}

	PRINTK_ITMIW("itm_giwscan fuc report to UI %d ap info\n",i);
	return ret;
	#else
	printk("ap mode\n");
	return 0;
	#endif
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : trout_cfg80211_report_scan_done                          */
/*                                                                           */
/*  Description   : This function reports scan done event to upper layer     */
/*                                                                           */
/*  Inputs        : dev - pointer to struct net_device                       */
/*                  aborted - if scanning is aborted or not                  */
/*  Globals       : None                                                     */
/*  Returns       : 0 on success, error code if failed                       */
/*  Issues        : Debugging                                                */
/*                                                                           */
/*****************************************************************************/
int trout_cfg80211_report_scan_done(struct net_device *dev, int aborted)
{
#ifdef IBSS_BSS_STATION_MODE
	struct trout_private *stp;
	struct wireless_dev *wdev;
	struct trout_wdev_priv *wdev_priv;
	struct wiphy *wiphy;
	int ret = 0;
	
	if (dev == NULL)
	{
		return -EINVAL;
	}

	stp = netdev_priv(dev);
	wdev = stp->wdev;
	wdev_priv = wiphy_priv(wdev->wiphy);
	wiphy = wdev->wiphy;

	if (aborted)
	{
		goto report_to_upper_layer;
	}

	/*
	 *FIXME: since itm_scan_task is no longer used, the codes related to it are removed
	 *Inform upper layer about all the BSSes found
	 */
	if (is_scanlist_report2ui == 1)
	{
		printk("Scan list is not reported to UI\n");
		aborted = true;
		goto report_to_upper_layer;
	}

	/*junbinwang modiy for report ap to uplayer by new way 2013-10-20*/
	//new_report_scan_results_fn(wiphy);
	
report_to_upper_layer:
	//Report scan done
	if ((BOOL_T)atomic_read(&g_mac_reset_done) == BTRUE){
		/*junbinwang modify for cr 238822. 20131128*/
		new_report_scan_results_fn(wiphy);
	}

	spin_lock_bh(&wdev_priv->scan_req_lock);

	if (wdev_priv->scan_request != NULL)
	{
		if (wdev_priv->scan_request->wiphy != wdev->wiphy)
		{
			printk("Wrong scan request for wiphy device");
		}
		else
		{
			printk("Reporting SCAN(aborted = %d) to upper layer\n", aborted);
			cfg80211_scan_done(wdev_priv->scan_request, aborted);
		}

		wdev_priv->scan_request = NULL;
	}
	else
	{
		printk("%s: No scan request from wiphy device\n", __func__);
	}

	spin_unlock_bh(&wdev_priv->scan_req_lock);

	return ret;
#else /* not in IBSS_BSS_STATION_MODE*/
	TRACE_FUNC();
	printk("%s in AP mode\n", __func__);
	return 0;
#endif
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : trout_cfg80211_del_prev_bss                          */
/*                                                                           */
/*  Description   : This function deletes previous connected bss from wpa_supplicant */
/*                                                                           */
/*  Inputs        : dev - pointer to struct net_device                       */

/*  Globals       : None                                                     */
/*  Returns       : 0 on success, error code if failed                       */
/*  Issues        : Debugging                                                */
/*                                                                           */
/*****************************************************************************/
// Add by Yiming.Li at 2014-01-07 for fix bug: reconnect
int trout_cfg80211_del_prev_bss(struct net_device *dev)
{
   struct cfg80211_bss *bss = NULL;
   struct trout_private *t_private;
	
    if (dev == NULL)
    {
	  return -EINVAL;
    }

    t_private = netdev_priv(dev);
    bss = cfg80211_get_bss(t_private->wdev->wiphy, NULL,  t_private->bssid, t_private->ssid,  
				t_private->ssid_len,  WLAN_CAPABILITY_ESS,  WLAN_CAPABILITY_ESS);

    if (bss) {
   	    printk("%s delete BSSID-- %02x:%02x:%02x:%02x:%02x:%02x\n", __func__,
			bss->bssid[0], bss->bssid[1], bss->bssid[2], 
			bss->bssid[3], bss->bssid[4], bss->bssid[5]);
  	    cfg80211_unlink_bss(t_private->wdev->wiphy, bss);
	    return 0;
    }
    else{
          printk("%s failed to delete previous BSS !\n", __func__);
	   return -1;
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : trout_wdev_alloc                                         */
/*                                                                           */
/*  Description   : This function allocate a new wireless_dev and a new      */
/*					wiphy and register the wiphy device.                     */
/*                                                                           */
/*  Inputs        : trout_priv - pointer to struct trout_private             */
/*					dev - pointer to a bus device(e.g: sdio_func->dev)		 */
/*  Globals       : None                                                     */
/*  Returns       : 0 on success, error code if failed                       */
/*  Issues        : Must be called before register_netdev and after 		 */
/*					member dev in trout_private(net_device)has been allocated*/
/*                                                                           */
/*****************************************************************************/
struct wireless_dev *g_wdev;

int trout_wdev_alloc(struct trout_private *trout_priv, struct device *dev)
{
	int ret = 0;
	struct wireless_dev *wdev;
	struct trout_wdev_priv *wdev_priv;
	struct net_device *netdev;

	if (trout_priv == NULL || trout_priv->dev == NULL)
	{
		return -ENODEV;
	}

	netdev = trout_priv->dev;

	//Allocate wireless_dev
	wdev = (struct wireless_dev *)kzalloc(sizeof(struct wireless_dev), GFP_KERNEL);

	if (wdev == NULL)
	{
		pr_err("Cannot allocate wireless device\n");
		return -ENOMEM;
	}

	//Allocate wiphy
	wdev->wiphy = wiphy_new(&trout_cfg80211_ops, sizeof(struct trout_wdev_priv));

	if (wdev->wiphy == NULL)
	{
		pr_err("Cannot allocate wiphy device\n");
		ret = -ENOMEM;
		goto out_free_wdev;
	}

	set_wiphy_dev(wdev->wiphy, dev);

	trout_priv->wdev = wdev;
	g_wdev = wdev;
	netdev->ieee80211_ptr = wdev;

	//Init wdev_priv
	wdev_priv = wiphy_priv(wdev->wiphy);	
	wdev_priv->trout_wdev = wdev;
	wdev_priv->trout_priv = trout_priv;
	wdev_priv->scan_request = NULL;
	spin_lock_init(&wdev_priv->scan_req_lock);

	wdev->netdev = netdev;
#ifdef IBSS_BSS_STATION_MODE
	wdev->iftype = NL80211_IFTYPE_STATION;
#endif
#ifdef BSS_ACCESS_POINT_MODE
	wdev->iftype = NL80211_IFTYPE_AP;
#endif

	//Init wiphy parameters
	init_wiphy_parameters(trout_priv, wdev->wiphy);

	//register wiphy
	ret = wiphy_register(wdev->wiphy);

	if (ret < 0)
	{
		pr_err("Cannot register wiphy device\n");
		goto out_free_wiphy;
	}

	SET_NETDEV_DEV(netdev, wiphy_dev(wdev->wiphy));

	printk("Linux wiphy device registered successfully(stp is %p wdev is %p wdev_priv->wdev is %p netdev->ieee80211_ptr is %p &stp->wdev is %p)\n"
			, trout_priv, trout_priv->wdev, wdev_priv->trout_wdev, netdev->ieee80211_ptr, &trout_priv->wdev);

	wmb();
	return 0;

out_free_wiphy:
	wiphy_free(wdev->wiphy);
out_free_wdev:
	kfree(wdev);
	return ret;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : trout_wdev_free                                          */
/*                                                                           */
/*  Description   : This function unregisters wiphy device and releases its  */
/*   				wireless_dev and the wiphy device.                       */
/*                                                                           */
/*  Inputs        : wdev - pointer to struct wireless_dev                    */
/*  Globals       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : Must be called after unregister_netdev                    */
/*                                                                           */
/*****************************************************************************/
void trout_wdev_free(struct wireless_dev *wdev)
{
	struct trout_wdev_priv *wdev_priv;

	//Debug message
	printk("Entered %s\n", __func__);
	if (wdev == NULL)
	{
		return ;
	}

	wdev_priv = wiphy_priv(wdev->wiphy);

	spin_lock_bh(&wdev_priv->scan_req_lock);

	if (wdev_priv->scan_request != NULL)
	{
		if (wdev_priv->scan_request->wiphy != wdev_priv->trout_wdev->wiphy )
		{
			TROUT_DBG5("Scan request is from a wrong wiphy device\n");
		}
		else
		{
			//If there's a pending scan request,abort it
			cfg80211_scan_done(wdev_priv->scan_request, 1);
		}

		wdev_priv->scan_request = NULL;
	}

	spin_unlock_bh(&wdev_priv->scan_req_lock);

	wiphy_unregister(wdev->wiphy);
	wiphy_free(wdev->wiphy);

	kfree(wdev);
}
#else
int trout_cfg80211_report_connect_result(struct net_device *dev, u8 *bssid, u8 *req_ie, u32 req_ie_len
		, u8* resp_ie, u32 resp_ie_len, u16 status)
{
	printk("%s: NULL debug\n", __func__);
	return 0;
}

int trout_cfg80211_report_scan_done(struct net_device *dev, int aborted)
{
	printk("%s: NULL debug\n", __func__);
	return 0;
}

int trout_wdev_alloc(struct trout_private *trout_priv, struct device *dev)
{
	printk("%s: NULL debug\n", __func__);
	return 0;
}

void trout_wdev_free(struct wireless_dev *wdev)
{
	printk("%s: NULL debug\n", __func__);
}

int trout_cfg80211_android_priv_cmd(struct net_device *dev, struct ifreq *req)
{
	printk("%s: NULL debug\n", __func__);
	return 0;
}


#endif

#endif /*CONFIG_CFG80211*/
