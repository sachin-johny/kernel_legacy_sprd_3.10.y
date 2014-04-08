#ifndef __ITM_WIFI_IW_H__
#define __ITM_WIFI_IW_H__ 

#include <linux/kernel.h>
#include <net/iw_handler.h>
#include "iconfig.h"

#define MAX_STRING_VAL_LEN    64
#define WID_HEAD_LEN 3
#define WID_CHAR_CFG_LEN 4
#define WID_SHORT_CFG_LEN   5
#define WID_INT_CFG_LEN 7

#define BSS_TYPE_STA 0
#define IBSS_TYPE_STA 1
#define AP_TYPE_AP 2

#define DEVICE_MODE_Not_Configured 0
#define DEVICE_MODE_Access_Point 1
#define DEVICE_MODE_BSS_STA 2
#define DEVICE_MODE_IBSS_STA 3
#define DEVICE_MODE_P2P_GO 4
#define DEVICE_MODE_P2P_Device 5
#define DEVICE_MODE_P2P_Client 6

#define NUM_WEPKEYS 4;
#define LARGE_KEY_SIZE 13
#define SMALL_KEY_SIZE 5

#define ITM_OPEN_SYSTEM 1
#define ITM_SHARED_KEY 2
#define ITM_WIFI_ROUTER_PORT_ACTION_8021X_PORT 5

#define No_ENCRYPTION 0
#define WEP64 0x03
#define WEP128 0x07
#define WPA_AES_PSK 0x29
#define WPA_TKIP_PSK 0x49
#define WPA_AES_TKIP_PSK 0x69
#define WPA2_AES_PSK 0x31
#define WPA2_TKIP_PSK 0x51
#define WPA2_AES_TKIP_PSK 0x71

#define Active_Mode 0
#define Minimum_FAST_Power_Save_Mode 1
#define Maximum_FAST_Power_Save_Mode 2
#define Minimum_PSPOLL_Power_Save_Mode 3
#define Maximum_PSPOLL_Power_Save_Mode 4

#define MAC_ELTID_ERP                    42
#define MAC_ELTID_RSN_IEEE               48				/* Proprietary   (0x30)          */
#define MAC_ELTID_EXT_RATES              50

#define MAC_ELTID_CHALLENGE              16

#define MAC_ELTID_OUI                   221            /* Proprietary   (0xDD)          */

#define MAC_OUI_TYPE_WPA                  1
#define MAC_OUI_TYPE_WME                  2
#define MAC_OUI_SUBTYPE_WME_INFORMATION   0
#define MAC_OUI_SUBTYPE_WME_PARAMETER     1
#define MAC_OUI_TYPE_WMMAC				  2

/*
* SUITE SELECTOR VALUES (WPA Extention)
*/
#define MAC_RSNIE_OUI_DEF       0x0050F2                /* WPA Default OUI value        */


/* Authentication and key managment suite selectors (table 1 p20)                       */
#define MAC_RSNIE_KEYMGT_8021X  1
#define MAC_RSNIE_KEYMGT_PSK    2

/* Descriptor values                                                                    */
/* [bit 0 1 2] Cipher suite selectors (table 2 p20)                                     */
#define MAC_RSNIE_CIPHER_MASK    0x07
#define MAC_RSNIE_CIPHER_WEP40   0x01
#define MAC_RSNIE_CIPHER_TKIP    0x02
#define MAC_RSNIE_CIPHER_CCMP    0x04
#define MAC_RSNIE_CIPHER_WEP104  0x05
#define MAC_RSNIE_CIPHER_NOSEC   0xFE
#define MAC_RSNIE_CIPHER_INVALID 0xFF

/*******************************************************************************
 VALUES
    CSR_WIFI_SME_AUTH_MODE_80211_OPEN
                   - Connects to an open system network (i.e. no authentication,
                     no encryption) or to a WEP enabled network.
    ITM_WIFI_SME_AUTH_MODE_80211_SHARED
                   - Connect to a WEP enabled network.
    ITM_WIFI_SME_AUTH_MODE_8021X_WPA
                   - Connects to a WPA Enterprise enabled network.
    ITM_WIFI_SME_AUTH_MODE_8021X_WPAPSK
                   - Connects to a WPA with Pre-Shared Key enabled network.
    ITM_WIFI_SME_AUTH_MODE_8021X_WPA2
                   - Connects to a WPA2 Enterprise enabled network.
    ITM_WIFI_SME_AUTH_MODE_8021X_WPA2PSK
                   - Connects to a WPA2 with Pre-Shared Key enabled network.
    ITM_WIFI_SME_AUTH_MODE_8021X_CCKM
                   - Connects to a CCKM enabled network.
    ITM_WIFI_SME_AUTH_MODE_WAPI_WAI
                   - Connects to a WAPI Enterprise enabled network.
    ITM_WIFI_SME_AUTH_MODE_WAPI_WAIPSK
                   - Connects to a WAPI with Pre-Shared Key enabled network.
    ITM_WIFI_SME_AUTH_MODE_8021X_OTHER1X
                   - For future use.

*******************************************************************************/
#define ITM_WIFI_SME_AUTH_MODE_80211_OPEN      ((UWORD16) 0x0001)
#define ITM_WIFI_SME_AUTH_MODE_80211_SHARED    ((UWORD16) 0x0002)
#define ITM_WIFI_SME_AUTH_MODE_8021X_WPA       ((UWORD16) 0x0004)
#define ITM_WIFI_SME_AUTH_MODE_8021X_WPAPSK    ((UWORD16) 0x0008)
#define ITM_WIFI_SME_AUTH_MODE_8021X_WPA2      ((UWORD16) 0x0010)
#define ITM_WIFI_SME_AUTH_MODE_8021X_WPA2PSK   ((UWORD16) 0x0020)
#define ITM_WIFI_SME_AUTH_MODE_8021X_CCKM      ((UWORD16) 0x0040)
#define ITM_WIFI_SME_AUTH_MODE_WAPI_WAI        ((UWORD16) 0x0080)
#define ITM_WIFI_SME_AUTH_MODE_WAPI_WAIPSK     ((UWORD16) 0x0100)
#define ITM_WIFI_SME_AUTH_MODE_8021X_OTHER1X   ((UWORD16) 0x0200)

/* Private IOCTLs */
#define SIOCIWS80211POWERSAVEPRIV           SIOCIWFIRSTPRIV
#define SIOCIWG80211POWERSAVEPRIV           SIOCIWFIRSTPRIV + 1
#define SIOCIWS80211RELOADDEFAULTSPRIV      SIOCIWFIRSTPRIV + 2
#define SIOCIWSCONFWAPIPRIV                 SIOCIWFIRSTPRIV + 4
#define SIOCIWSWAPIKEYPRIV                  SIOCIWFIRSTPRIV + 6
#define SIOCIWSSMEDEBUGPRIV                 SIOCIWFIRSTPRIV + 8
#define SIOCIWSAPCFGPRIV                    SIOCIWFIRSTPRIV + 10
#define SIOCIWSAPSTARTPRIV                  SIOCIWFIRSTPRIV + 12
#define SIOCIWSAPSTOPPRIV                   SIOCIWFIRSTPRIV + 14
#define SIOCIWSFWRELOADPRIV                 SIOCIWFIRSTPRIV + 16
#define SIOCIWSSTACKSTART                   SIOCIWFIRSTPRIV + 18
#define SIOCIWSSTACKSTOP                    SIOCIWFIRSTPRIV + 20


//chenq add for IC test
#define SIOCIWSICTEST                       SIOCIWFIRSTPRIV + 0

//chenq add for debug
#define SIOCIWG_FORDEBUG                     SIOCIWFIRSTPRIV + 3
#define SIOCIWS_FORDEBUG                     SIOCIWFIRSTPRIV + 2

#define SIOCIWS_FORWID						SIOCIWFIRSTPRIV + 4

//chenq add for iwconfig trout
#define SIOCIWS_IWCONFIG_TROUT              SIOCIWFIRSTPRIV + 6
#define SIOCIWG_IWCONFIG_TROUT				SIOCIWFIRSTPRIV + 7


#define IWPRIV_POWER_SAVE_MAX_STRING 32
#define IWPRIV_SME_DEBUG_MAX_STRING 32
#define IWPRIV_SME_MAX_STRING 120

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
} itm_wapi_key_t;

typedef union
{
    UWORD8  c_val;
    UWORD16 s_val;
    UWORD32 i_val;
    WORD8   str_val[MAX_STRING_VAL_LEN];
} for_iw_wid_val_t;

typedef struct
{
    UWORD16   id;    /* WID Identifier */
    UWORD8    type;  /* WID Data Type  */
    for_iw_wid_val_t value; /* WID Value      */
} for_iw_wid_struct_t;


//moved by chengwg for using in other files, 2013.7.9
#define ITMIW_DEBUD_PRINT
#ifdef ITMIW_DEBUD_PRINT
#define PRINTK_ITMIW(args...)  printk(args)
#else  
#define PRINTK_ITMIW(args...)
#endif 

#define LPM_ACCESS		1	//if in LPM mode, want to access Trout register, but we can not stop it
#define LPM_NO_ACCESS		0	//if in LPM mode, can not access
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifdef WIFI_SLEEP_POLICY
#define CHECK_MAC_RESET_IN_IW_HANDLER(expt) do{							\
	if( (BOOL_T)atomic_read(&g_mac_reset_done) == BFALSE )\
	{\
		PRINTK_ITMIW("%s not do,in reset process\n",__func__);\
		return -EBUSY;\
	}\
	if(g_wifi_power_mode){									\
		if(!expt){									\
			pr_info("LPM: exit %s\n", __func__);					\
			return -EBUSY;								\
		}										\
	}else{											\
	if (mutex_is_locked(&suspend_mutex) || (g_wifi_suspend_status != wifi_suspend_nosuspend)) {\
			pr_info("We can't do %s in SPD, gss = %d\n", __func__, g_wifi_suspend_status); \
		return -EBUSY;\
	}\
	}											\
}while(0)



#define CHECK_MAC_RESET_IN_IW_HANDLER_RETURN_NULL(expt) do{					\
	if( (BOOL_T)atomic_read(&g_mac_reset_done) == BFALSE )\
	{\
		PRINTK_ITMIW("%s not do,in reset process\n",__func__);\
		return NULL;\
	}\
	if(g_wifi_power_mode){									\
		if(!expt){									\
			pr_info("LPM: exit %s\n", __func__);					\
			return NULL;								\
		}										\
	}else{											\
	if (mutex_is_locked(&suspend_mutex) || (g_wifi_suspend_status != wifi_suspend_nosuspend)) {\
			pr_info("We can't do %s in SPD, gss = %d\n", __func__, g_wifi_suspend_status);	\
		return NULL;\
	}\
	}											\
}while(0)

#else

/* prevent everything when Wi-Fi is suspending/resuming, keguang 2013-5-4 */
#define CHECK_MAC_RESET_IN_IW_HANDLER(expt) do{\
	if( (BOOL_T)atomic_read(&g_mac_reset_done) == BFALSE )\
	{\
		PRINTK_ITMIW("%s not do,in reset process\n",__func__);\
		return -EBUSY;\
	}\
	if (g_wifi_suspend_status != wifi_suspend_nosuspend) {\
		pr_info("We can't do %s now: g_wifi_suspend_status = %d\n", __func__, g_wifi_suspend_status);\
		return -EBUSY;\
	}\
}while(0)

#define CHECK_MAC_RESET_IN_IW_HANDLER_RETURN_NULL(expt) do{\
	if( (BOOL_T)atomic_read(&g_mac_reset_done) == BFALSE )\
	{\
		PRINTK_ITMIW("%s not do,in reset process\n",__func__);\
		return NULL;\
	}\
	if (g_wifi_suspend_status != wifi_suspend_nosuspend) {\
		pr_info("We can't do %s now: g_wifi_suspend_status = %d\n", __func__, g_wifi_suspend_status);\
		return NULL;\
	}\
}while(0)
#endif

#else

#define CHECK_MAC_RESET_IN_IW_HANDLER(expt) do{\
	if( (BOOL_T)atomic_read(&g_mac_reset_done) == BFALSE )\
	{\
		PRINTK_ITMIW("%s not do,in reset process\n",__func__);\
		return -EBUSY;\
	}\
}while(0)

#define CHECK_MAC_RESET_IN_IW_HANDLER_RETURN_NULL(expt) do{\
	if( (BOOL_T)atomic_read(&g_mac_reset_done) == BFALSE )\
	{\
		PRINTK_ITMIW("%s not do,in reset process\n",__func__);\
		return NULL;\
	}\
}while(0)

#endif
INLINE int get_trour_rsp_data(UWORD8 * dest,UWORD16 dest_len,UWORD8 * src,UWORD16 src_len,int type)
{
	UWORD8    checksum = 0;
	UWORD16  rsp_idx = 2;
	UWORD16  wid_data_len = 0;
	UWORD16  dest_idx = 0;
	UWORD8   lenbyte_cnt=0;
	int i=0;
	
	switch(type)
	{
		case WID_CHAR:
			while( rsp_idx < src_len )
		    	{
		    		
		    	       //printk("WID:%02x  %02x\n",src[1] & 0xFF,src[0] & 0xFF);
				wid_data_len = src[rsp_idx] & 0xFF;
				//printk("wid_data_len: %d\n",wid_data_len);		
				if( wid_data_len != 1)
				{
					rsp_idx = 0;
					break;
				}
				
				if( (dest_idx + wid_data_len) <= dest_len )
				{
					rsp_idx++;
				}
				else
				{
					rsp_idx = 0;
					break;
				}
				dest[dest_idx] = src[rsp_idx];
				
				dest++;
				
				rsp_idx  += wid_data_len+2;
				dest_idx += wid_data_len;
			}
			break;
		case WID_SHORT:
			while( rsp_idx < src_len )
		    	{
		    		UWORD16 * tmp_shotr = ( UWORD16 * )dest;
		    	       //printk("WID:%02x  %02x\n",src[1] & 0xFF,src[0] & 0xFF);
				wid_data_len = src[rsp_idx] & 0xFF;
				//printk("wid_data_len: %d\n",wid_data_len);		
				if( wid_data_len != 2)
				{
					rsp_idx = 0;
					break;
				}
				
				if( (dest_idx + wid_data_len) <= dest_len )
				{
					rsp_idx++;
				}
				else
				{
					rsp_idx = 0;
					break;
				}
				*tmp_shotr   = src[rsp_idx];
				*tmp_shotr  |= (src[rsp_idx+1]  << 8);
				
				tmp_shotr++;
				
				rsp_idx  += wid_data_len+2;
				dest_idx += wid_data_len;
			}
			break;
		case WID_INT:
			while( rsp_idx < src_len )
		    	{
		    	       UWORD32 * tmp_int = ( UWORD32 * )dest;
		    	       //printk("WID:%02x  %02x\n",src[1] & 0xFF,src[0] & 0xFF);
				wid_data_len = src[rsp_idx] & 0xFF;
				//printk("wid_data_len: %d\n",wid_data_len);		
				if( wid_data_len != 4)
				{
					rsp_idx = 0;
					break;
				}
				
				if( (dest_idx + wid_data_len) <= dest_len )
				{
					rsp_idx++;
				}
				else
				{
					rsp_idx = 0;
					break;
				}

				*tmp_int   =  src[rsp_idx];
				*tmp_int |= (src[rsp_idx+1]  << 8);
				*tmp_int |= (src[rsp_idx+2]  << 16);
				*tmp_int |= (src[rsp_idx+3]  << 24);
				
				tmp_int++;
				
				rsp_idx  += wid_data_len+2;
				dest_idx += wid_data_len;
			}
			break;
		case WID_STR:	
			while( rsp_idx < src_len )
		    	{
		    	//printk("WID:%02x  %02x\n",src[1] & 0xFF,src[0] & 0xFF);
				if( (src[1] == 0x30) && (src[0] == 0x12) )
				{
					wid_data_len   = src[rsp_idx] & 0x00FF;
					wid_data_len |= (src[rsp_idx+1] << 8) & 0xFF00;
					lenbyte_cnt = 2;
				}
				else
				{
					wid_data_len = src[rsp_idx] & 0xFF;
					lenbyte_cnt = 1;
				}	
				//printk("wid_data_len: %d\n",wid_data_len);
				
				if( (dest_idx + wid_data_len) <= dest_len )
				{
					rsp_idx+=lenbyte_cnt;
				}
				else
				{
					rsp_idx = 0;
					break;
				}
				memcpy(&dest[dest_idx],&src[rsp_idx],wid_data_len);
			/*
				printk("idx = %d,get_trour_rsp_data:",dest_idx);
				for(i=0;i<wid_data_len;i++)
					printk("0x%02x ",dest[dest_idx+i]);
				printk("\n");
			*/	
				rsp_idx  += wid_data_len + 2 + ((lenbyte_cnt == 1) ? 0 : 1);
				dest_idx += wid_data_len;
			}
			break;
		
              	case WID_BIN_DATA:
				while( rsp_idx < src_len )
			    	{
					wid_data_len   = src[rsp_idx] & 0x00FF;
					wid_data_len |= (src[rsp_idx+1] << 8) & 0xFF00;
					if( (dest_idx+=wid_data_len) <= dest_len )
					{
						rsp_idx+=2;
					}
					else
					{
						rsp_idx = 0;
						break;
					}
					
					for(i=0;i<wid_data_len;i++)
					{
						checksum += src[rsp_idx+i];
					}
					if( checksum != src[rsp_idx+i] )
					{
						rsp_idx = 0;
						break;
					}
					memcpy(&dest[dest_idx],&src[rsp_idx],wid_data_len);
					rsp_idx  += wid_data_len+1+2;
					dest_idx += wid_data_len;
				}
			break;		
	}

	return dest_idx & 0x0000FFFF;
}

int itm_get_dhcp_status();
void itm_set_dhcp_status(int status);
#endif/*__ITM_WIFI_IW_H__*/
