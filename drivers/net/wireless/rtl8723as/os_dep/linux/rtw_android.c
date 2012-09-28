/******************************************************************************
 *
 * Copyright(c) 2007 - 2011 Realtek Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110, USA
 *
 *
 ******************************************************************************/

#include <linux/module.h>
#include <linux/netdevice.h>

#include <rtw_android.h>
#include <osdep_service.h>
#include <rtw_debug.h>
#include <ioctl_cfg80211.h>
#include <rtw_ioctl_set.h>

const char *android_wifi_cmd_str[ANDROID_WIFI_CMD_MAX] = {
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

};

#ifdef PNO_SUPPORT
#define PNO_TLV_PREFIX			'S'
#define PNO_TLV_VERSION			'1'
#define PNO_TLV_SUBVERSION 		'2'
#define PNO_TLV_RESERVED		'0'
#define PNO_TLV_TYPE_SSID_IE		'S'
#define PNO_TLV_TYPE_TIME		'T'
#define PNO_TLV_FREQ_REPEAT		'R'
#define PNO_TLV_FREQ_EXPO_MAX		'M'

typedef struct cmd_tlv {
	char prefix;
	char version;
	char subver;
	char reserved;
} cmd_tlv_t;
#endif /* PNO_SUPPORT */

typedef struct android_wifi_priv_cmd {

#ifdef CONFIG_COMPAT
	compat_uptr_t buf;
#else
	char *buf;
#endif

	int used_len;
	int total_len;
} android_wifi_priv_cmd;

/**
 * Local (static) functions and variables
 */

/* Initialize g_wifi_on to 1 so dhd_bus_start will be called for the first
 * time (only) in dhd_open, subsequential wifi on will be handled by
 * wl_android_wifi_on
 */
static int g_wifi_on = _TRUE;


#ifdef PNO_SUPPORT
static int wl_android_set_pno_setup(struct net_device *dev, char *command, int total_len)
{
	wlc_ssid_t ssids_local[MAX_PFN_LIST_COUNT];
	int res = -1;
	int nssid = 0;
	cmd_tlv_t *cmd_tlv_temp;
	char *str_ptr;
	int tlv_size_left;
	int pno_time = 0;
	int pno_repeat = 0;
	int pno_freq_expo_max = 0;

#ifdef PNO_SET_DEBUG
	int i;
	char pno_in_example[] = {
		'P', 'N', 'O', 'S', 'E', 'T', 'U', 'P', ' ',
		'S', '1', '2', '0',
		'S',
		0x05,
		'd', 'l', 'i', 'n', 'k',
		'S',
		0x04,
		'G', 'O', 'O', 'G',
		'T',
		'0', 'B',
		'R',
		'2',
		'M',
		'2',
		0x00
		};
#endif /* PNO_SET_DEBUG */

	DHD_INFO(("%s: command=%s, len=%d\n", __FUNCTION__, command, total_len));

	if (total_len < (strlen(CMD_PNOSETUP_SET) + sizeof(cmd_tlv_t))) {
		DHD_ERROR(("%s argument=%d less min size\n", __FUNCTION__, total_len));
		goto exit_proc;
	}

#ifdef PNO_SET_DEBUG
	memcpy(command, pno_in_example, sizeof(pno_in_example));
	for (i = 0; i < sizeof(pno_in_example); i++)
		printf("%02X ", command[i]);
	printf("\n");
	total_len = sizeof(pno_in_example);
#endif

	str_ptr = command + strlen(CMD_PNOSETUP_SET);
	tlv_size_left = total_len - strlen(CMD_PNOSETUP_SET);

	cmd_tlv_temp = (cmd_tlv_t *)str_ptr;
	memset(ssids_local, 0, sizeof(ssids_local));

	if ((cmd_tlv_temp->prefix == PNO_TLV_PREFIX) &&
		(cmd_tlv_temp->version == PNO_TLV_VERSION) &&
		(cmd_tlv_temp->subver == PNO_TLV_SUBVERSION)) {

		str_ptr += sizeof(cmd_tlv_t);
		tlv_size_left -= sizeof(cmd_tlv_t);

		if ((nssid = wl_iw_parse_ssid_list_tlv(&str_ptr, ssids_local,
			MAX_PFN_LIST_COUNT, &tlv_size_left)) <= 0) {
			DHD_ERROR(("SSID is not presented or corrupted ret=%d\n", nssid));
			goto exit_proc;
		} else {
			if ((str_ptr[0] != PNO_TLV_TYPE_TIME) || (tlv_size_left <= 1)) {
				DHD_ERROR(("%s scan duration corrupted field size %d\n",
					__FUNCTION__, tlv_size_left));
				goto exit_proc;
			}
			str_ptr++;
			pno_time = simple_strtoul(str_ptr, &str_ptr, 16);
			DHD_INFO(("%s: pno_time=%d\n", __FUNCTION__, pno_time));

			if (str_ptr[0] != 0) {
				if ((str_ptr[0] != PNO_TLV_FREQ_REPEAT)) {
					DHD_ERROR(("%s pno repeat : corrupted field\n",
						__FUNCTION__));
					goto exit_proc;
				}
				str_ptr++;
				pno_repeat = simple_strtoul(str_ptr, &str_ptr, 16);
				DHD_INFO(("%s :got pno_repeat=%d\n", __FUNCTION__, pno_repeat));
				if (str_ptr[0] != PNO_TLV_FREQ_EXPO_MAX) {
					DHD_ERROR(("%s FREQ_EXPO_MAX corrupted field size\n",
						__FUNCTION__));
					goto exit_proc;
				}
				str_ptr++;
				pno_freq_expo_max = simple_strtoul(str_ptr, &str_ptr, 16);
				DHD_INFO(("%s: pno_freq_expo_max=%d\n",
					__FUNCTION__, pno_freq_expo_max));
			}
		}
	} else {
		DHD_ERROR(("%s get wrong TLV command\n", __FUNCTION__));
		goto exit_proc;
	}

	res = dhd_dev_pno_set(dev, ssids_local, nssid, pno_time, pno_repeat, pno_freq_expo_max);

exit_proc:
	return res;
}
#endif /* PNO_SUPPORT */

int rtw_android_cmdstr_to_num(char *cmdstr)
{
	int cmd_num;
	for(cmd_num=0 ; cmd_num<ANDROID_WIFI_CMD_MAX; cmd_num++)
		if(0 == strnicmp(cmdstr , android_wifi_cmd_str[cmd_num], strlen(android_wifi_cmd_str[cmd_num])) )
			break;

	return cmd_num;
}

int rtw_android_get_rssi(struct net_device *net, char *command, int total_len)
{
	_adapter *padapter = (_adapter *)rtw_netdev_priv(net);
	struct	mlme_priv	*pmlmepriv = &(padapter->mlmepriv);
	struct	wlan_network	*pcur_network = &pmlmepriv->cur_network;
	int bytes_written = 0;

	if(check_fwstate(pmlmepriv, _FW_LINKED) == _TRUE) {
		bytes_written += snprintf(&command[bytes_written], total_len, "%s rssi %d",
			pcur_network->network.Ssid.Ssid, padapter->recvpriv.rssi);
	}

	return bytes_written;
}

int rtw_android_get_link_speed(struct net_device *net, char *command, int total_len)
{
	_adapter *padapter = (_adapter *)rtw_netdev_priv(net);
	struct	mlme_priv	*pmlmepriv = &(padapter->mlmepriv);
	struct	wlan_network	*pcur_network = &pmlmepriv->cur_network;
	int bytes_written = 0;
	u16 link_speed = 0;

	link_speed = rtw_get_network_max_rate(padapter, &pcur_network->network);
	bytes_written = snprintf(command, total_len, "LinkSpeed %d", link_speed);

	return bytes_written;
}

int rtw_android_get_macaddr(struct net_device *net, char *command, int total_len)
{
	_adapter *adapter = (_adapter *)rtw_netdev_priv(net);
	int bytes_written = 0;

	bytes_written = snprintf(command, total_len, "Macaddr = "MAC_FMT, MAC_ARG(net->dev_addr));
	return bytes_written;
}

int rtw_android_set_country(struct net_device *net, char *command, int total_len)
{
	_adapter *adapter = (_adapter *)rtw_netdev_priv(net);
	char *country_code = command + strlen(android_wifi_cmd_str[ANDROID_WIFI_CMD_COUNTRY]) + 1;
	int ret;

	ret = rtw_set_country(adapter, country_code);

	return (ret==_SUCCESS)?0:-1;
}

int rtw_android_get_p2p_dev_addr(struct net_device *net, char *command, int total_len)
{
	int ret;
	int bytes_written = 0;

	//We use the same address as our HW MAC address
	_rtw_memcpy(command, net->dev_addr, ETH_ALEN);

	bytes_written = ETH_ALEN;
	return bytes_written;
}

int rtw_android_set_block(struct net_device *net, char *command, int total_len)
{
	int ret;
	_adapter *adapter = (_adapter *)rtw_netdev_priv(net);
	char *block_value = command + strlen(android_wifi_cmd_str[ANDROID_WIFI_CMD_BLOCK]) + 1;

	#ifdef CONFIG_IOCTL_CFG80211
	wdev_to_priv(adapter->rtw_wdev)->block = (*block_value=='0')?_FALSE:_TRUE;
	#endif

	return 0;
}

int rtw_android_priv_cmd(struct net_device *net, struct ifreq *ifr, int cmd)
{
	int ret = 0;
	char *command = NULL;
	int cmd_num;
	int bytes_written = 0;
	android_wifi_priv_cmd priv_cmd;

	rtw_lock_suspend();

	if (!ifr->ifr_data) {
		ret = -EINVAL;
		goto exit;
	}
	if (copy_from_user(&priv_cmd, ifr->ifr_data, sizeof(android_wifi_priv_cmd))) {
		ret = -EFAULT;
		goto exit;
	}

	//DBG_871X("%s priv_cmd.buf=%p priv_cmd.total_len=%d  priv_cmd.used_len=%d\n",__func__,priv_cmd.buf,priv_cmd.total_len,priv_cmd.used_len);
	command = kmalloc(priv_cmd.total_len, GFP_KERNEL);
	if (!command)
	{
		DBG_871X("%s: failed to allocate memory\n", __FUNCTION__);
		ret = -ENOMEM;
		goto exit;
	}

	if (!access_ok(VERIFY_READ, priv_cmd.buf, priv_cmd.total_len)){
	 	DBG_871X("%s: failed to access memory\n", __FUNCTION__);
		ret = -EFAULT;
		goto exit;
	 }
	if (copy_from_user(command, (void *)priv_cmd.buf, priv_cmd.total_len)) {
		ret = -EFAULT;
		goto exit;
	}

	DBG_871X("%s: Android private cmd \"%s\" on %s\n"
		, __FUNCTION__, command, ifr->ifr_name);

	cmd_num = rtw_android_cmdstr_to_num(command);

	switch(cmd_num) {
	case ANDROID_WIFI_CMD_START:
		//bytes_written = wl_android_wifi_on(net);
		goto response;
	case ANDROID_WIFI_CMD_SETFWPATH:
		goto response;
	}

	if (!g_wifi_on) {
		DBG_871X("%s: Ignore private cmd \"%s\" - iface %s is down\n"
			,__FUNCTION__, command, ifr->ifr_name);
		ret = 0;
		goto exit;
	}

	switch(cmd_num) {

	case ANDROID_WIFI_CMD_STOP:
		//bytes_written = wl_android_wifi_off(net);
		break;

	case ANDROID_WIFI_CMD_SCAN_ACTIVE:
		rtw_set_scan_mode((_adapter *)rtw_netdev_priv(net), SCAN_ACTIVE);
		break;
	case ANDROID_WIFI_CMD_SCAN_PASSIVE:
		rtw_set_scan_mode((_adapter *)rtw_netdev_priv(net), SCAN_PASSIVE);
		break;

	case ANDROID_WIFI_CMD_RSSI:
		bytes_written = rtw_android_get_rssi(net, command, priv_cmd.total_len);
		break;
	case ANDROID_WIFI_CMD_LINKSPEED:
		bytes_written = rtw_android_get_link_speed(net, command, priv_cmd.total_len);
		break;

	case ANDROID_WIFI_CMD_MACADDR:
		bytes_written = rtw_android_get_macaddr(net, command, priv_cmd.total_len);
		break;

	case ANDROID_WIFI_CMD_BLOCK:
		bytes_written = rtw_android_set_block(net, command, priv_cmd.total_len);
		break;

	case ANDROID_WIFI_CMD_RXFILTER_START:
		//bytes_written = net_os_set_packet_filter(net, 1);
		break;
	case ANDROID_WIFI_CMD_RXFILTER_STOP:
		//bytes_written = net_os_set_packet_filter(net, 0);
		break;
	case ANDROID_WIFI_CMD_RXFILTER_ADD:
		//int filter_num = *(command + strlen(CMD_RXFILTER_ADD) + 1) - '0';
		//bytes_written = net_os_rxfilter_add_remove(net, TRUE, filter_num);
		break;
	case ANDROID_WIFI_CMD_RXFILTER_REMOVE:
		//int filter_num = *(command + strlen(CMD_RXFILTER_REMOVE) + 1) - '0';
		//bytes_written = net_os_rxfilter_add_remove(net, FALSE, filter_num);
		break;

	case ANDROID_WIFI_CMD_BTCOEXSCAN_START:
		/* TBD: BTCOEXSCAN-START */
		break;
	case ANDROID_WIFI_CMD_BTCOEXSCAN_STOP:
		/* TBD: BTCOEXSCAN-STOP */
		break;
	case ANDROID_WIFI_CMD_BTCOEXMODE:
		#if 0
		uint mode = *(command + strlen(CMD_BTCOEXMODE) + 1) - '0';
		if (mode == 1)
			net_os_set_packet_filter(net, 0); /* DHCP starts */
		else
			net_os_set_packet_filter(net, 1); /* DHCP ends */
#ifdef WL_CFG80211
		bytes_written = wl_cfg80211_set_btcoex_dhcp(net, command);
#endif
		#endif
		break;

	case ANDROID_WIFI_CMD_SETSUSPENDOPT:
		//bytes_written = wl_android_set_suspendopt(net, command, priv_cmd.total_len);
		break;

	case ANDROID_WIFI_CMD_SETBAND:
		//uint band = *(command + strlen(CMD_SETBAND) + 1) - '0';
		//bytes_written = wldev_set_band(net, band);
		break;
	case ANDROID_WIFI_CMD_GETBAND:
		//bytes_written = wl_android_get_band(net, command, priv_cmd.total_len);
		break;

	case ANDROID_WIFI_CMD_COUNTRY:
		bytes_written = rtw_android_set_country(net, command, priv_cmd.total_len);
		break;

#ifdef PNO_SUPPORT
	case ANDROID_WIFI_CMD_PNOSSIDCLR_SET:
		//bytes_written = dhd_dev_pno_reset(net);
		break;
	case ANDROID_WIFI_CMD_PNOSETUP_SET:
		//bytes_written = wl_android_set_pno_setup(net, command, priv_cmd.total_len);
		break;
	case ANDROID_WIFI_CMD_PNOENABLE_SET:
		//uint pfn_enabled = *(command + strlen(CMD_PNOENABLE_SET) + 1) - '0';
		//bytes_written = dhd_dev_pno_enable(net, pfn_enabled);
		break;
#endif

	case ANDROID_WIFI_CMD_P2P_DEV_ADDR:
		bytes_written = rtw_android_get_p2p_dev_addr(net, command, priv_cmd.total_len);
		break;
	case ANDROID_WIFI_CMD_P2P_SET_NOA:
		//int skip = strlen(CMD_P2P_SET_NOA) + 1;
		//bytes_written = wl_cfg80211_set_p2p_noa(net, command + skip, priv_cmd.total_len - skip);
		break;
	case ANDROID_WIFI_CMD_P2P_GET_NOA:
		//bytes_written = wl_cfg80211_get_p2p_noa(net, command, priv_cmd.total_len);
		break;
	case ANDROID_WIFI_CMD_P2P_SET_PS:
		//int skip = strlen(CMD_P2P_SET_PS) + 1;
		//bytes_written = wl_cfg80211_set_p2p_ps(net, command + skip, priv_cmd.total_len - skip);
		break;

#ifdef CONFIG_IOCTL_CFG80211
	case ANDROID_WIFI_CMD_SET_AP_WPS_P2P_IE:
	{
		int skip = strlen(android_wifi_cmd_str[ANDROID_WIFI_CMD_SET_AP_WPS_P2P_IE]) + 3;
		bytes_written = rtw_cfg80211_set_mgnt_wpsp2pie(net, command + skip, priv_cmd.total_len - skip, *(command + skip - 2) - '0');
		break;
	}
#endif //CONFIG_IOCTL_CFG80211

	default:
		DBG_871X("Unknown PRIVATE command %s - ignored\n", command);
		snprintf(command, 3, "OK");
		bytes_written = strlen("OK");
	}

response:
	if (bytes_written >= 0) {
		if ((bytes_written == 0) && (priv_cmd.total_len > 0))
			command[0] = '\0';
		if (bytes_written >= priv_cmd.total_len) {
			DBG_871X("%s: bytes_written = %d\n", __FUNCTION__, bytes_written);
			bytes_written = priv_cmd.total_len;
		} else {
			bytes_written++;
		}
		priv_cmd.used_len = bytes_written;
		if (copy_to_user((void *)priv_cmd.buf, command, bytes_written)) {
			DBG_871X("%s: failed to copy data to user buffer\n", __FUNCTION__);
			ret = -EFAULT;
		}
	}
	else {
		ret = bytes_written;
	}

exit:
	rtw_unlock_suspend();
	if (command) {
		kfree(command);
	}

	return ret;
}
