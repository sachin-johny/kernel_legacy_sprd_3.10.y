/******************************************************************************
 *
 * Copyright(c) 2007 - 2012 Realtek Corporation. All rights reserved.
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

#define _HAL_INTF_C_
#include <drv_conf.h>
#include <osdep_service.h>
#include <drv_types.h>
#include <rtw_byteorder.h>

#include <hal_intf.h>

#ifdef CONFIG_SDIO_HCI
	#include <sdio_hal.h>
#elif defined(CONFIG_USB_HCI)
	#include <usb_hal.h>
#elif defined(CONFIG_GSPI_HCI)
	#include <gspi_hal.h>
#endif

void intf_chip_configure(_adapter *padapter)
{
	if(padapter->HalFunc.intf_chip_configure)
		padapter->HalFunc.intf_chip_configure(padapter);
}

void intf_read_chip_info(_adapter *padapter)
{
	if(padapter->HalFunc.read_adapter_info)
		padapter->HalFunc.read_adapter_info(padapter);
}

void intf_read_chip_version(_adapter *padapter)
{
	if(padapter->HalFunc.read_chip_version)
		padapter->HalFunc.read_chip_version(padapter);
}

void rtw_hal_def_value_init(PADAPTER Adapter)
{
	if(Adapter->HalFunc.init_default_value)
		Adapter->HalFunc.init_default_value(Adapter);
}
void	rtw_hal_free_data(PADAPTER Adapter)
{
	if(Adapter->HalFunc.free_hal_data)
		Adapter->HalFunc.free_hal_data(Adapter);
}
void	rtw_hal_dm_init(_adapter *padapter)
{
	if(padapter->HalFunc.dm_init)
		padapter->HalFunc.dm_init(padapter);
}
void rtw_hal_dm_deinit(_adapter *padapter)
{
	// cancel dm  timer
	if(padapter->HalFunc.dm_deinit)
		padapter->HalFunc.dm_deinit(padapter);
}
void	rtw_hal_sw_led_init(_adapter *padapter)
{
	if(padapter->HalFunc.InitSwLeds)
		padapter->HalFunc.InitSwLeds(padapter);
}

void rtw_hal_sw_led_deinit(_adapter *padapter)
{
	if(padapter->HalFunc.DeInitSwLeds)
		padapter->HalFunc.DeInitSwLeds(padapter);
}

uint	 rtw_hal_init(_adapter *padapter)
{
	uint	status = _SUCCESS;

#ifdef CONFIG_DUALMAC_CONCURRENT
	if(padapter->hw_init_completed == _TRUE)
	{
		DBG_871X("rtw_hal_init: hw_init_completed == _TRUE\n");
		return status;
	}

	// before init mac0, driver must init mac1 first to avoid usb rx error.
	if((padapter->pbuddy_adapter != NULL) && (padapter->DualMacConcurrent == _TRUE)
		&& (padapter->adapter_type == PRIMARY_ADAPTER))
	{
		if(padapter->pbuddy_adapter->hw_init_completed == _TRUE)
		{
			DBG_871X("rtw_hal_init: pbuddy_adapter hw_init_completed == _TRUE\n");
		}
		else
		{
			status = 	padapter->HalFunc.hal_init(padapter->pbuddy_adapter);
			if(status == _SUCCESS){
				padapter->pbuddy_adapter->hw_init_completed = _TRUE;
			}
			else{
			 	padapter->pbuddy_adapter->hw_init_completed = _FALSE;
				RT_TRACE(_module_hal_init_c_,_drv_err_,("rtw_hal_init: hal__init fail(pbuddy_adapter)\n"));
				return status;
			}
		}
	}
#endif

	padapter->hw_init_completed=_FALSE;

	status = padapter->HalFunc.hal_init(padapter);

	if(status == _SUCCESS){
		padapter->hw_init_completed = _TRUE;
	}
	else{
	 	padapter->hw_init_completed = _FALSE;
		DBG_871X("rtw_hal_init: hal__init fail\n");
	}

	RT_TRACE(_module_hal_init_c_,_drv_err_,("-rtl871x_hal_init:status=0x%x\n",status));

	return status;

}

uint rtw_hal_deinit(_adapter *padapter)
{
	uint	status = _SUCCESS;

_func_enter_;

	status = padapter->HalFunc.hal_deinit(padapter);

	if(status == _SUCCESS){
		padapter->hw_init_completed = _FALSE;
	}
	else
	{
		DBG_871X("\n rtw_hal_deinit: hal_init fail\n");
	}

_func_exit_;

	return status;
}

void rtw_hal_set_hwreg(PADAPTER padapter, u8 variable, u8 *val)
{
	if (padapter->HalFunc.SetHwRegHandler)
		padapter->HalFunc.SetHwRegHandler(padapter, variable, val);
}

void rtw_hal_get_hwreg(PADAPTER padapter, u8 variable, u8 *val)
{
	if (padapter->HalFunc.GetHwRegHandler)
		padapter->HalFunc.GetHwRegHandler(padapter, variable, val);
}

u8 rtw_hal_set_def_var(PADAPTER Adapter, HAL_DEF_VARIABLE eVariable, PVOID pValue)
{
	if(Adapter->HalFunc.SetHalDefVarHandler)
		return Adapter->HalFunc.SetHalDefVarHandler(Adapter,eVariable,pValue);
	return _FAIL;
}
u8 rtw_hal_get_def_var(PADAPTER Adapter, HAL_DEF_VARIABLE eVariable, PVOID pValue)
{
	if(Adapter->HalFunc.GetHalDefVarHandler)
		return Adapter->HalFunc.GetHalDefVarHandler(Adapter,eVariable,pValue);
	return _FAIL;
}

void rtw_hal_set_odm_var(PADAPTER Adapter, HAL_ODM_VARIABLE eVariable, PVOID pValue1,BOOLEAN bSet)
{
	if(Adapter->HalFunc.SetHalODMVarHandler)
		Adapter->HalFunc.SetHalODMVarHandler(Adapter,eVariable,pValue1,bSet);
}
void	rtw_hal_get_odm_var(PADAPTER Adapter, HAL_ODM_VARIABLE eVariable, PVOID pValue1,BOOLEAN bSet)
{
	if(Adapter->HalFunc.GetHalODMVarHandler)
		Adapter->HalFunc.GetHalODMVarHandler(Adapter,eVariable,pValue1,bSet);
}

void rtw_hal_enable_interrupt(PADAPTER Adapter)
{
	if (Adapter->HalFunc.enable_interrupt)
		Adapter->HalFunc.enable_interrupt(Adapter);
	else
		DBG_871X("%s: HalFunc.enable_interrupt is NULL!\n", __FUNCTION__);

}
void rtw_hal_disable_interrupt(PADAPTER Adapter)
{
	if (Adapter->HalFunc.disable_interrupt)
		Adapter->HalFunc.disable_interrupt(Adapter);
	else
		DBG_871X("%s: HalFunc.disable_interrupt is NULL!\n", __FUNCTION__);

}


u32	rtw_hal_inirp_init(PADAPTER Adapter)
{
	u32 rst = _FAIL;
	if(Adapter->HalFunc.inirp_init)
		rst = Adapter->HalFunc.inirp_init(Adapter);
	else
		DBG_871X(" %s Initialize dvobjpriv.inirp_init error!!!\n",__FUNCTION__);
	return rst;
}

u32	rtw_hal_inirp_deinit(PADAPTER Adapter)
{

	if(Adapter->HalFunc.inirp_deinit)
		return Adapter->HalFunc.inirp_deinit(Adapter);

	return _FAIL;

}

u8	rtw_hal_intf_ps_func(PADAPTER Adapter,HAL_INTF_PS_FUNC efunc_id, u8* val)
{
	if(Adapter->HalFunc.interface_ps_func)
		return Adapter->HalFunc.interface_ps_func(Adapter,efunc_id,val);
	return _FAIL;
}

s32	rtw_hal_xmit(PADAPTER Adapter, struct xmit_frame *pxmitframe)
{
	if(Adapter->HalFunc.hal_xmit)
		return Adapter->HalFunc.hal_xmit(Adapter, pxmitframe);

	return _FALSE;
}

void	rtw_hal_mgnt_xmit(PADAPTER Adapter, struct xmit_frame *pmgntframe)
{
	if(Adapter->HalFunc.mgnt_xmit)
		Adapter->HalFunc.mgnt_xmit(Adapter, pmgntframe);
}
s32	rtw_hal_init_xmit_priv(PADAPTER Adapter)
{
	if(Adapter->HalFunc.init_xmit_priv != NULL)
		return Adapter->HalFunc.init_xmit_priv(Adapter);
	return _FAIL;
}
void	rtw_hal_free_xmit_priv(PADAPTER Adapter)
{
	if(Adapter->HalFunc.free_xmit_priv != NULL)
		Adapter->HalFunc.free_xmit_priv(Adapter);
}

s32	rtw_hal_init_recv_priv(PADAPTER Adapter)
{
	if(Adapter->HalFunc.init_recv_priv)
		return Adapter->HalFunc.init_recv_priv(Adapter);

	return _FAIL;
}
void	rtw_hal_free_recv_priv(PADAPTER Adapter)
{
	if(Adapter->HalFunc.free_recv_priv)
		Adapter->HalFunc.free_recv_priv(Adapter);
}

void rtw_hal_update_ra_mask(PADAPTER Adapter, u32 mac_id, u8 rssi_level)
{
	if(Adapter->HalFunc.UpdateRAMaskHandler)
		Adapter->HalFunc.UpdateRAMaskHandler(Adapter,mac_id,rssi_level);
}

void	rtw_hal_add_ra_tid(PADAPTER Adapter, u32 bitmap, u8 arg)
{
	if(Adapter->HalFunc.Add_RateATid)
		Adapter->HalFunc.Add_RateATid(Adapter, bitmap, arg);
}

u32	rtw_hal_read_bbreg(PADAPTER Adapter, u32 RegAddr, u32 BitMask)
{
	u32 data = 0;
	if(Adapter->HalFunc.read_bbreg)
		 data = Adapter->HalFunc.read_bbreg(Adapter, RegAddr, BitMask);
	return data;
}
void	rtw_hal_write_bbreg(PADAPTER Adapter, u32 RegAddr, u32 BitMask, u32 Data)
{
	if(Adapter->HalFunc.write_bbreg)
		Adapter->HalFunc.write_bbreg(Adapter, RegAddr, BitMask, Data);
}

u32	rtw_hal_read_rfreg(PADAPTER Adapter, u32 eRFPath, u32 RegAddr, u32 BitMask)
{
	u32 data = 0;
	if( Adapter->HalFunc.read_rfreg)
		data = Adapter->HalFunc.read_rfreg(Adapter, eRFPath, RegAddr, BitMask);
	return data;
}
void	rtw_hal_write_rfreg(PADAPTER Adapter, u32 eRFPath, u32 RegAddr, u32 BitMask, u32 Data)
{
	if(Adapter->HalFunc.write_rfreg)
		Adapter->HalFunc.write_rfreg(Adapter, eRFPath, RegAddr, BitMask, Data);
}

s32	rtw_hal_interrupt_handler(PADAPTER Adapter)
{
	if(Adapter->HalFunc.interrupt_handler)
		return Adapter->HalFunc.interrupt_handler(Adapter);
	return _FAIL;
}

void	rtw_hal_set_bwmode(PADAPTER Adapter, HT_CHANNEL_WIDTH Bandwidth, u8 Offset)
{
	if(Adapter->HalFunc.set_bwmode_handler)
		Adapter->HalFunc.set_bwmode_handler(Adapter, Bandwidth, Offset);
}

void	rtw_hal_set_chan(PADAPTER Adapter, u8 channel)
{
	if(Adapter->HalFunc.set_channel_handler)
		Adapter->HalFunc.set_channel_handler(Adapter, channel);
}

void	rtw_hal_dm_watchdog(PADAPTER Adapter)
{
	if(Adapter->HalFunc.hal_dm_watchdog)
		Adapter->HalFunc.hal_dm_watchdog(Adapter);
}

void rtw_hal_bcn_related_reg_setting(PADAPTER Adapter)
{
	if(Adapter->HalFunc.SetBeaconRelatedRegistersHandler)
		Adapter->HalFunc.SetBeaconRelatedRegistersHandler(Adapter);
}


#ifdef CONFIG_ANTENNA_DIVERSITY
u8	rtw_hal_antdiv_before_linked(PADAPTER Adapter)
{
	if(Adapter->HalFunc.AntDivBeforeLinkHandler)
		return Adapter->HalFunc.AntDivBeforeLinkHandler(Adapter);
	return _FALSE;
}
void	rtw_hal_antdiv_rssi_compared(PADAPTER Adapter, WLAN_BSSID_EX *dst, WLAN_BSSID_EX *src)
{
	if(Adapter->HalFunc.AntDivCompareHandler)
		Adapter->HalFunc.AntDivCompareHandler(Adapter, dst, src);
}
#endif

#ifdef CONFIG_HOSTAPD_MLME
s32	rtw_hal_hostap_mgnt_xmit_entry(PADAPTER Adapter, _pkt *pkt)
{
	if(Adapter->HalFunc.hostap_mgnt_xmit_entry)
		return Adapter->HalFunc.hostap_mgnt_xmit_entry(Adapter, pkt);
	return _FAIL;
}
#endif //CONFIG_HOSTAPD_MLME

#ifdef DBG_CONFIG_ERROR_DETECT
void	rtw_hal_sreset_init(_adapter *padapter)
{
	if(padapter->HalFunc.sreset_init_value)
		padapter->HalFunc.sreset_init_value(padapter);
}
void rtw_hal_sreset_reset(_adapter *padapter)
{
	if(padapter->HalFunc.silentreset)
		padapter->HalFunc.silentreset(padapter);
}

void rtw_hal_silent_reset(_adapter *padapter)
{
	if(padapter->HalFunc.sreset_reset_value)
		padapter->HalFunc.sreset_reset_value(padapter);
}

void rtw_hal_sreset_xmit_status_check(_adapter *padapter)
{
	if(padapter->HalFunc.sreset_xmit_status_check)
		padapter->HalFunc.sreset_xmit_status_check(padapter);
}
void rtw_hal_sreset_linked_status_check(_adapter *padapter)
{
	if(padapter->HalFunc.sreset_linked_status_check)
		padapter->HalFunc.sreset_linked_status_check(padapter);
}
u8   rtw_hal_sreset_get_wifi_status(_adapter *padapter)
{
	u8 status = 0;
	if(padapter->HalFunc.sreset_get_wifi_status)
		status = padapter->HalFunc.sreset_get_wifi_status(padapter);
	return status;
}

#endif	//DBG_CONFIG_ERROR_DETECT

#ifdef CONFIG_IOL
int rtw_hal_iol_cmd(ADAPTER *adapter, struct xmit_frame *xmit_frame, u32 max_wating_ms)
{
	if(adapter->HalFunc.IOL_exec_cmds_sync)
		return adapter->HalFunc.IOL_exec_cmds_sync(adapter, xmit_frame, max_wating_ms);
	return _FAIL;
}
#endif

#ifdef CONFIG_XMIT_THREAD_MODE
s32 rtw_hal_xmit_thread_handler(_adapter *padapter)
{
	if(padapter->HalFunc.xmit_thread_handler)
		return padapter->HalFunc.xmit_thread_handler(padapter);
	return _FAIL;
}
#endif


