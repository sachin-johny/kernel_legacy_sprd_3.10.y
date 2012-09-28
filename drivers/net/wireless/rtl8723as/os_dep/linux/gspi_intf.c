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
#define _HCI_INTF_C_

#include <drv_conf.h>
#include <osdep_service.h>
#include <drv_types.h>
#include <recv_osdep.h>
#include <xmit_osdep.h>
#include <rtw_version.h>

#ifndef CONFIG_GSPI_HCI
#error "CONFIG_GSPI_HCI should be on!\n"
#endif

#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
//#include <mach/ldo.h>
#include <asm/mach-types.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <mach/board.h>
#include <mach/hardware.h>
#include <mach/irqs.h>

#ifdef CONFIG_RTL8723A
#include <rtl8723a_hal.h>
#include <HalPwrSeqCmd.h>
#include <Hal8723PwrSeq.h>
#endif

#ifdef CONFIG_RTL8188E
#include <rtl8188e_hal.h>
#endif

#include <hal_intf.h>
#include <gspi_hal.h>
#include <gspi_ops.h>

#include <custom_gpio.h>


extern char* ifname;

typedef struct _driver_priv {
	int drv_registered;

#if defined(CONFIG_CONCURRENT_MODE) || defined(CONFIG_DUALMAC_CONCURRENT)
	//global variable
	_mutex h2c_fwcmd_mutex;
	_mutex setch_mutex;
	_mutex setbw_mutex;
	_mutex hw_init_mutex;
#endif
} drv_priv, *pdrv_priv;

unsigned int oob_irq;
static drv_priv drvpriv = {

};

static void decide_chip_type_by_device_id(PADAPTER padapter, u32 id)
{
	padapter->chip_type = NULL_CHIP_TYPE;

	switch (id)
	{
		case 0x8723:
			padapter->chip_type = RTL8188C_8192C;
			padapter->HardwareType = HARDWARE_TYPE_RTL8723AS;
			break;
		case 0x8179:
			padapter->chip_type = RTL8188E;
			padapter->HardwareType = HARDWARE_TYPE_RTL8188ES;
			break;
	}
}

static irqreturn_t spi_interrupt_thread(int irq, void *data)
{
	PADAPTER padapter = (PADAPTER)data;

	//spi_int_hdl(padapter);
	if (padapter->priv_wq)
		queue_delayed_work(padapter->priv_wq, &padapter->irq_work, 0);

	return IRQ_HANDLED;
}

static u32 gspi_init(PADAPTER padapter)
{
	struct dvobj_priv *psddev;
	PGSPI_DATA pgspi_data;
	struct spi_device *spi;
	int err;
	int g_irq = 0;

	_func_enter_;

	RT_TRACE(_module_hci_intfs_c_, _drv_notice_, ("+gspi_init\n"));
	if (padapter == NULL) {
		DBG_8192C(KERN_ERR "%s: padapter is NULL!\n", __func__);
		err = -1;
		goto exit;
	}

	psddev = &padapter->dvobjpriv;
	pgspi_data = &psddev->intf_data;
	spi = pgspi_data->func;

#if 0
	//3 1. init SDIO bus
	sdio_claim_host(func);

	err = sdio_enable_func(func);
	if (err) {
		DBG_8192C(KERN_CRIT "%s: sdio_enable_func FAIL(%d)!\n", __func__, err);
		goto release;
	}
	err = sdio_set_block_size(func, 512);
	if (err) {
		DBG_8192C(KERN_CRIT "%s: sdio_set_block_size FAIL(%d)!\n", __func__, err);
		goto release;
	}
#endif
	pgspi_data->block_transfer_len = 512;
	pgspi_data->tx_block_mode = 0;
	pgspi_data->rx_block_mode = 0;

	err = request_irq(oob_irq, spi_interrupt_thread,
			IRQF_TRIGGER_FALLING,//IRQF_TRIGGER_HIGH;//|IRQF_ONESHOT,
		       	DRV_NAME, padapter);
	//err = request_threaded_irq(oob_irq, NULL, spi_interrupt_thread,
	//		IRQF_TRIGGER_FALLING,
	//		DRV_NAME, padapter);
	if (err < 0) {
		DBG_8192C("Oops: can't allocate irq %d err:%d\n", oob_irq, err);
		goto exit;
	}
	enable_irq_wake(oob_irq);
	disable_irq(oob_irq);
release:
	//sdio_release_host(func);

exit:
	_func_exit_;

	if (err) return _FAIL;
	return _SUCCESS;
}

static void gspi_deinit(PADAPTER padapter)
{
	struct dvobj_priv *psddev;
	struct spi_device *spi;
	int err;


	RT_TRACE(_module_hci_intfs_c_, _drv_notice_, ("+gspi_deinit\n"));

	if (padapter == NULL) {
		DBG_8192C(KERN_ERR "%s: padapter is NULL!\n", __func__);
		return;
	}
	psddev = &padapter->dvobjpriv;
	spi = psddev->intf_data.func;


	if (spi) {
		free_irq(oob_irq, padapter);
	}
}

static void spi_irq_work(void *data)
{
	struct delayed_work *dwork;
	PADAPTER padapter;

	dwork = container_of(data, struct delayed_work, work);
	padapter = container_of(dwork, struct _ADAPTER, irq_work);

	spi_int_hdl(padapter);
}

static void sd_intf_start(PADAPTER padapter)
{
	if (padapter == NULL) {
		DBG_8192C(KERN_ERR "%s: padapter is NULL!\n", __func__);
		return;
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
	padapter->priv_wq = alloc_workqueue("spi_wq", 0, 0);
#else
	padapter->priv_wq = create_workqueue("spi_wq");
#endif
	INIT_DELAYED_WORK(&padapter->irq_work, (void*)spi_irq_work);

	enable_irq(oob_irq);
	//hal dep
	rtw_hal_enable_interrupt(padapter);

	return;
}

static void sd_intf_stop(PADAPTER padapter)
{
	if (padapter == NULL) {
		DBG_8192C(KERN_ERR "%s: padapter is NULL!\n", __func__);
		return;
	}

	if (padapter->priv_wq) {
		cancel_delayed_work_sync(&padapter->irq_work);
		flush_workqueue(padapter->priv_wq);
		destroy_workqueue(padapter->priv_wq);
		padapter->priv_wq = NULL;
	}

	//hal dep
	rtw_hal_disable_interrupt(padapter);
	disable_irq(oob_irq);
}
/*
 * drv_init() - a device potentially for us
 *
 * notes: drv_init() is called when the bus driver has located a card for us to support.
 *        We accept the new device by returning 0.
 */
static int /*__devinit*/  rtw_drv_probe(
	struct spi_device *spi)
{
	struct net_device *pnetdev;
	PADAPTER padapter;
	struct dvobj_priv *pdvobjpriv;
	PGSPI_DATA pgspi;


	DBG_8192C("RTW: %s line:%d", __FUNCTION__, __LINE__);

	//3 1. init network device data
	pnetdev = rtw_init_netdev(NULL);
	if (!pnetdev) goto error;

	SET_NETDEV_DEV(pnetdev, &spi->dev);

	padapter = rtw_netdev_priv(pnetdev);
	pdvobjpriv = &padapter->dvobjpriv;
	pdvobjpriv->padapter = padapter;
	pgspi = &pdvobjpriv->intf_data;
	pgspi->func = spi;

#ifdef CONFIG_IOCTL_CFG80211
	rtw_wdev_alloc(padapter, &spi->dev);
#endif

	//spi init
	/* This is the only SPI value that we need to set here, the rest
	 * comes from the board-peripherals file */
	spi->bits_per_word = 32;
	spi->max_speed_hz = 48 * 1000 * 1000;
	//here mode 0 and 3 all ok,
	//3 can run under 48M clock when SPI_CTL4 bit14 IS_FST set to 1
	//0 can run under 24M clock, but can run under 48M when SPI_CTL4 bit14 IS_FST set to 1 and Ctl0_reg[1:0] set to 3.
	spi->mode = SPI_MODE_3;
	spi_setup(spi);

#if 1
	//DBG_8192C("set spi ==========================%d \n", spi_setup(spi));

	DBG_8192C("%s, mode = %d \n", __func__, spi->mode);
	DBG_8192C("%s, bit_per_word = %d \n", __func__, spi->bits_per_word);
	DBG_8192C("%s, speed = %d \n", __func__, spi->max_speed_hz);
	DBG_8192C("%s, chip_select = %d \n", __func__, spi->chip_select);
	DBG_8192C("%s, controller_data = %d \n", __func__, *(int *)spi->controller_data);

	DBG_8192C("%s, irq= %d \n", __func__, oob_irq);

#endif
	DBG_8192C("RTW: %s line:%d", __FUNCTION__, __LINE__);

	//3 2. set interface private data
	spi_set_drvdata(spi, padapter);

	//3 3. init driver special setting, interface, OS and hardware relative
	// set interface_type to gspi
	padapter->interface_type = RTW_SPI;
	decide_chip_type_by_device_id(padapter, 0x8723);

	//4 3.1 set hardware operation functions
	padapter->HalData = rtw_zmalloc(sizeof(HAL_DATA_TYPE));
	if (padapter->HalData == NULL) {
		RT_TRACE(_module_hci_intfs_c_, _drv_err_,
			 ("rtw_drv_init: can't alloc memory for HAL DATA\n"));
		goto error;
	}
        padapter->hal_data_sz = sizeof(HAL_DATA_TYPE);
	set_hal_ops(padapter);

	//3 4. interface init
	if (gspi_init(padapter) != _SUCCESS) {
		RT_TRACE(_module_hci_intfs_c_, _drv_err_,
			 ("rtw_drv_init: initialize device object priv Failed!\n"));
		goto error;
	}
	padapter->intf_start = &sd_intf_start;
	padapter->intf_stop = &sd_intf_stop;


	//3 5. register I/O operations
	if (rtw_init_io_priv(padapter) == _FAIL)
	{
		RT_TRACE(_module_hci_intfs_c_, _drv_err_,
			("rtw_drv_init: Can't init io_priv\n"));
		goto deinit;
	}

	{
		u32 ret = 0;
		DBG_8192C("read start:\n");
		//spi_write8_endian(padapter, SPI_LOCAL_OFFSET | 0xF0, 0x01, 1);
		rtw_write8(padapter, SPI_LOCAL_OFFSET | 0xF0, 0x03);
		ret = rtw_read32(padapter, SPI_LOCAL_OFFSET | 0xF0);
		DBG_8192C("read end 0xF0 read32:%x:\n", ret);
		DBG_8192C("read end 0xF0 read8:%x:\n", rtw_read8(padapter, SPI_LOCAL_OFFSET | 0xF0));

	}
	//goto deinit;
	//3 6.
	intf_read_chip_version(padapter);

	//3 7.
	intf_chip_configure(padapter);

	//3 8. read efuse/eeprom data
	intf_read_chip_info(padapter);

	//3 9. init driver common data
	if (rtw_init_drv_sw(padapter) == _FAIL) {
		RT_TRACE(_module_hci_intfs_c_, _drv_err_,
			 ("rtw_drv_init: Initialize driver software resource Failed!\n"));
		goto deinit;
	}

	//3 10. get WLan MAC address
	// alloc dev name after read efuse.
	rtw_init_netdev_name(pnetdev, ifname);

	rtw_macaddr_cfg(padapter->eeprompriv.mac_addr);
	_rtw_memcpy(pnetdev->dev_addr, padapter->eeprompriv.mac_addr, ETH_ALEN);


#ifdef CONFIG_PROC_DEBUG
#ifdef RTK_DMP_PLATFORM
	rtw_proc_init_one(pnetdev);
#endif
#endif

#ifdef CONFIG_HOSTAPD_MLME
	hostapd_mode_init(padapter);
#endif

	DBG_871X("bDriverStopped:%d, bSurpriseRemoved:%d, bup:%d, hw_init_completed:%d\n"
		,padapter->bDriverStopped
		,padapter->bSurpriseRemoved
		,padapter->bup
		,padapter->hw_init_completed
	);


	//3 8. Tell the network stack we exist
	if (register_netdev(pnetdev) != 0) {
		RT_TRACE(_module_hci_intfs_c_, _drv_err_,
			 ("rtw_drv_init: register_netdev() failed\n"));
		goto deinit;
	}

	RT_TRACE(_module_hci_intfs_c_, _drv_info_,
		 ("-rtw_drv_init: Success. bDriverStopped=%d bSurpriseRemoved=%d\n",
		  padapter->bDriverStopped, padapter->bSurpriseRemoved));

	return 0;

deinit:
	gspi_deinit(padapter);

error:
	if (padapter)
	{
		if (padapter->HalData && padapter->hal_data_sz>0) {
			rtw_mfree(padapter->HalData, padapter->hal_data_sz);
			padapter->HalData = NULL;
		}
	}

	if (pnetdev)
		rtw_free_netdev(pnetdev);

	RT_TRACE(_module_hci_intfs_c_, _drv_crit_, ("-rtw_drv_init: FAIL!\n"));

	return -1;
}

/*
 * Do deinit job corresponding to netdev_open()
 */
static void rtw_dev_unload(PADAPTER padapter)
{
	struct net_device *pnetdev = (struct net_device*)padapter->pnetdev;
	struct mlme_priv *pmlmepriv = &padapter->mlmepriv;

	RT_TRACE(_module_hci_intfs_c_, _drv_notice_, ("+rtw_dev_unload\n"));

	padapter->bDriverStopped = _TRUE;

	if (padapter->bup == _TRUE)
	{
#if 0
		if (padapter->intf_stop)
			padapter->intf_stop(padapter);
#else
		sd_intf_stop(padapter);
#endif
		RT_TRACE(_module_hci_intfs_c_, _drv_notice_, ("@ rtw_dev_unload: stop intf complete!\n"));

		if (!padapter->pwrctrlpriv.bInternalAutoSuspend)
			rtw_stop_drv_threads(padapter);

		RT_TRACE(_module_hci_intfs_c_, _drv_notice_, ("@ rtw_dev_unload: stop thread complete!\n"));

		if (padapter->bSurpriseRemoved == _FALSE)
		{
#ifdef CONFIG_WOWLAN
			if (padapter->pwrctrlpriv.bSupportWakeOnWlan == _TRUE) {
				DBG_871X("%s bSupportWakeOnWlan==_TRUE  do not run rtw_hal_deinit()\n",__FUNCTION__);
			}
			else
#endif
			{
#ifdef CONFIG_IPS
				/* IPS will make wifi power state enter suspend, we
				 * need leave suspend state first, then enter
				 * power down state. */
				if ((check_fwstate(pmlmepriv, _FW_LINKED) == _FALSE)
					&& (padapter->pwrctrlpriv.rf_pwrstate== rf_off)) {
					// unlock ISO/CLK/Power control register
					rtw_write8(padapter, REG_RSV_CTRL, 0x0);
					HalPwrSeqCmdParsing(padapter, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_SDIO_MSK,
							rtl8723A_ips_to_hwpdn_flow);
				} else
#endif
				{
					//amy modify 20120221 for power seq is different between driver open and ips
					rtw_hal_deinit(padapter);
				}
			}
			padapter->bSurpriseRemoved = _TRUE;
		}
		RT_TRACE(_module_hci_intfs_c_, _drv_notice_, ("@ rtw_dev_unload: deinit hal complelt!\n"));

		padapter->bup = _FALSE;
	}
	else {
		RT_TRACE(_module_hci_intfs_c_, _drv_notice_, ("rtw_dev_unload: bup==_FALSE\n"));
	}

	RT_TRACE(_module_hci_intfs_c_, _drv_notice_, ("-rtw_dev_unload\n"));
}

static int /*__devexit*/  rtw_dev_remove(struct spi_device *spi)
{
	PADAPTER padapter = spi_get_drvdata(spi);
	struct net_device *pnetdev;
	u32 value;
#ifdef CONFIG_IOCTL_CFG80211
	struct wireless_dev *wdev;
#endif

_func_enter_;

	RT_TRACE(_module_hci_intfs_c_, _drv_notice_, ("+rtw_dev_remove\n"));

	//padapter = ((struct dvobj_priv*)sdio_get_drvdata(func))->padapter;
#ifdef CONFIG_IOCTL_CFG80211
	wdev = padapter->rtw_wdev;
#endif

#if defined(CONFIG_HAS_EARLYSUSPEND ) || defined(CONFIG_ANDROID_POWER)
	rtw_unregister_early_suspend(&padapter->pwrctrlpriv);
#endif

	if (padapter->bSurpriseRemoved == _FALSE)
	{
		// test surprise remove
		int err;

#if 0
		sdio_claim_host(func);
		sdio_readb(func, 0, &err);
		sdio_release_host(func);
		if (err == -ENOMEDIUM) {
			padapter->bSurpriseRemoved = _TRUE;
			DBG_871X(KERN_NOTICE "%s: device had been removed!\n", __func__);
		}
#endif
	}

#ifdef CONFIG_HOSTAPD_MLME
	hostapd_mode_unload(padapter);
#endif

	LeaveAllPowerSaveMode(padapter);

	pnetdev = (struct net_device*)padapter->pnetdev;
	if (pnetdev) {
		unregister_netdev(pnetdev); //will call netdev_close()
		RT_TRACE(_module_hci_intfs_c_, _drv_notice_, ("rtw_dev_remove: unregister netdev\n"));
#ifdef CONFIG_PROC_DEBUG
		rtw_proc_remove_one(pnetdev);
#endif
	} else {
		RT_TRACE(_module_hci_intfs_c_, _drv_err_, ("rtw_dev_remove: NO padapter->pnetdev!\n"));
	}

	rtw_cancel_all_timer(padapter);

	rtw_dev_unload(padapter);

#if 1
	//WILL OUT
	/* enable power down pin detection preparing for entering
	 * power down mode in rtw_drv_halt() function. */
	// unlock ISO/CLK/Power control register
	rtw_write8(padapter, REG_RSV_CTRL, 0x0);
	// enable power down pin input detection
	value = rtw_read32(padapter, 0x4);
	value |= BIT15;
	rtw_write32(padapter, 0x4, value);
#endif

	// interface deinit
	gspi_deinit(padapter);
	spi_set_drvdata(spi, NULL);
	RT_TRACE(_module_hci_intfs_c_, _drv_notice_, ("rtw_dev_remove: deinit intf complete!\n"));

#ifdef CONFIG_GSPI_HCI
	rtw_write8(padapter, SPI_LOCAL_OFFSET | 0xF0, 0x03);
#endif
	rtw_free_drv_sw(padapter);

#ifdef CONFIG_IOCTL_CFG80211
	rtw_wdev_free(wdev);
#endif

	RT_TRACE(_module_hci_intfs_c_, _drv_notice_, ("-rtw_dev_remove\n"));

_func_exit_;
	return 0;
}


static int rtw_gspi_suspend(struct spi_device *spi, pm_message_t mesg)
{
	PADAPTER padapter = spi_get_drvdata(spi);
	struct pwrctrl_priv *pwrpriv = &padapter->pwrctrlpriv;
	struct mlme_priv *pmlmepriv = &padapter->mlmepriv;
	struct net_device *pnetdev = padapter->pnetdev;
	int ret = 0;
	u32 value;

	u32 start_time = rtw_get_current_time();

	_func_enter_;

	DBG_871X("==> %s (%s:%d)\n",__FUNCTION__, current->comm, current->pid);

	pwrpriv->bInSuspend = _TRUE;

	while (pwrpriv->bips_processing == _TRUE)
		rtw_msleep_os(1);

	if((!padapter->bup) || (padapter->bDriverStopped)||(padapter->bSurpriseRemoved))
	{
		DBG_871X("%s bup=%d bDriverStopped=%d bSurpriseRemoved = %d\n", __FUNCTION__
			,padapter->bup, padapter->bDriverStopped,padapter->bSurpriseRemoved);
		goto exit;
	}

	rtw_cancel_all_timer(padapter);
	LeaveAllPowerSaveMode(padapter);

	//padapter->net_closed = _TRUE;
	//s1.
	if(pnetdev)
	{
		netif_carrier_off(pnetdev);
		rtw_netif_stop_queue(pnetdev);
	}
#ifdef CONFIG_WOWLAN
	padapter->pwrctrlpriv.bSupportWakeOnWlan=_TRUE;
#else
	//s2.
	//s2-1.  issue rtw_disassoc_cmd to fw
	disconnect_hdl(padapter, NULL);
	//rtw_disassoc_cmd(padapter);
#endif

#ifdef CONFIG_LAYER2_ROAMING_RESUME
	if(check_fwstate(pmlmepriv, WIFI_STATION_STATE) && check_fwstate(pmlmepriv, _FW_LINKED) )
	{
		DBG_871X("%s %s(" MAC_FMT "), length:%d assoc_ssid.length:%d\n",__FUNCTION__,
				pmlmepriv->cur_network.network.Ssid.Ssid,
				MAC_ARG(pmlmepriv->cur_network.network.MacAddress),
				pmlmepriv->cur_network.network.Ssid.SsidLength,
				pmlmepriv->assoc_ssid.SsidLength);

		pmlmepriv->to_roaming = 1;
	}
#endif

	//s2-2.  indicate disconnect to os
	rtw_indicate_disconnect(padapter);
	//s2-3.
	rtw_free_assoc_resources(padapter, 1);

	//s2-4.
	rtw_free_network_queue(padapter, _TRUE);

	rtw_led_control(padapter, LED_CTL_POWER_OFF);

	rtw_dev_unload(padapter);

	if(check_fwstate(pmlmepriv, _FW_UNDER_SURVEY))
		rtw_indicate_scan_done(padapter, 1);

	if(check_fwstate(pmlmepriv, _FW_UNDER_LINKING))
		rtw_indicate_disconnect(padapter);

	// interface deinit
	gspi_deinit(padapter);
	RT_TRACE(_module_hci_intfs_c_, _drv_notice_, ("%s: deinit SDIO complete!\n", __FUNCTION__));

	rtw_wifi_gpio_wlan_ctrl(WLAN_PWDN_OFF);
	rtw_mdelay_os(1);
exit:
	DBG_871X("<===  %s return %d.............. in %dms\n", __FUNCTION__
		, ret, rtw_get_passing_time_ms(start_time));

	_func_exit_;
	return ret;
}

extern int pm_netdev_open(struct net_device *pnetdev,u8 bnormal);
int rtw_resume_process(_adapter *padapter)
{
	struct net_device *pnetdev;
	struct pwrctrl_priv *pwrpriv;
	u8 is_pwrlock_hold_by_caller;
	u8 is_directly_called_by_auto_resume;
	int ret = 0;
	u32 start_time = rtw_get_current_time();

	_func_enter_;

	DBG_871X("==> %s (%s:%d)\n",__FUNCTION__, current->comm, current->pid);

	rtw_wifi_gpio_wlan_ctrl(WLAN_PWDN_ON);
	rtw_mdelay_os(1);

	{
		u32 ret = 0;
		DBG_8192C("read start:\n");
		//spi_write8_endian(padapter, SPI_LOCAL_OFFSET | 0xF0, 0x01, 1);
		rtw_write8(padapter, SPI_LOCAL_OFFSET | 0xF0, 0x03);
		ret = rtw_read32(padapter, SPI_LOCAL_OFFSET | 0xF0);
		DBG_8192C("read end 0xF0 read32:%x:\n", ret);
		DBG_8192C("read end 0xF0 read8:%x:\n", rtw_read8(padapter, SPI_LOCAL_OFFSET | 0xF0));

	}

	if (padapter) {
		pnetdev = padapter->pnetdev;
		pwrpriv = &padapter->pwrctrlpriv;
	} else {
		ret = -1;
		goto exit;
	}

	// interface init
	if (gspi_init(padapter) != _SUCCESS)
	{
		ret = -1;
		RT_TRACE(_module_hci_intfs_c_, _drv_err_, ("%s: initialize SDIO Failed!!\n", __FUNCTION__));
		goto exit;
	}

	rtw_reset_drv_sw(padapter);
	pwrpriv->bkeepfwalive = _FALSE;

	DBG_871X("bkeepfwalive(%x)\n",pwrpriv->bkeepfwalive);
	if(pm_netdev_open(pnetdev,_TRUE) != 0) {
		ret = -1;
		goto exit;
	}

	netif_device_attach(pnetdev);
	netif_carrier_on(pnetdev);

	if( padapter->pid[1]!=0) {
		DBG_871X("pid[1]:%d\n",padapter->pid[1]);
		rtw_signal_process(padapter->pid[1], SIGUSR2);
	}

	#ifdef CONFIG_LAYER2_ROAMING_RESUME
	rtw_roaming(padapter, NULL);
	#endif

	#ifdef CONFIG_RESUME_IN_WORKQUEUE
	rtw_unlock_suspend();
	#endif //CONFIG_RESUME_IN_WORKQUEUE

	pwrpriv->bInSuspend = _FALSE;
exit:
	DBG_871X("<===  %s return %d.............. in %dms\n", __FUNCTION__
		, ret, rtw_get_passing_time_ms(start_time));

	_func_exit_;

	return ret;
}

static int rtw_gspi_resume(struct spi_device *spi)
{
	PADAPTER padapter = spi_get_drvdata(spi);
	struct pwrctrl_priv *pwrpriv = &padapter->pwrctrlpriv;
	 int ret = 0;

	DBG_871X("==> %s (%s:%d)\n",__FUNCTION__, current->comm, current->pid);

	if(pwrpriv->bInternalAutoSuspend ){
 		ret = rtw_resume_process(padapter);
	} else {
#ifdef CONFIG_RESUME_IN_WORKQUEUE
		rtw_resume_in_workqueue(pwrpriv);
#elif defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_ANDROID_POWER)
		if(rtw_is_earlysuspend_registered(pwrpriv)) {
			//jeff: bypass resume here, do in late_resume
			pwrpriv->do_late_resume = _TRUE;
		} else {
			ret = rtw_resume_process(padapter);
		}
#else // Normal resume process
		ret = rtw_resume_process(padapter);
#endif //CONFIG_RESUME_IN_WORKQUEUE
	}

	DBG_871X("<========  %s return %d\n", __FUNCTION__, ret);
	return ret;

}


static struct spi_driver rtw_spi_drv = {
	.probe = rtw_drv_probe,
	.remove = rtw_dev_remove,
	.suspend = rtw_gspi_suspend,
	.resume = rtw_gspi_resume,
	.driver = {
		.name = "wlan_spi",
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
	}

};

static int __init rtw_drv_entry(void)
{
	int ret;


	RT_TRACE(_module_hci_intfs_c_, _drv_notice_, ("+rtw_drv_entry\n"));
	DBG_8192C("RTW: rtw_drv_entry enter\n");

	rtw_suspend_lock_init();

#if defined(CONFIG_CONCURRENT_MODE) || defined(CONFIG_DUALMAC_CONCURRENT)
	//init global variable
	_rtw_mutex_init(&drvpriv.h2c_fwcmd_mutex);
	_rtw_mutex_init(&drvpriv.setch_mutex);
	_rtw_mutex_init(&drvpriv.setbw_mutex);
	_rtw_mutex_init(&drvpriv.hw_init_mutex);
#endif

	drvpriv.drv_registered = _TRUE;

	rtw_wifi_gpio_init();
	rtw_wifi_gpio_wlan_ctrl(WLAN_PWDN_ON);
	ret = spi_register_driver(&rtw_spi_drv);

	DBG_8192C("RTW: rtw_drv_entry exit %d\n", ret);

	return 0;
}

static void __exit rtw_drv_halt(void)
{
	RT_TRACE(_module_hci_intfs_c_, _drv_notice_, ("+rtw_drv_halt\n"));
	DBG_8192C("RTW: rtw_drv_halt enter\n");

	rtw_suspend_lock_uninit();
	drvpriv.drv_registered = _FALSE;

	spi_unregister_driver(&rtw_spi_drv);

#if defined(CONFIG_CONCURRENT_MODE) || defined(CONFIG_DUALMAC_CONCURRENT)
	_rtw_mutex_free(&drvpriv.h2c_fwcmd_mutex);
	_rtw_mutex_free(&drvpriv.setch_mutex);
	_rtw_mutex_free(&drvpriv.setbw_mutex);
	_rtw_mutex_free(&drvpriv.hw_init_mutex);
#endif

	rtw_wifi_gpio_wlan_ctrl(WLAN_PWDN_OFF);
	rtw_wifi_gpio_deinit();

	DBG_8192C("RTW: rtw_drv_halt enter\n");
	RT_TRACE(_module_hci_intfs_c_, _drv_notice_, ("-rtw_drv_halt\n"));
}
module_init(rtw_drv_entry);
module_exit(rtw_drv_halt);
