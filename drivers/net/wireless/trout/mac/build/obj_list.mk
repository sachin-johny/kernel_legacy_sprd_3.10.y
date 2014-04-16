SRC_DIR = src
$(TROUT_KO_NAME)-objs := \
$(SRC_DIR)/Utils/MM/imem.o \
$(SRC_DIR)/Utils/MM/imem_if.o \
$(SRC_DIR)/Utils/QM/qmu.o \
$(SRC_DIR)/Utils/QM/qmu_rx.o \
$(SRC_DIR)/Utils/QM/qmu_tx.o \
$(SRC_DIR)/Utils/QM/coex_tx.o \
$(SRC_DIR)/Common/common.o \
$(SRC_DIR)/Common/trout_wifi_rx.o \
$(SRC_DIR)/Common/mac_reg_dbg.o \
$(SRC_DIR)/Common/trout_trace.o \
$(SRC_DIR)/Test/MacSwTest/host_if_test.o \
$(SRC_DIR)/Test/MacSwTest/test_config.o \
$(SRC_DIR)/Test/MacHwTest/mh_test.o \
$(SRC_DIR)/Test/MacHwTest/mh_test_config.o \
$(SRC_DIR)/Test/MacHwTest/mh_test_init.o \
$(SRC_DIR)/Test/MacHwTest/mh_test_isr.o \
$(SRC_DIR)/Test/MacHwTest/mh_test_txabort.o \
$(SRC_DIR)/Test/MacHwTest/mh_test_utils.o \
$(SRC_DIR)/Library/WPS/Common/wps_common.o \
$(SRC_DIR)/Library/WPS/Protocols/aes_cbc.o \
$(SRC_DIR)/Library/WPS/Protocols/dh_key.o \
$(SRC_DIR)/Library/WPS/Protocols/hmac_sha256.o \
$(SRC_DIR)/Library/WPS/Protocols/wps_key.o \
$(SRC_DIR)/Library/WPS/Protocols/wps_prot.o \
$(SRC_DIR)/Library/WPS/Registrar/wps_eap_rx_reg.o \
$(SRC_DIR)/Library/WPS/Registrar/wps_eap_tx_reg.o \
$(SRC_DIR)/Library/WPS/Registrar/wps_registrar.o \
$(SRC_DIR)/Library/WPS/Enrollee/wpse_dh_key.o \
$(SRC_DIR)/Library/WPS/Enrollee/wpse_hmac_sha256.o \
$(SRC_DIR)/Library/WPS/Enrollee/wpse_init.o \
$(SRC_DIR)/Library/WPS/Enrollee/wpse_key.o \
$(SRC_DIR)/Library/WPS/Enrollee/wpse_parse.o \
$(SRC_DIR)/Library/WPS/Enrollee/wpse_prot_rx.o \
$(SRC_DIR)/Library/WPS/Enrollee/wpse_prot_tx.o \
$(SRC_DIR)/Controller/AP-STA/event_manager.o \
$(SRC_DIR)/Controller/AP-STA/event_parser.o \
$(SRC_DIR)/Controller/AP-STA/iconfig.o \
$(SRC_DIR)/Controller/AP-STA/maccontroller.o \
$(SRC_DIR)/Controller/AP/event_parser_ap.o \
$(SRC_DIR)/Controller/AP/sme_ap.o \
$(SRC_DIR)/Controller/STA/event_parser_sta.o \
$(SRC_DIR)/Controller/STA/sme_sta.o \
$(SRC_DIR)/Controller/STA/P2P/p2p_sme_sta.o \
$(SRC_DIR)/Edge/Processor/MPC8349/mpc8349.o \
$(SRC_DIR)/Edge/PHY/PHYHW/ITTIAM/phy_ittiam.o \
$(SRC_DIR)/Edge/PHY/PHYHW/ITTIAM/phy_model.o \
$(SRC_DIR)/Edge/PHY/PHYProt/P802_11n/mib_802_11n.o \
$(SRC_DIR)/Edge/PHY/PHYProt/P802_11n/phy_802_11n.o \
$(SRC_DIR)/Edge/MACHW/ce_lut.o \
$(SRC_DIR)/Edge/MACHW/mh.o \
$(SRC_DIR)/Edge/IF/host_if.o \
$(SRC_DIR)/Edge/IF/proc_if.o \
$(SRC_DIR)/Edge/IF/rf_if.o \
$(SRC_DIR)/Edge/RF/GENERIC_RF/reg_domain_generic.o \
$(SRC_DIR)/Edge/RF/AIROHA_ITTIAM/vco_airoha_ittiam.o \
$(SRC_DIR)/Edge/RF/MAXIM_ITTIAM/reg_domain_maxim_ittiam.o \
$(SRC_DIR)/Edge/RF/MAXIM_ITTIAM/vco_maxim_ittiam.o \
$(SRC_DIR)/CSL/csl_if.o \
$(SRC_DIR)/CSL/csl_linux.o \
$(SRC_DIR)/CSL/trout_rf_test_mode.o \
$(SRC_DIR)/CSL/itm_wifi_iw.o \
$(SRC_DIR)/CSL/trace_log_fuc.o \
$(SRC_DIR)/Core/AP-STA/autorate.o \
$(SRC_DIR)/Core/AP-STA/buff_desc.o \
$(SRC_DIR)/Core/AP-STA/cglobals.o \
$(SRC_DIR)/Core/AP-STA/ch_info.o \
$(SRC_DIR)/Core/AP-STA/frame.o \
$(SRC_DIR)/Core/AP-STA/fsm.o \
$(SRC_DIR)/Core/AP-STA/index_util.o \
$(SRC_DIR)/Core/AP-STA/mac_init.o \
$(SRC_DIR)/Core/AP-STA/management.o \
$(SRC_DIR)/Core/AP-STA/metrics.o \
$(SRC_DIR)/Core/AP-STA/mib.o \
$(SRC_DIR)/Core/AP-STA/receive.o \
$(SRC_DIR)/Core/AP-STA/reg_domains.o \
$(SRC_DIR)/Core/AP-STA/transmit.o \
$(SRC_DIR)/Core/AP-STA/wep.o \
$(SRC_DIR)/Core/AP-STA/wapi.o \
$(SRC_DIR)/Core/AP-STA/wapi_sms4.o \
$(SRC_DIR)/Core/AP-STA/M802_11i/aes.o \
$(SRC_DIR)/Core/AP-STA/M802_11i/aeskeywrap.o \
$(SRC_DIR)/Core/AP-STA/M802_11i/auth_frame_11i.o \
$(SRC_DIR)/Core/AP-STA/M802_11i/eapol_key_auth.o \
$(SRC_DIR)/Core/AP-STA/M802_11i/eapol_key.o \
$(SRC_DIR)/Core/AP-STA/M802_11i/hmac_md5.o \
$(SRC_DIR)/Core/AP-STA/M802_11i/ieee_11i_auth.o \
$(SRC_DIR)/Core/AP-STA/M802_11i/md5.o \
$(SRC_DIR)/Core/AP-STA/M802_11i/mib_11i.o \
$(SRC_DIR)/Core/AP-STA/M802_11i/prf.o \
$(SRC_DIR)/Core/AP-STA/M802_11i/rc4.o \
$(SRC_DIR)/Core/AP-STA/M802_11i/rkmal_auth.o \
$(SRC_DIR)/Core/AP-STA/M802_11i/rsna_auth_4way.o \
$(SRC_DIR)/Core/AP-STA/M802_11i/rsna_auth_glk.o \
$(SRC_DIR)/Core/AP-STA/M802_11i/rsna_auth_grpkey.o \
$(SRC_DIR)/Core/AP-STA/M802_11i/rsna_auth_km.o \
$(SRC_DIR)/Core/AP-STA/M802_11i/rsna_km.o \
$(SRC_DIR)/Core/AP-STA/M802_11i/tkip_auth.o \
$(SRC_DIR)/Core/AP-STA/M802_11i/tkip.o \
$(SRC_DIR)/Core/AP-STA/M802_11i/utils.o \
$(SRC_DIR)/Core/AP-STA/M802_11e/frame_11e.o \
$(SRC_DIR)/Core/AP-STA/M802_11e/management_11e.o \
$(SRC_DIR)/Core/AP-STA/M802_11e/mib_11e.o \
$(SRC_DIR)/Core/AP-STA/P2P/frame_p2p.o \
$(SRC_DIR)/Core/AP-STA/P2P/mgmt_p2p.o \
$(SRC_DIR)/Core/AP-STA/P2P/mib_p2p.o \
$(SRC_DIR)/Core/AP-STA/P2P/p2p_ps.o \
$(SRC_DIR)/Core/AP-STA/M802_11h/mib_11h.o \
$(SRC_DIR)/Core/AP-STA/M802_11n/amsdu_aggr.o \
$(SRC_DIR)/Core/AP-STA/M802_11n/amsdu_deaggr.o \
$(SRC_DIR)/Core/AP-STA/M802_11n/blockack.o \
$(SRC_DIR)/Core/AP-STA/M802_11n/frame_11n.o \
$(SRC_DIR)/Core/AP-STA/M802_11n/management_11n.o \
$(SRC_DIR)/Core/AP-STA/M802_11n/mib_11n.o \
$(SRC_DIR)/Core/AP-STA/MDOM/frame_mdom.o \
$(SRC_DIR)/Core/AP-STA/MDOM/mib_mdom.o \
$(SRC_DIR)/Core/AP/autorate_ap.o \
$(SRC_DIR)/Core/AP/cglobals_ap.o \
$(SRC_DIR)/Core/AP/chan_mgmt_ap.o \
$(SRC_DIR)/Core/AP/frame_ap.o \
$(SRC_DIR)/Core/AP/fsm_ap.o \
$(SRC_DIR)/Core/AP/host_rx_mlme_ap.o \
$(SRC_DIR)/Core/AP/host_rx_msdu_ap.o \
$(SRC_DIR)/Core/AP/management_ap.o \
$(SRC_DIR)/Core/AP/pm_ap.o \
$(SRC_DIR)/Core/AP/wep_ap.o \
$(SRC_DIR)/Core/AP/wlan_misc_ap.o \
$(SRC_DIR)/Core/AP/wlan_rx_data_ap.o \
$(SRC_DIR)/Core/AP/wlan_rx_mgmt_ap.o \
$(SRC_DIR)/Core/AP/M802_1x/ieee_1x_ap.o \
$(SRC_DIR)/Core/AP/M802_1x/ieee_1x_auth_bak.o \
$(SRC_DIR)/Core/AP/M802_1x/ieee_1x_auth.o \
$(SRC_DIR)/Core/AP/M802_1x/ieee_1x_reauth.o \
$(SRC_DIR)/Core/AP/M802_1x/radius.o \
$(SRC_DIR)/Core/AP/M802_1x/radius_client.o \
$(SRC_DIR)/Core/AP/M802_1x/radius_timers.o \
$(SRC_DIR)/Core/AP/M802_1x/x_timers_auth.o \
$(SRC_DIR)/Core/AP/M802_11i/ieee_11i_ap.o \
$(SRC_DIR)/Core/AP/M802_11e/ap_frame_11e.o \
$(SRC_DIR)/Core/AP/M802_11e/ap_management_11e.o \
$(SRC_DIR)/Core/AP/M802_11e/ap_uapsd.o \
$(SRC_DIR)/Core/AP/P2P/ap_frame_p2p.o \
$(SRC_DIR)/Core/AP/P2P/ap_mgmt_p2p.o \
$(SRC_DIR)/Core/AP/P2P/p2p_wlan_rx_mgmt_ap.o \
$(SRC_DIR)/Core/AP/WPS/ap_management_wps.o \
$(SRC_DIR)/Core/AP/WPS/wps_ap.o \
$(SRC_DIR)/Core/AP/WPS/wps_cred_mgmt_ap.o \
$(SRC_DIR)/Core/AP/M802_11h/dfs_ap.o \
$(SRC_DIR)/Core/AP/M802_11h/frame_11h_ap.o \
$(SRC_DIR)/Core/AP/M802_11n/ap_frame_11n.o \
$(SRC_DIR)/Core/AP/M802_11n/ap_management_11n.o \
$(SRC_DIR)/Core/AP/M802_11n/ap_wlan_rx_mgmt_11n.o \
$(SRC_DIR)/Core/AP/MDOM/ap_management_mdom.o \
$(SRC_DIR)/Core/STA/autorate_sta.o \
$(SRC_DIR)/Core/STA/cglobals_sta.o \
$(SRC_DIR)/Core/STA/channel_sw.o \
$(SRC_DIR)/Core/STA/frame_sta.o \
$(SRC_DIR)/Core/STA/fsm_sta.o \
$(SRC_DIR)/Core/STA/host_rx_mlme_sta.o \
$(SRC_DIR)/Core/STA/host_rx_msdu_sta.o \
$(SRC_DIR)/Core/STA/management_sta.o \
$(SRC_DIR)/Core/STA/pm_sta.o \
$(SRC_DIR)/Core/STA/wlan_misc_sta.o \
$(SRC_DIR)/Core/STA/wlan_rx_data_sta.o \
$(SRC_DIR)/Core/STA/wlan_rx_mgmt_sta.o \
$(SRC_DIR)/Core/STA/M802_1x/ieee_1x_md5_supp.o \
$(SRC_DIR)/Core/STA/M802_1x/ieee_1x_sta.o \
$(SRC_DIR)/Core/STA/M802_1x/ieee_1x_supp.o \
$(SRC_DIR)/Core/STA/M802_1x/x_timers_supp.o \
$(SRC_DIR)/Core/STA/M802_11i/ieee_11i_sta.o \
$(SRC_DIR)/Core/STA/M802_11i/rkmal_sta.o \
$(SRC_DIR)/Core/STA/M802_11i/rsna_supp_4way.o \
$(SRC_DIR)/Core/STA/M802_11i/rsna_supp_km.o \
$(SRC_DIR)/Core/STA/M802_11i/sta_frame_11i.o \
$(SRC_DIR)/Core/STA/M802_11i/tkip_sta.o \
$(SRC_DIR)/Core/STA/M802_11e/sta_management_11e.o \
$(SRC_DIR)/Core/STA/M802_11e/sta_uapsd.o \
$(SRC_DIR)/Core/STA/P2P/p2p_host_rx_mlme_sta.o \
$(SRC_DIR)/Core/STA/P2P/p2p_ps_sta.o \
$(SRC_DIR)/Core/STA/P2P/p2p_wlan_rx_mgmt_sta.o \
$(SRC_DIR)/Core/STA/P2P/sta_frame_p2p.o \
$(SRC_DIR)/Core/STA/P2P/sta_mgmt_p2p.o \
$(SRC_DIR)/Core/STA/WPS/wps_sta.o \
$(SRC_DIR)/Core/STA/M802_11h/dfs_sta.o \
$(SRC_DIR)/Core/STA/M802_11h/ieee_11h_sta.o \
$(SRC_DIR)/Core/STA/M802_11h/tpc_sta.o \
$(SRC_DIR)/Core/STA/M802_11n/sta_frame_11n.o \
$(SRC_DIR)/Core/STA/M802_11n/sta_management_11n.o \
$(SRC_DIR)/Core/STA/MDOM/sta_management_mdom.o \
$(SRC_DIR)/Utils/mac_string.o\
$(SRC_DIR)/Test/trw/rw_register.o \
$(SRC_DIR)/Test/meter/meter_time.o \
$(SRC_DIR)/Test/meter/meter.o 

#leon liu added for powersave timer compiling
ifneq (ap, $(MAC_MODE))
$(TROUT_KO_NAME)-objs += $(SRC_DIR)/Core/STA/ps_timer.o
endif

#leon liu added for powersave timer compiling
ifeq (ap, $(MAC_MODE))
$(TROUT_KO_NAME)-objs += $(SRC_DIR)/CSL/hostap_conf.o
endif

#leon liu added for cfg80211 support
$(TROUT_KO_NAME)-objs += $(SRC_DIR)/CSL/trout_cfg80211.o

#zhuyg add for npi 2013-05-27
ifeq (npi, $(MAC_MODE))
$(TROUT_KO_NAME)-objs += $(SRC_DIR)/CSL/trout_wifi_npi.o
endif
