/*!
* \file    <mxd_castor_reg.h>
* \brief
* \par    Include files
* \par    (C) 2013 Maxscend Technologies Inc.
* \version
* Revision of last commit: $Rev: 66 $
* Author of last commit  : $Author: maxscend\zhenlin.liu $
* Date of last commit    : $Date: 2013-09-28 17:09:04 +0800 (??, 28 ?? 2013) $
*
*/



#ifndef _CASTOR_REG_H_
#define _CASTOR_REG_H_

#ifndef __USE_32BIT_PROCESSOR__
#define __USE_32BIT_PROCESSOR__
typedef unsigned char MXD_U8, *PMXD_U8;
typedef unsigned short MXD_U16, *PMXD_U16;
typedef unsigned int MXD_U32, *PMXD_U32;
typedef signed char MXD_S8, *PMXD_S8;
typedef signed short MXD_S16, *PMXD_S16;
typedef signed int MXD_S32, *PMXD_S32;
typedef signed char MXD_BOOL;
typedef char MXD_CHAR;
#endif
//#ifdef __cplusplus
//extern "C"{
//#endif

#if 1
//#define __CastorVersion2x
#define TROUT2_CHIP_VER_V2
#else
#define __CastorVersion2b
#endif

#define ctrl_soft_reset_reg 0x000
#define rfmux_rst_n_bit (1<<12)
#define frxp_rst_n_bit (1<<11)
#define wrxp_rst_n_bit (1<<10)
#define sdc_rst_n_bit (1<<9)
#define dcoc_rst_n_bit (1<<8)
#define txp_rst_n_bit (1<<7)
#define brxp_rst_n_bit (1<<6)
#define fd_rst_n_bit (1<<5)
#define wifi_sdm_rst_n_bit (1<<4)
#define bt_sdm_rst_n_bit (1<<3)
#define fm_sdm_rst_n_bit (1<<2)
#define afc_rst_n_bit (1<<1)
#define fmagc_rst_n_bit (1<<0)

#define ctrl_chip_id_reg 0x001
#define chip_id_shift 0
#define chip_id_mask ((1<<8)-1)

#define ctrl_fsm_start_reg 0x002
#define dcoc_bgn_bit (1<<1)
#define agc_start_bit (1<<0)

#define ctrl_tuner_status_reg 0x003
#define frxp_rcstd_done_bit (1<<5)
#define brxp_std_done_bit (1<<4)
#define wrxp_std_done_bit (1<<3)
#define frxp_std_done_bit (1<<2)
#define fd_tune_finish_sync_bit (1<<1)
#define dcoc_done_bit (1<<0)

#define ctrl_cgf_ctrl_reg 0x004
#define isolow_auto_bit (1<<12)
#define isolow_en_bit (1<<11)
#define ck26m_auto_en_bit (1<<10)
#define ck26m_buf_en_bit (1<<9)
#define gfclk_glb_gate_en_bit (1<<8)
#define gfclk40m_sel_26m_bit (1<<7)
#define gfclk44m_sel_26m_bit (1<<6)
#define gfclk48m_sel_26m_bit (1<<5)
#define gfclk80m_sel_26m_bit (1<<4)
#define gfclk40m_en_bit (1<<3)
#define gfclk44m_en_bit (1<<2)
#define gfclk48m_en_bit (1<<1)
#define gfclk80m_en_bit (1<<0)

#define ctrl_read_freeze_reg 0x005
#define frxp_dcr_dc_freeze_bit (1<<2)
#define frxp_imb_freeze_bit (1<<1)
#define fmagc_freeze_bit (1<<0)

#define ctrl_bt_adc_ctrl_reg 0x006
#define bt_adc_clk_edge_bit (1<<13)
#define bt_adc_iq_sel_bit (1<<12)
#define bt_adc_code_sel_bit (1<<11)
#define bt_adc_q_inv_bit (1<<10)
#define bt_adc_iq_ref_shift 0
#define bt_adc_iq_ref_mask ((1<<10)-1)

#define ctrl_fm_adc_ctrl_reg 0x007
#define fm_adc_clk_edge_bit (1<<13)
#define fm_adc_iq_sel_bit (1<<12)
#define fmadc_code_sel_bit (1<<11)
#define fm_adc_q_inv_bit (1<<10)
#define fm_adc_iq_ref_shift 0
#define fm_adc_iq_ref_mask ((1<<10)-1)

#define ctrl_wifi_adc_ctrl_reg 0x008
#define wifi_adc_clk_edge_bit (1<<13)
#define wifi_adc_iq_sel_bit (1<<12)
#define wifi_adc_code_sel_bit (1<<11)
#define wifi_adc_q_inv_bit (1<<10)
#define wifi_adc_iq_ref_shift 0
#define wifi_adc_iq_ref_mask ((1<<10)-1)

#define ctrl_bw_dac_ctrl_reg 0x009
#define wifi_loop_back_bit (1<<13)
#define bw_dac_clk_edge_bit (1<<12)
#define bw_dac_iq_sel_bit (1<<11)
#define bw_dac_q_inv_bit (1<<10)
#define bw_dac_iq_ref_shift 0
#define bw_dac_iq_ref_mask ((1<<10)-1)

#define ctrl_txp_clk_force_ctrl_reg 0x00a
#define txp_clk_force_26m_bit (1<<1)
#define txp_clk_force_40m_bit (1<<0)

#define misc_fpga_debug_port_sel_reg 0x00b
#define top_debug_sel_shift 12
#define top_debug_sel_mask ((1<<4)-1)
#define sub_debug_sel_shift 8
#define sub_debug_sel_mask ((1<<4)-1)
#define sig_debug_sel_shift 0
#define sig_debug_sel_mask ((1<<8)-1)

#define misc_fpga_test_wfrx_adc_i_scale_reg 0x00c
#define wfrx_iadc_scl_shift 0
#define wfrx_iadc_scl_mask ((1<<10)-1)

#define misc_fpga_test_wfrx_adc_q_scale_reg 0x00d
#define wfrx_qadc_scl_shift 0
#define wfrx_qadc_scl_mask ((1<<10)-1)

#define ctrl_bw_dac_debug_i_reg 0x00e
#define bw_dac_sel_bit (1<<15)
#define bw_dac_i_shift 0
#define bw_dac_i_mask ((1<<10)-1)

#define ctrl_bw_dac_debug_q_reg 0x00f
#define bw_dac_q_shift 0
#define bw_dac_q_mask ((1<<10)-1)

#define ctrl_sreg0_data 0x010
#define _RFREG_SW_RF_STATUS_  0x010
// bit0==1 wifi-work ;  bit1==1 bt work; bit0==1  fm work;
#define _FLAG_B_WIFI  (0)
#define _FLAG_B_BT   (1)
#define _FLAG_B_FM   (2)

#define ctrl_sreg1_data 0x011
#define _RFREG_SW_CFG_LDO  ctrl_sreg1_data

#define ctrl_sreg2_data 0x012
#define _RFREG_SW_CFG_PA_LNA ctrl_sreg2_data
/* pa_id= bit[15:8]            lna_id = bit [7:0]  */
/*  id==0 means without pa/lna */


#define ctrl_sreg3_data 0x013
/*this reg val is from sys0x58,;sys58 from nv; nv from hw switch-type*/
#define _RFREG_SW_ANTSEL_BK ctrl_sreg3_data

#define frxp_fm_dcr_ctrl_reg 0x014
#define dcr_dc_sel_bit (1<<5)
#define dcr_en_bit (1<<4)
#define dcr_hold_bit (1<<3)
#define dcr_n_shift 0
#define dcr_n_mask ((1<<3)-1)

#define frxp_fm_imb_ctrl_reg 0x015
#define imb_en_bit (1<<9)
#define imb_hold_bit (1<<8)
#define imb_g_shift 4
#define imb_g_mask ((1<<4)-1)
#define imb_p_shift 0
#define imb_p_mask ((1<<4)-1)

#define frxp_fm_std_ctrl_reg 0x016
#define std_en_bit (1<<3)
#define std_k_shift 0
#define std_k_mask ((1<<3)-1)

#define frxp_std_exp 0x017

#define frxp_fm_std_cos_reg 0x018
#define std_cos_shift 0
#define std_cos_mask ((1<<16)-1)

#define frxp_fm_std_sin_reg 0x019
#define std_sin_shift 0
#define std_sin_mask ((1<<16)-1)

#define frxp_fm_std_mag_status_reg 0x01a
#define std_mag_shift 0
#define std_mag_mask ((1<<16)-1)

#define frxp_fm_dcr_dc_real_reg 0x01b
#define dcr_dc_real_shift 0
#define dcr_dc_real_mask ((1<<10)-1)

#define frxp_fm_dcr_dc_imag_reg 0x01c
#define dcr_dc_imag_shift 0
#define dcr_dc_imag_mask ((1<<10)-1)

#define frxp_fm_imb_kg_reg 0x01d
#define imb_kg_shift 0
#define imb_kg_mask ((1<<16)-1)

#define frxp_fm_imb_kp_reg 0x01e
#define imb_kp_shift 0
#define imb_kp_mask ((1<<16)-1)

#define frxp_frxp_rcstd_ctrl_reg 0x01f
#define rcstd_en_bit (1<<4)
#define rcstd_n_bit (1<<3)
#define rcstd_k_shift 0
#define rcstd_k_mask ((1<<3)-1)

#define frxp_rcstd_bgn 0x020

#define frxp_frxp_rcstd_cos_reg 0x021
#define rcstd_cos_shift 0
#define rcstd_cos_mask ((1<<16)-1)

#define frxp_frxp_rcstd_sin_reg 0x022
#define rcstd_sin_shift 0
#define rcstd_sin_mask ((1<<16)-1)

#define frxp_frxp_rcstd_mag_status_reg 0x023
#define rcstd_mag_shift 0
#define rcstd_mag_mask ((1<<16)-1)

#define wrxp_wrxp_dcr_ctrl_reg 0x024
#define dcr_dc_sel_bit (1<<5)
#define dcr_en_bit (1<<4)
#define dcr_hold_bit (1<<3)
#define dcr_n_shift 0
#define dcr_n_mask ((1<<3)-1)

#define wrxp_wrxp_imb_ctrl_reg 0x025
#define imb_en_bit (1<<9)
#define imb_hold_bit (1<<8)
#define imb_g_shift 4
#define imb_g_mask ((1<<4)-1)
#define imb_p_shift 0
#define imb_p_mask ((1<<4)-1)

#define wrxp_wrxp_std_ctrl_reg 0x026
#define std_en_bit (1<<3)
#define std_k_shift 0
#define std_k_mask ((1<<3)-1)

#define wrxp_imb_std_exp_reg 0x027
#define dcr_dc_freeze_level_bit (1<<2)
#define std_exp_trig_level_bit (1<<1)
#define imb_freeze_level_bit (1<<0)

#define wrxp_wrxp_std_cos_reg 0x028
#define std_cos_shift 0
#define std_cos_mask ((1<<16)-1)

#define wrxp_wrxp_std_sin_reg 0x029
#define std_sin_shift 0
#define std_sin_mask ((1<<16)-1)

#define wrxp_wrxp_dcr_dc_real_reg 0x02a
#define dcr_dc_real_sync_shift 0
#define dcr_dc_real_sync_mask ((1<<10)-1)

#define wrxp_wrxp_dcr_dc_imag_reg 0x02b
#define dcr_dc_imag_sync_shift 0
#define dcr_dc_imag_sync_mask ((1<<10)-1)

#define wrxp_wrxp_imb_kg_reg 0x02c
#define imb_kg_sync_shift 0
#define imb_kg_sync_mask ((1<<16)-1)

#define wrxp_wrxp_imb_kp_reg 0x02d
#define imb_kp_sync_shift 0
#define imb_kp_sync_mask ((1<<16)-1)

#define wrxp_std_mag 0x02e
#define std_mag_sync_shift 0
#define std_mag_sync_mask ((1<<16)-1)

#define wrxp_wrxp_iqb_rxe1_reg 0x02f
#define iqb_rxe1_shift 0
#define iqb_rxe1_mask ((1<<16)-1)

#define wrxp_wrxp_iqb_rxe2_reg 0x030
#define iqb_rxe2_shift 0
#define iqb_rxe2_mask ((1<<16)-1)

#define wrxp_oc_sel_reg 0x031
#define cal_path_sel_shift 2
#define cal_path_sel_mask ((1<<2)-1)
#define oc_sel_bit (1<<1)
#define wfrx_2x2_en_bit (1<<0)

#define brxp_brxp_dcr_ctrl_reg 0x032
#define dcr_dc_sel_bit (1<<5)
#define dcr_en_bit (1<<4)
#define dcr_hold_bit (1<<3)
#define dcr_n_shift 0
#define dcr_n_mask ((1<<3)-1)

#define brxp_brxp_ctrl_freeze_reg 0x033
#define std_exp_bit (1<<2)
#define dcr_dc_freeze_bit (1<<1)
#define imb_freeze_bit (1<<0)

#define brxp_brxp_dcr_dc_real_reg 0x034
#define dcr_dc_real_shift 0
#define dcr_dc_real_mask ((1<<10)-1)

#define brxp_brxp_dcr_dc_imag_reg 0x035
#define dcr_dc_imag_shift 0
#define dcr_dc_imag_mask ((1<<10)-1)

#define brxp_brxp_rximb_rxe1_reg 0x036
#define rxe1_shift 0
#define rxe1_mask ((1<<16)-1)

#define brxp_brxp_rximb_rxe2_reg 0x037
#define rxe2_shift 0
#define rxe2_mask ((1<<16)-1)

#define brxp_brxp_imb_ctrl_reg 0x038
#define imb_en_bit (1<<9)
#define imb_hold_bit (1<<8)
#define imb_g_shift 4
#define imb_g_mask ((1<<4)-1)
#define imb_p_shift 0
#define imb_p_mask ((1<<4)-1)

#define brxp_brxp_imb_kp_reg 0x039
#define imb_kp_shift 0
#define imb_kp_mask ((1<<16)-1)

#define brxp_brxp_imb_kg_reg 0x03a
#define imb_kg_shift 0
#define imb_kg_mask ((1<<16)-1)

#define brxp_brxp_std_cos_reg 0x03b
#define std_cos_shift 0
#define std_cos_mask ((1<<16)-1)

#define brxp_brxp_std_sin_reg 0x03c
#define std_sin_shift 0
#define std_sin_mask ((1<<16)-1)

#define brxp_brxp_std_ctrl_reg 0x03d
#define std_en_bit (1<<3)
#define std_k_shift 0
#define std_k_mask ((1<<3)-1)

#define brxp_brxp_std_mag_status_reg 0x03e
#define std_mag_shift 0
#define std_mag_mask ((1<<16)-1)

#define brxp_btrx_2x2_en 0x03f

#define brxp_brxp_aci_reg 0x040
#define aci_bypass_bit (1<<2)
#define aci_fmov_reverse_bit (1<<1)
#define out_oc_sel_bit (1<<0)

#define txp_txp_ctrl_reg 0x041
#define txe_sel_shift 11
#define txe_sel_mask ((1<<2)-1)
#define dpd_en_sel_bit (1<<10)
#define bw_dat_lsb_shift 7
#define bw_dat_lsb_mask ((1<<3)-1)
#define dac_oci_bit (1<<6)
#define __dpd_en_bit (1<<5)
#define rfcali_tx_tone_en_bit (1<<4)
#define dual_tone_en_bit (1<<3)
#define iq_swap_bit (1<<2)
#define tx_2x2_en_bit (1<<1)
#define scale_en_bit (1<<0)

#define txp_txp_wtx_scale_factor_reg 0x042
#define wtx_scale_factor_shift 0
#define wtx_scale_factor_mask ((1<<8)-1)

#define txp_txp_lut1_size_reg 0x043
#define lut1_size_shift 0
#define lut1_size_mask ((1<<9)-1)

#define txp_txp_i_dc_reg 0x044
#define i_dc_shift 0
#define i_dc_mask ((1<<10)-1)

#define txp_txp_q_dc_reg 0x045
#define q_dc_shift 0
#define q_dc_mask ((1<<10)-1)

#define txp_txp_bttx_txe1_reg 0x046
#define bttx_txe1_shift 0
#define bttx_txe1_mask ((1<<12)-1)

#define txp_txp_bttx_txe2_reg 0x047
#define bttx_txe2_shift 0
#define bttx_txe2_mask ((1<<12)-1)

#define txp_txp_wftx_txe1_reg 0x048
#define wftx_txe1_shift 0
#define wftx_txe1_mask ((1<<12)-1)

#define txp_txp_wftx_txe2_reg 0x049
#define wftx_txe2_shift 0
#define wftx_txe2_mask ((1<<12)-1)

#define txp_txp_nco1_freq_h_reg 0x04a
#define nco1_freq_h_shift 0
#define nco1_freq_h_mask ((1<<10)-1)

#define txp_txp_nco1_freq_l_reg 0x04b
#define nco1_freq_l_shift 0
#define nco1_freq_l_mask ((1<<16)-1)

#define txp_txp_nco2_freq_h_reg 0x04c
#define nco2_freq_h_shift 0
#define nco2_freq_h_mask ((1<<10)-1)

#define txp_txp_nco2_freq_l_reg 0x04d
#define nco2_freq_l_shift 0
#define nco2_freq_l_mask ((1<<16)-1)

#define txp_txp_tssi_acc_status 0x04e
#define tssi_acc_sync_shift 0
#define tssi_acc_sync_mask ((1<<16)-1)

#define fd_btrx_tune_ctrl 0x04f

#define fd_bttx_tune_ctrl 0x050

#define fd_wifirx_tune_ctrl 0x051

#define fd_wifitx_tune_ctrl 0x052

#define fd_tune_sel 0x053

#define fd_btrx_lock_offset_reg 0x054
#define tuning_offset_btrx_ctrl_shift 0
#define tuning_offset_btrx_ctrl_mask ((1<<7)-1)

#define fd_bttx_lock_offset_reg 0x055
#define tuning_offset_bttx_ctrl_shift 0
#define tuning_offset_bttx_ctrl_mask ((1<<7)-1)

#define fd_wifirx_lock_offset_reg 0x056
#define tuning_offset_wifirx_ctrl_shift 0
#define tuning_offset_wifirx_ctrl_mask ((1<<7)-1)

#define fd_wifitx_lock_offset_reg 0x057
#define tuning_offset_wifitx_ctrl_shift 0
#define tuning_offset_wifitx_ctrl_mask ((1<<7)-1)

#define fd_btrx_status_reg 0x058
#define btrx_tune_lpf_sync_shift 0
#define btrx_tune_lpf_sync_mask ((1<<7)-1)

#define fd_bttx_status_reg 0x059
#define bttx_tune_lpf_sync_shift 0
#define bttx_tune_lpf_sync_mask ((1<<7)-1)

#define fd_wifirx_status_reg 0x05a
#define wifirx_tune_lpf_sync_shift 0
#define wifirx_tune_lpf_sync_mask ((1<<7)-1)

#define fd_wifitx_status_reg 0x05b
#define wifitx_tune_lpf_sync_shift 0
#define wifitx_tune_lpf_sync_mask ((1<<7)-1)

#define sdm_syn_divn_cfg1_reg 0x05c
#define fm_divn_msb_shift 0
#define fm_divn_msb_mask ((1<<10)-1)

#define sdm_syn_divn_cfg2_reg 0x05d
#define fm_divn_lsb_shift 0
#define fm_divn_lsb_mask ((1<<16)-1)

#define sdm_syn_divn_cfg3_reg 0x05e
#define bt_divn_msb_shift 0
#define bt_divn_msb_mask ((1<<10)-1)

#define sdm_syn_divn_cfg4_reg 0x05f
#define bt_divn_lsb_shift 0
#define bt_divn_lsb_mask ((1<<16)-1)

#define sdm_syn_divn_cfg5_reg 0x060
#define wifi_divn_msb_shift 0
#define wifi_divn_msb_mask ((1<<10)-1)

#define sdm_syn_divn_cfg6_reg 0x061
#define wifi_divn_lsb_shift 0
#define wifi_divn_lsb_mask ((1<<16)-1)

#define sdm_sdm_setup_reg 0x062
#define wifi_byp_mode_bit (1<<3)
#define bt_byp_mode_bit (1<<2)
#define fm_byp_mode_bit (1<<1)
#define dither_en_bit (1<<0)

#define afc_afc_setup_reg 0x063
#define tw_fm_shift 4
#define tw_fm_mask ((1<<2)-1)
#define tw_bt_shift 2
#define tw_bt_mask ((1<<2)-1)
#define tw_wifi_shift 0
#define tw_wifi_mask ((1<<2)-1)

#define afc_tune_fm 0x064

#define afc_tune_bt 0x065

#define afc_tune_wifi 0x066

#define afc_fvco_fm_config 0x067
#define byp_fm_bit (1<<14)
#define fvco_target_fm_shift 0
#define fvco_target_fm_mask ((1<<14)-1)

#define afc_fvco_bt_config 0x068
#define byp_bt_bit (1<<14)
#define fvco_target_bt_shift 0
#define fvco_target_bt_mask ((1<<14)-1)

#define afc_fvco_wifi_config 0x069
#define byp_wifi_bit (1<<14)
#define fvco_target_wifi_shift 0
#define fvco_target_wifi_mask ((1<<14)-1)

#define afc_tune_fm_status 0x06a

#define afc_tune_bt_status 0x06b

#define afc_tune_wifi_status 0x06c

#define fmagc_setup_reg 0x06d
#define mode_shift 0
#define mode_mask ((1<<2)-1)

#define fmagc_initial_dbm_reg 0x06e
#define ad_pwr_shift 8
#define ad_pwr_mask ((1<<8)-1)
#define init_dbm_shift 0
#define init_dbm_mask ((1<<8)-1)

#define fmagc_initial_gidx_reg 0x06f
#define sprd_gidx_shift 5
#define sprd_gidx_mask ((1<<5)-1)
#define init_gidx_shift 0
#define init_gidx_mask ((1<<5)-1)

#define fmagc_rssi_len_wait_time_reg 0x070
#define nc_shift 8
#define nc_mask ((1<<2)-1)
#define nw_shift 0
#define nw_mask ((1<<8)-1)

#define fmagc_gidx_status_reg 0x071
#define gidx_shift 0
#define gidx_mask ((1<<5)-1)

#define fmagc_rssi_status_reg 0x072
#define rssi_shift 0
#define rssi_mask ((1<<10)-1)

#define dcoc_dcoc_switch_timer_reg 0x073
#define switch_tw_shift 0
#define switch_tw_mask ((1<<8)-1)

#define dcoc_dcoc_path_switch_reg 0x074
#define dcocsw_bt_rx_pga_i_bit (1<<7)
#define dcocsw_bt_rx_pga_q_bit (1<<6)
#define dcocsw_bt_rx_tia_i_bit (1<<5)
#define dcocsw_bt_rx_tia_q_bit (1<<4)
#define dcocsw_wifi_rx_pga_i_bit (1<<3)
#define dcocsw_wifi_rx_pga_q_bit (1<<2)
#define dcocsw_wifi_rx_tia_i_bit (1<<1)
#define dcocsw_wifi_rx_tia_q_bit (1<<0)

#define dcoc_dcoc_ctrl_reg 0x075
#define sample_num_shift 0
#define sample_num_mask ((1<<4)-1)

#define dcoc_rfcali_dcoc_gain_settle_tw 0x076
#define gain_settle_tw_shift 0
#define gain_settle_tw_mask ((1<<14)-1)

#define dcoc_rfcali_dcoc_idac_settle_tw 0x077
#define idac_settle_tw_shift 0
#define idac_settle_tw_mask ((1<<14)-1)

#define dcoc_rfcali_dcoc_init_settle_tw 0x078
#define init_settle_tw_shift 0
#define init_settle_tw_mask ((1<<14)-1)

#define dcoc_rfcali_dcoc_ipath_dc_target 0x079
#define ipath_dc_target_shift 0
#define ipath_dc_target_mask ((1<<10)-1)

#define dcoc_rfcali_dcoc_qpath_dc_target 0x07a
#define qpath_dc_target_shift 0
#define qpath_dc_target_mask ((1<<10)-1)

#define dcoc_rfcali_fm_idat_reg 0x07b
#define rfcali_fm_idat_shift 0
#define rfcali_fm_idat_mask ((1<<10)-1)

#define dcoc_rfcali_fm_qdat_reg 0x07c
#define rfcali_fm_qdat_shift 0
#define rfcali_fm_qdat_mask ((1<<10)-1)

#define dcoc_rfcali_tbl_entry_0_status_reg 0x07d
#define tbl_entry_0_reg_shift 0
#define tbl_entry_0_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_1_status_reg 0x07e
#define tbl_entry_1_reg_shift 0
#define tbl_entry_1_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_2_status_reg 0x07f
#define tbl_entry_2_reg_shift 0
#define tbl_entry_2_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_3_status_reg 0x080
#define tbl_entry_3_reg_shift 0
#define tbl_entry_3_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_4_status_reg 0x081
#define tbl_entry_4_reg_shift 0
#define tbl_entry_4_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_5_status_reg 0x082
#define tbl_entry_5_reg_shift 0
#define tbl_entry_5_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_6_status_reg 0x083
#define tbl_entry_6_reg_shift 0
#define tbl_entry_6_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_7_status_reg 0x084
#define tbl_entry_7_reg_shift 0
#define tbl_entry_7_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_8_status_reg 0x085
#define tbl_entry_8_reg_shift 0
#define tbl_entry_8_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_9_status_reg 0x086
#define tbl_entry_9_reg_shift 0
#define tbl_entry_9_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_10_status_reg 0x087
#define tbl_entry_10_reg_shift 0
#define tbl_entry_10_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_11_status_reg 0x088
#define tbl_entry_11_reg_shift 0
#define tbl_entry_11_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_12_status_reg 0x089
#define tbl_entry_12_reg_shift 0
#define tbl_entry_12_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_13_status_reg 0x08a
#define tbl_entry_13_reg_shift 0
#define tbl_entry_13_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_14_status_reg 0x08b
#define tbl_entry_14_reg_shift 0
#define tbl_entry_14_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_15_status_reg 0x08c
#define tbl_entry_15_reg_shift 0
#define tbl_entry_15_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_16_status_reg 0x08d
#define tbl_entry_16_reg_shift 0
#define tbl_entry_16_reg_mask ((1<<16)-1)


#define dcoc_rfcali_tbl_entry_17_status_reg 0x08e
#define tbl_entry_17_reg_shift 0
#define tbl_entry_17_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_18_status_reg 0x08f
#define tbl_entry_18_reg_shift 0
#define tbl_entry_18_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_19_status_reg 0x090
#define tbl_entry_19_reg_shift 0
#define tbl_entry_19_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_20_status_reg 0x091
#define tbl_entry_20_reg_shift 0
#define tbl_entry_20_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_21_status_reg 0x092
#define tbl_entry_21_reg_shift 0
#define tbl_entry_21_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_22_status_reg 0x093
#define tbl_entry_22_reg_shift 0
#define tbl_entry_22_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_23_status_reg 0x094
#define tbl_entry_23_reg_shift 0
#define tbl_entry_23_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_24_status_reg 0x095
#define tbl_entry_24_reg_shift 0
#define tbl_entry_24_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_25_status_reg 0x096
#define tbl_entry_25_reg_shift 0
#define tbl_entry_25_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_26_status_reg 0x097
#define tbl_entry_26_reg_shift 0
#define tbl_entry_26_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_27_status_reg 0x098
#define tbl_entry_27_reg_shift 0
#define tbl_entry_27_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_28_status_reg 0x099
#define tbl_entry_28_reg_shift 0
#define tbl_entry_28_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_29_status_reg 0x09a
#define tbl_entry_29_reg_shift 0
#define tbl_entry_29_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_30_status_reg 0x09b
#define tbl_entry_30_reg_shift 0
#define tbl_entry_30_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_31_status_reg 0x09c
#define tbl_entry_31_reg_shift 0
#define tbl_entry_31_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_32_status_reg 0x09d
#define tbl_entry_32_reg_shift 0
#define tbl_entry_32_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_33_status_reg 0x09e
#define tbl_entry_33_reg_shift 0
#define tbl_entry_33_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_34_status_reg 0x09f
#define tbl_entry_34_reg_shift 0
#define tbl_entry_34_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_35_status_reg 0x0a0
#define tbl_entry_35_reg_shift 0
#define tbl_entry_35_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_36_status_reg 0x0a1
#define tbl_entry_36_reg_shift 0
#define tbl_entry_36_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_37_status_reg 0x0a2
#define tbl_entry_37_reg_shift 0
#define tbl_entry_37_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_38_status_reg 0x0a3
#define tbl_entry_38_reg_shift 0
#define tbl_entry_38_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_39_status_reg 0x0a4
#define tbl_entry_39_reg_shift 0
#define tbl_entry_39_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_40_status_reg 0x0a5
#define tbl_entry_40_reg_shift 0
#define tbl_entry_40_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_41_status_reg 0x0a6
#define tbl_entry_41_reg_shift 0
#define tbl_entry_41_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_42_status_reg 0x0a7
#define tbl_entry_42_reg_shift 0
#define tbl_entry_42_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_43_status_reg 0x0a8
#define tbl_entry_43_reg_shift 0
#define tbl_entry_43_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_44_status_reg 0x0a9
#define tbl_entry_44_reg_shift 0
#define tbl_entry_44_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_45_status_reg 0x0aa
#define tbl_entry_45_reg_shift 0
#define tbl_entry_45_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_46_status_reg 0x0ab
#define tbl_entry_46_reg_shift 0
#define tbl_entry_46_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_47_status_reg 0x0ac
#define tbl_entry_47_reg_shift 0
#define tbl_entry_47_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_48_status_reg 0x0ad
#define tbl_entry_48_reg_shift 0
#define tbl_entry_48_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_49_status_reg 0x0ae
#define tbl_entry_49_reg_shift 0
#define tbl_entry_49_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_50_status_reg 0x0af
#define tbl_entry_50_reg_shift 0
#define tbl_entry_50_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_51_status_reg 0x0b0
#define tbl_entry_51_reg_shift 0
#define tbl_entry_51_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_52_status_reg 0x0b1
#define tbl_entry_52_reg_shift 0
#define tbl_entry_52_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_53_status_reg 0x0b2
#define tbl_entry_53_reg_shift 0
#define tbl_entry_53_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_54_status_reg 0x0b3
#define tbl_entry_54_reg_shift 0
#define tbl_entry_54_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_55_status_reg 0x0b4
#define tbl_entry_55_reg_shift 0
#define tbl_entry_55_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_56_status_reg 0x0b5
#define tbl_entry_56_reg_shift 0
#define tbl_entry_56_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_57_status_reg 0x0b6
#define tbl_entry_57_reg_shift 0
#define tbl_entry_57_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_58_status_reg 0x0b7
#define tbl_entry_58_reg_shift 0
#define tbl_entry_58_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_59_status_reg 0x0b8
#define tbl_entry_59_reg_shift 0
#define tbl_entry_59_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_60_status_reg 0x0b9
#define tbl_entry_60_reg_shift 0
#define tbl_entry_60_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_61_status_reg 0x0ba
#define tbl_entry_61_reg_shift 0
#define tbl_entry_61_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_62_status_reg 0x0bb
#define tbl_entry_62_reg_shift 0
#define tbl_entry_62_reg_mask ((1<<16)-1)

#define dcoc_rfcali_tbl_entry_63_status_reg 0x0bc
#define tbl_entry_63_reg_shift 0
#define tbl_entry_63_reg_mask ((1<<16)-1)

#define rfmux_mode_sel_reg 0x0bd
#define cali_type_shift 5
#define cali_type_mask ((1<<2)-1)
#define bw_ctrl_shift 3
#define bw_ctrl_mask ((1<<2)-1)
#define bw_mode_shift 0
#define bw_mode_mask ((1<<3)-1)

#define _RF_MODE_INIT       (0)
#define _RF_MODE_WIFI_TX    (1)
#define _RF_MODE_BT_TX      (2)
#define _RF_MODE_WIFI_RX    (3)
#define _RF_MODE_BT_RX      (4)
#define _RF_MODE_RX_COL     (3)
#define rfmux_sprd_mode_sel_reg 0x0be
#define sprd_mode_3_entry_shift 9
#define sprd_mode_3_entry_mask ((1<<3)-1)
#define sprd_mode_2_entry_shift 6
#define sprd_mode_2_entry_mask ((1<<3)-1)
#define sprd_mode_1_entry_shift 3
#define sprd_mode_1_entry_mask ((1<<3)-1)
#define sprd_mode_0_entry_shift 0
#define sprd_mode_0_entry_mask ((1<<3)-1)

#define rfmux_gain_table_sel_reg 0x0bf
#define bwco_priority_bit (1<<8)
#define lna_gain_sel_bit (1<<7)
#define pa_sel_bit (1<<6)
#define btrx_gain_sel_bit (1<<5)
#define bttx_gain_sel_bit (1<<4)
#define wfrx_gain_sel_shift 2
#define wfrx_gain_sel_mask ((1<<2)-1)
#define wftx_gain_sel_shift 0
#define wftx_gain_sel_mask ((1<<2)-1)

#define rfmux_btrx_gidx_reg 0x0c0
#define btrx_gidx_shift 0
#define btrx_gidx_mask ((1<<7)-1)

#define rfmux_bttx_gidx_reg 0x0c1
#define bttx_gidx_shift 0
#define bttx_gidx_mask ((1<<5)-1)

#define rfmux_wfrx_gidx_reg 0x0c2
#define wfrx_gidx_map_bit (1<<7)
#define wfrx_gidx_shift 0
#define wfrx_gidx_mask ((1<<7)-1)

#define rfmux_wftx_gidx_reg 0x0c3
#define wftx_sprd_gidx_sel_bit (1<<6)
#define __wftx_gidx_shift 0
#define wftx_gidx_mask ((1<<6)-1)

#define rfmux_sprd_wifi_gidx_reg 0x0c4
#define sprd_wifi_gidx_shift 0
#define sprd_wifi_gidx_mask ((1<<7)-1)

#define rfmux_wifi_actual_gidx_reg 0x0c5
#define _wftx_gidx_shift 7
#define wftx_gidx_mask ((1<<6)-1)
#define wfrx_gidx_shift 0
#define wfrx_gidx_mask ((1<<7)-1)

#define rfmux_sprd_sig_control_reg 0x0c6
#define sprd_ctrl_bt_adc_en_bit (1<<2)
#define sprd_ctrl_tx_dac_en_bit (1<<1)
#define sprd_ctrl_wifi_adc_en_bit (1<<0)

#define rfmux_loft_pout_cfg_reg 0x0c7
#define bttx_loft_sel_bit (1<<7)
#define wftx_loft_sel_bit (1<<6)
#define negm_sel_bit (1<<5)
#define wftx_negm_sel_bit (1<<4)
#define pout_cfg_sel_bit (1<<3)
#define wftx_pout_cfg_sel_bit (1<<2)
#define band_sel_shift 0
#define band_sel_mask ((1<<2)-1)

#define rfmux_pout_tx_scale_reg 0x0c8
#define pout_rfid_bit (1<<10)
#define pout_tx_scale_shift 0
#define pout_tx_scale_mask ((1<<10)-1)

#define rfmux_pout_id_reg 0x0c9
#define _dpd_en_bit (1<<13)
#define bttx_pa_id_shift 10
#define bttx_pa_id_mask ((1<<3)-1)
#define bttx_loft_id_shift 8
#define bttx_loft_id_mask ((1<<2)-1)
#define wftx_pout_cfg_id_shift 6
#define wftx_pout_cfg_id_mask ((1<<2)-1)
#define wftx_pa_id_shift 2
#define wftx_pa_id_mask ((1<<4)-1)
#define wftx_loft_id_shift 0
#define wftx_loft_id_mask ((1<<2)-1)

#define rfmux_mem_read_addr_reg 0x0ca
#define mem_read_addr_shift 0
#define mem_read_addr_mask ((1<<11)-1)

#define rfmux_mem_read_trig 0x0cb

#define rfmux_mem_read_data_reg 0x0cc
#define mem_read_data_shift 0
#define mem_read_data_mask ((1<<16)-1)

#define misc_fpga_adda_config_reg 0x0cd
#define fpga_txpwrdwn_bit (1<<4)
#define fpga_rxpwrdwn_bit (1<<3)
#define fpga_rx_clk_sel_bit (1<<2)
#define fpga_dac_tx_rx_bit (1<<1)
#define fpga_adc_en_bit (1<<0)

#define misc_fpga_adi_sel 0x151

#define rfmux_cal_cfg_reg 0x200
#define complex_en_bit (1<<2)
#define sido_en_bit (1<<1)
#define pdet_pos_bit (1<<0)

#define rf_ldo_ctrl_reg 0x201
#define ldo_en_bak_shift 14
#define ldo_en_bak_mask ((1<<2)-1)
#define ldo_dab_en_bit (1<<13)
#define ldo_pa_en_bit (1<<12)
#define ldo_lcpll_en_1v_bit (1<<11)
#define ldo_cp_en_1v_wifi_bit (1<<10)
#define ldo_lo_en_1v_wifi_bit (1<<9)
#define ldo_cp_en_1v_bt_bit (1<<8)
#define ldo_lo_en_1v_bt_bit (1<<7)
#define ldo_cp_en_1v_fm_bit (1<<6)
#define ldo_lo_en_1v_fm_bit (1<<5)
#define fm_ldo_en_bit (1<<4)
#define ldo_rx_en_bit (1<<3)
#define ldo_tx_en_bit (1<<2)
#define ldo_adc_en_bit (1<<1)
#define bg_ana_en_bit (1<<0)

#define rfmux_fm_gain_reg 0x202
#define fm_rf_gm_sel_bit (1<<6)
#define fm_tia_gain_shift 3
#define fm_tia_gain_mask ((1<<3)-1)
#define fm_pga_gain_shift 0
#define fm_pga_gain_mask ((1<<3)-1)

#define rfmux_fm_cfg_reg 0x203
#define fm_lo_en_bit (1<<9)
#define fm_lo_rstn_bit (1<<8)
#define fmrx_bias_ictrl_shift 6
#define fmrx_bias_ictrl_mask ((1<<2)-1)
#define fm_dcoc_en_bit (1<<5)
#define fm_adc_en_bit (1<<4)
#define fm_filter_en_bit (1<<3)
#define fm_pga_en_bit (1<<2)
#define fm_rx_bias_en_bit (1<<1)
#define fm_mixer_tia_en_bit (1<<0)

#define rf_ldo_cfg_reg 0x204
#define ldo_cfg_bak_shift 8
#define ldo_cfg_bak_mask ((1<<8)-1)
#define ldo_pa_ctrl_shift 6
#define ldo_pa_ctrl_mask ((1<<2)-1)
#define fm_ldo_ctrl_shift 4
#define fm_ldo_ctrl_mask ((1<<2)-1)
#define ldo_rx_ctrl_shift 2
#define ldo_rx_ctrl_mask ((1<<2)-1)
#define ldo_tx_ctrl_shift 0
#define ldo_tx_ctrl_mask ((1<<2)-1)

#define rfmux_mode_cfg_ctrl0_reg 0x205
#define bt_tia_dcoc_en_bit (1<<15)
#define wifi_tia_dcoc_en_bit (1<<14)
#define wifi_swbias_en_bit (1<<13)
#define bt_swbias_en_bit (1<<12)
#define rx_bias_en_bit (1<<11)
#define bt_adc_en_bit (1<<10)
#define wifi_adc_en_bit (1<<9)
#define tx_dac_en_bit (1<<8)
#define bt_rx_if_en_bit (1<<7)
#define bt_pga_dcoc_en_bit (1<<6)
#define wifi_rx_filter_en_bit (1<<5)
#define wifi_rx_pga_en_bit (1<<4)
#define wifi_pga_dcoc_en_bit (1<<3)
#define bt_tx_filter_en_bit (1<<2)
#define bt_tx_pga_en_bit (1<<1)
#define bt_loft_dac_en_bit (1<<0)

#define rfmux_mode_cfg_ctrl1_reg 0x206
#define wifi_rx_lobuf_en_bit (1<<5)
#define tx_lobuf_en_bit (1<<4)
#define lomux2_en_bit (1<<3)
#define lomux1_en_bit (1<<2)
#define bt_tx_en_bit (1<<1)
#define bt_rx_lobuf_en_bit (1<<0)

#define rfmux_mode_cfg_ctrl2_reg 0x207
#define mode_backup2_shift 13
#define mode_backup2_mask ((1<<3)-1)
#define wifi_tx_filter_en_bit (1<<11)
#define wifi_tx_pga_en_bit (1<<10)
#define wifi_loft_dac_en_bit (1<<9)
#define tx_bias_en_bit (1<<8)
#define wifi_tia_en_bit (1<<7)
#define bt_tia_en_bit (1<<6)
#define txgm_boost_en_bit (1<<5)
#define txgm_modulator_en_bit (1<<4)
#define rf_tx_enable_bit (1<<3)
#define rf_rx_enable_bit (1<<2)
#define sw_enable_bit (1<<1)
#define pdet_enable_bit (1<<0)

#define rf_pa_lna_cfg_reg 0x208
#define rf_backup_shift 10
#define rf_backup_mask ((1<<6)-1)
#define lna_outcap_tuning_shift 6
#define lna_outcap_tuning_mask ((1<<4)-1)
#define pdet_gain_shift 4
#define pdet_gain_mask ((1<<2)-1)
#define pad_aux1_en_bit (1<<3)
#define pad_aux2_en_bit (1<<2)
#define pa_aux1_en_bit (1<<1)
#define pa_aux2_en_bit (1<<0)

#define rf_lna_sw_bias_ctrl1_reg 0x209
#define sw_bias_ctrl_shift 8
#define sw_bias_ctrl_mask ((1<<8)-1)
#define lna_ctrl_vb2_shift 0
#define lna_ctrl_vb2_mask ((1<<8)-1)

#define rfmux_lna_sw_bias_ctrl2_reg 0x20a
#define lna_ctrl_vb1_main_shift 8
#define lna_ctrl_vb1_main_mask ((1<<8)-1)
#define lna_ctrl_vb1_aux_shift 0
#define lna_ctrl_vb1_aux_mask ((1<<8)-1)

#define rfmux_pad_bias_ctrl1_reg 0x20b
#define pad_ctrl_vb2_shift 8
#define pad_ctrl_vb2_mask ((1<<8)-1)
#define pad_ctrl_vb1_main_shift 0
#define pad_ctrl_vb1_main_mask ((1<<8)-1)

#define rfmux_pad_bias_ctrl2_reg 0x20c
#define pad_ctrl_vb1_aux1_shift 8
#define pad_ctrl_vb1_aux1_mask ((1<<8)-1)
#define pad_ctrl_vb1_aux2_shift 0
#define pad_ctrl_vb1_aux2_mask ((1<<8)-1)

#define rfmux_pa_bias_ctrl1_reg 0x20d
#define pa_ctrl_vb2_shift 8
#define pa_ctrl_vb2_mask ((1<<8)-1)
#define pa_ctrl_vb1_main_shift 0
#define pa_ctrl_vb1_main_mask ((1<<8)-1)

#define rfmux_pa_bias_ctrl2_reg 0x20e
#define pa_ctrl_vb1_aux1_shift 8
#define pa_ctrl_vb1_aux1_mask ((1<<8)-1)
#define pa_ctrl_vb1_aux2_shift 0
#define pa_ctrl_vb1_aux2_mask ((1<<8)-1)

#define rfmux_wb_rx_gm_reg 0x20f
#define bt_gm_ctrl_shift 10
#define bt_gm_ctrl_mask ((1<<2)-1)
#define wifi_gm_ctrl_shift 8
#define wifi_gm_ctrl_mask ((1<<2)-1)
#define bt_gm_id_shift 4
#define bt_gm_id_mask ((1<<4)-1)
#define wifi_gm_id_shift 0
#define wifi_gm_id_mask ((1<<4)-1)

#define rfmux_wb_rx_gain_reg 0x210
#define wifi_rx_pga_gain_shift 12
#define wifi_rx_pga_gain_mask ((1<<4)-1)
#define wifi_tia_gain_shift 9
#define wifi_tia_gain_mask ((1<<3)-1)
#define bt_rx_pga_gain_shift 6
#define bt_rx_pga_gain_mask ((1<<3)-1)
#define bt_tia_gain_shift 3
#define bt_tia_gain_mask ((1<<3)-1)
#define lna_gain_shift 0
#define lna_gain_mask ((1<<3)-1)

#define rfmux_wb_tx_gain_reg 0x211
#define wifi_tx_pga_gain_shift 7
#define wifi_tx_pga_gain_mask ((1<<4)-1)
#define bt_tx_pga_gain_shift 3
#define bt_tx_pga_gain_mask ((1<<4)-1)
#define txgm_rfgain_shift 0
#define txgm_rfgain_mask ((1<<3)-1)

#define rf_wb_rx_cfg_reg 0x212
#define wifi_rx_backup_shift 6
#define wifi_rx_backup_mask ((1<<10)-1)
#define bt_tia_res_bit (1<<5)
#define wifi_tia_res_bit (1<<4)
#define wifi_tia_lo_bias_shift 2
#define wifi_tia_lo_bias_mask ((1<<2)-1)
#define bt_tia_lo_bias_shift 0
#define bt_tia_lo_bias_mask ((1<<2)-1)

#define rfmux_txgm_ctrl_reg 0x213
#define txgm_negm_ctrl_shift 4
#define txgm_negm_ctrl_mask ((1<<4)-1)
#define txgm_cap_ctrl_shift 0
#define txgm_cap_ctrl_mask ((1<<4)-1)

#define rf_wb_tx_cfg_reg 0x214
#define tia_bak_bit (1<<7)
#define txgm_lobias_ctrl_bit (1<<6)
#define txgm_cas_ctrl_shift 4
#define txgm_cas_ctrl_mask ((1<<2)-1)
#define tx_bias_ictrl_shift 2
#define tx_bias_ictrl_mask ((1<<2)-1)
#define rx_bias_ictrl_shift 0
#define rx_bias_ictrl_mask ((1<<2)-1)

#define rf_ts_ctrl_reg 0x215
#define ant_sel1_en_bit (1<<15)
#define test_out_en_bit (1<<14)
#define temp_bak_shift 8
#define temp_bak_mask ((1<<6)-1)
#define temp_sense_shift 0
#define temp_sense_mask ((1<<8)-1)

#define rfmux_bt_rx_dcoc_ctrl0_reg 0x216
#define bt_pga_dcoci_shift 8
#define bt_pga_dcoci_mask ((1<<8)-1)
#define bt_pga_dcocq_shift 0
#define bt_pga_dcocq_mask ((1<<8)-1)

#define rfmux_bt_rx_dcoc_ctrl1_reg 0x217
#define bt_tia_dcoci_shift 8
#define bt_tia_dcoci_mask ((1<<8)-1)
#define bt_tia_dcocq_shift 0
#define bt_tia_dcocq_mask ((1<<8)-1)

#define rfmux_wf_rx_dcoc_ctrl0_reg 0x218
#define wifi_pga_dcoci_shift 8
#define wifi_pga_dcoci_mask ((1<<8)-1)
#define wifi_pga_dcocq_shift 0
#define wifi_pga_dcocq_mask ((1<<8)-1)

#define rfmux_wf_rx_dcoc_ctrl1_reg 0x219
#define wifi_tia_dcoci_shift 8
#define wifi_tia_dcoci_mask ((1<<8)-1)
#define wifi_tia_dcocq_shift 0
#define wifi_tia_dcocq_mask ((1<<8)-1)

#define rfmux_bt_tx_loft_ctrl_reg 0x21a
#define bt_loft_dac_i_shift 8
#define bt_loft_dac_i_mask ((1<<8)-1)
#define bt_loft_dac_q_shift 0
#define bt_loft_dac_q_mask ((1<<8)-1)

#define rfmux_wf_tx_loft_ctrl_reg 0x21b
#define wifi_loft_dac_i_shift 8
#define wifi_loft_dac_i_mask ((1<<8)-1)
#define wifi_loft_dac_q_shift 0
#define wifi_loft_dac_q_mask ((1<<8)-1)

#define rfmux_fm_rx_dcoc_ctrl_reg 0x21c
#define fm_rx_tia_dcoci_shift 8
#define fm_rx_tia_dcoci_mask ((1<<8)-1)
#define fm_rx_tia_dcocq_shift 0
#define fm_rx_tia_dcocq_mask ((1<<8)-1)

#define rf_test_ctrl0_reg 0x21d
#define test0_ctrl_shift 0
#define test0_ctrl_mask ((1<<16)-1)

#define rf_test_ctrl1_reg 0x21e
#define test_bak_shift 7
#define test_bak_mask ((1<<9)-1)
#define test_en_bit (1<<6)
#define temp_sense_en_bit (1<<5)
#define tuning_en_bit (1<<4)
#define test1_ctrl_shift 0
#define test1_ctrl_mask ((1<<4)-1)

#define rf_lobuffer_cfg_reg 0x21f
#define wifi_rx_lobuf_nolap_bit (1<<8)
#define tx_lobuf_ctrl_shift 6
#define tx_lobuf_ctrl_mask ((1<<2)-1)
#define sel_lo_pad_bit (1<<5)
#define rx_lobuf_ctrl_shift 3
#define rx_lobuf_ctrl_mask ((1<<2)-1)
#define lomixer_wifi_en_bit (1<<2)
#define lomixer_bt_en_bit (1<<1)
#define bt_rx_lobuf_nolap_bit (1<<0)

#define rf_syn_clk_cfg_reg 0x220
#define syn_bak_shift 3
#define syn_bak_mask ((1<<2)-1)
#define clk_80m_en_bit (1<<2)
#define clk_48m_en_bit (1<<1)
#define clk_44m_en_bit (1<<0)

#define rf_syn_cp_cfg_reg 0x221
#define cp_ctrl_lcpll_shift 12
#define cp_ctrl_lcpll_mask ((1<<4)-1)
#define cp_ctrl_wifi_shift 8
#define cp_ctrl_wifi_mask ((1<<4)-1)
#define cp_ctrl_bt_shift 4
#define cp_ctrl_bt_mask ((1<<4)-1)
#define cp_ctrl_fm_shift 0
#define cp_ctrl_fm_mask ((1<<4)-1)

#define rf_bt_lomixer_cfg_reg 0x222
#define bt_lomixer_cfg_shift 0
#define bt_lomixer_cfg_mask ((1<<16)-1)

#define rf_wifi_lomixer_cfg_reg 0x223
#define wifi_lomixer_cfg_shift 0
#define wifi_lomixer_cfg_mask ((1<<16)-1)

#define rf_vco_cfg_lcpll_reg 0x224
#define vco_cfg_lcpll_shift 0
#define vco_cfg_lcpll_mask ((1<<6)-1)

#define rf_syn_lcpll_cfg_reg 0x225
#define fref_divn_shift 8
#define fref_divn_mask ((1<<4)-1)
#define tune_lcpll_shift 0
#define tune_lcpll_mask ((1<<8)-1)

#define rf_vco_cfg_btwifi_reg 0x226
#define vco_cfg_bt_wifi_shift 0
#define vco_cfg_bt_wifi_mask ((1<<12)-1)

#define rf_vco_cfg_fm_reg 0x227
#define lo_div45_sel_fm_bit (1<<6)
#define vco_res_fm_shift 0
#define vco_res_fm_mask ((1<<6)-1)

#define rf_syn_lotest_reg 0x228
#define wifi_lotest_en_bit (1<<7)
#define fref_sel_wifi_bit (1<<6)
#define fref_sel_bt_bit (1<<5)
#define fref_sel_fm_bit (1<<4)
#define pll_44mhz_test_en_bit (1<<3)
#define pll_40mhz_test_en_bit (1<<2)
#define bt_lo_test_en_bit (1<<1)
#define fm_lo_test_en_bit (1<<0)

#define afc_syn_status_reg 0x229
#define temp_sense1_out_bit (1<<8)
#define vco_slow_fm_bit (1<<7)
#define vco_slow_bt_bit (1<<6)
#define vco_slow_wifi_bit (1<<5)
#define vco_slow_lcpll_bit (1<<4)
#define vco_lock_fm_bit (1<<3)
#define vco_lock_bt_bit (1<<2)
#define vco_lock_wifi_bit (1<<1)
#define vco_lock_lcpll_bit (1<<0)

#define rfmux_mode_en0_h_tbl 0x400
#define tbl_mode_en0_h_shift 0
#define tbl_mode_en0_h_mask ((1<<10)-1)

#define rfmux_mode_en0_m_tbl 0x401
#define tbl_mode_en0_m_shift 0
#define tbl_mode_en0_m_mask ((1<<16)-1)

#define rfmux_mode_en0_l_tbl 0x402
#define tbl_mode_en0_l_shift 0
#define tbl_mode_en0_l_mask ((1<<16)-1)

#define rfmux_mode_en1_h_tbl 0x403
#define tbl_mode_en1_h_shift 0
#define tbl_mode_en1_h_mask ((1<<10)-1)

#define rfmux_mode_en1_m_tbl 0x404
#define tbl_mode_en1_m_shift 0
#define tbl_mode_en1_m_mask ((1<<16)-1)

#define rfmux_mode_en1_l_tbl 0x405
#define tbl_mode_en1_l_shift 0
#define tbl_mode_en1_l_mask ((1<<16)-1)

#define rfmux_mode_en2_h_tbl 0x406
#define tbl_mode_en2_h_shift 0
#define tbl_mode_en2_h_mask ((1<<10)-1)

#define rfmux_mode_en2_m_tbl 0x407
#define tbl_mode_en2_m_shift 0
#define tbl_mode_en2_m_mask ((1<<16)-1)

#define rfmux_mode_en2_l_tbl 0x408
#define tbl_mode_en2_l_shift 0
#define tbl_mode_en2_l_mask ((1<<16)-1)

#define rfmux_mode_en3_h_tbl 0x409
#define tbl_mode_en3_h_shift 0
#define tbl_mode_en3_h_mask ((1<<10)-1)

#define rfmux_mode_en3_m_tbl 0x40a
#define tbl_mode_en3_m_shift 0
#define tbl_mode_en3_m_mask ((1<<16)-1)

#define rfmux_mode_en3_l_tbl 0x40b
#define tbl_mode_en3_l_shift 0
#define tbl_mode_en3_l_mask ((1<<16)-1)

#define rfmux_mode_en4_h_tbl 0x40c
#define tbl_mode_en4_h_shift 0
#define tbl_mode_en4_h_mask ((1<<10)-1)

#define rfmux_mode_en4_m_tbl 0x40d
#define tbl_mode_en4_m_shift 0
#define tbl_mode_en4_m_mask ((1<<16)-1)

#define rfmux_mode_en4_l_tbl 0x40e
#define tbl_mode_en4_l_shift 0
#define tbl_mode_en4_l_mask ((1<<16)-1)

#define rfmux_mode_en5_h_tbl 0x40f
#define tbl_mode_en5_h_shift 0
#define tbl_mode_en5_h_mask ((1<<10)-1)

#define rfmux_mode_en5_m_tbl 0x410
#define tbl_mode_en5_m_shift 0
#define tbl_mode_en5_m_mask ((1<<16)-1)

#define rfmux_mode_en5_l_tbl 0x411
#define tbl_mode_en5_l_shift 0
#define tbl_mode_en5_l_mask ((1<<16)-1)

#define rfmux_mode_en6_h_tbl 0x412
#define tbl_mode_en6_h_shift 0
#define tbl_mode_en6_h_mask ((1<<10)-1)

#define rfmux_mode_en6_m_tbl 0x413
#define tbl_mode_en6_m_shift 0
#define tbl_mode_en6_m_mask ((1<<16)-1)

#define rfmux_mode_en6_l_tbl 0x414
#define tbl_mode_en6_l_shift 0
#define tbl_mode_en6_l_mask ((1<<16)-1)

#define rfmux_mode_en7_h_tbl 0x415
#define tbl_mode_en7_h_shift 0
#define tbl_mode_en7_h_mask ((1<<10)-1)

#define rfmux_mode_en7_m_tbl 0x416
#define tbl_mode_en7_m_shift 0
#define tbl_mode_en7_m_mask ((1<<16)-1)

#define rfmux_mode_en7_l_tbl 0x417
#define tbl_mode_en7_l_shift 0
#define tbl_mode_en7_l_mask ((1<<16)-1)

#define rfmux_rfcali_en_tbl_0_h 0x418
#define tbl_rfcali_en0_h_shift 0
#define tbl_rfcali_en0_h_mask ((1<<9)-1)

#define rfmux_rfcali_en_tbl_0_m 0x419
#define tbl_rfcali_en0_m_shift 0
#define tbl_rfcali_en0_m_mask ((1<<16)-1)

#define rfmux_rfcali_en_tbl_0_l 0x41a
#define tbl_rfcali_en0_l_shift 0
#define tbl_rfcali_en0_l_mask ((1<<16)-1)

#define rfmux_rfcali_en_tbl_1_h 0x41b
#define tbl_rfcali_en1_h_shift 0
#define tbl_rfcali_en1_h_mask ((1<<9)-1)

#define rfmux_rfcali_en_tbl_1_m 0x41c
#define tbl_rfcali_en1_m_shift 0
#define tbl_rfcali_en1_m_mask ((1<<16)-1)

#define rfmux_rfcali_en_tbl_1_l 0x41d
#define tbl_rfcali_en1_l_shift 0
#define tbl_rfcali_en1_l_mask ((1<<16)-1)

#define rfmux_rfcali_en_tbl_2_h 0x41e
#define tbl_rfcali_en2_h_shift 0
#define tbl_rfcali_en2_h_mask ((1<<9)-1)

#define rfmux_rfcali_en_tbl_2_m 0x41f
#define tbl_rfcali_en2_m_shift 0
#define tbl_rfcali_en2_m_mask ((1<<16)-1)

#define rfmux_rfcali_en_tbl_2_l 0x420
#define tbl_rfcali_en2_l_shift 0
#define tbl_rfcali_en2_l_mask ((1<<16)-1)

#define rfmux_rfcali_en_tbl_3_h 0x421
#define tbl_rfcali_en3_h_shift 0
#define tbl_rfcali_en3_h_mask ((1<<9)-1)

#define rfmux_rfcali_en_tbl_3_m 0x422
#define tbl_rfcali_en3_m_shift 0
#define tbl_rfcali_en3_m_mask ((1<<16)-1)

#define rfmux_rfcali_en_tbl_3_l 0x423
#define tbl_rfcali_en3_l_shift 0
#define tbl_rfcali_en3_l_mask ((1<<16)-1)

#define rfmux_rfcali_en_tbl_4_h 0x424
#define tbl_rfcali_en4_h_shift 0
#define tbl_rfcali_en4_h_mask ((1<<9)-1)

#define rfmux_rfcali_en_tbl_4_m 0x425
#define tbl_rfcali_en4_m_shift 0
#define tbl_rfcali_en4_m_mask ((1<<16)-1)

#define rfmux_rfcali_en_tbl_4_l 0x426
#define tbl_rfcali_en4_l_shift 0
#define tbl_rfcali_en4_l_mask ((1<<16)-1)

#define rfmux_rfcali_en_tbl_5_h 0x427
#define tbl_rfcali_en5_h_shift 0
#define tbl_rfcali_en5_h_mask ((1<<9)-1)

#define rfmux_rfcali_en_tbl_5_m 0x428
#define tbl_rfcali_en5_m_shift 0
#define tbl_rfcali_en5_m_mask ((1<<16)-1)

#define rfmux_rfcali_en_tbl_5_l 0x429
#define tbl_rfcali_en5_l_shift 0
#define tbl_rfcali_en5_l_mask ((1<<16)-1)

#define rfmux_rfcali_en_tbl_6_h 0x42a
#define tbl_rfcali_en6_h_shift 0
#define tbl_rfcali_en6_h_mask ((1<<9)-1)

#define rfmux_rfcali_en_tbl_6_m 0x42b
#define tbl_rfcali_en6_m_shift 0
#define tbl_rfcali_en6_m_mask ((1<<16)-1)

#define rfmux_rfcali_en_tbl_6_l 0x42c
#define tbl_rfcali_en6_l_shift 0
#define tbl_rfcali_en6_l_mask ((1<<16)-1)

#define rfmux_rfcali_en_tbl_7_h 0x42d
#define tbl_rfcali_en7_h_shift 0
#define tbl_rfcali_en7_h_mask ((1<<9)-1)

#define rfmux_rfcali_en_tbl_7_m 0x42e
#define tbl_rfcali_en7_m_shift 0
#define tbl_rfcali_en7_m_mask ((1<<16)-1)

#define rfmux_rfcali_en_tbl_7_l 0x42f
#define tbl_rfcali_en7_l_shift 0
#define tbl_rfcali_en7_l_mask ((1<<16)-1)

#define rfmux_wfrx_gidx_0_1_map_tbl 0x438
#define wfrx_gidx_0_1_map_shift 0
#define wfrx_gidx_0_1_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_2_3_map_tbl 0x439
#define wfrx_gidx_2_3_map_shift 0
#define wfrx_gidx_2_3_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_4_5_map_tbl 0x43a
#define wfrx_gidx_4_5_map_shift 0
#define wfrx_gidx_4_5_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_6_7_map_tbl 0x43b
#define wfrx_gidx_6_7_map_shift 0
#define wfrx_gidx_6_7_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_8_9_map_tbl 0x43c
#define wfrx_gidx_8_9_map_shift 0
#define wfrx_gidx_8_9_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_10_11_map_tbl 0x43d
#define wfrx_gidx_10_11_map_shift 0
#define wfrx_gidx_10_11_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_12_13_map_tbl 0x43e
#define wfrx_gidx_12_13_map_shift 0
#define wfrx_gidx_12_13_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_14_15_map_tbl 0x43f
#define wfrx_gidx_14_15_map_shift 0
#define wfrx_gidx_14_15_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_16_17_map_tbl 0x440
#define wfrx_gidx_16_17_map_shift 0
#define wfrx_gidx_16_17_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_18_19_map_tbl 0x441
#define wfrx_gidx_18_19_map_shift 0
#define wfrx_gidx_18_19_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_20_21_map_tbl 0x442
#define wfrx_gidx_20_21_map_shift 0
#define wfrx_gidx_20_21_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_22_23_map_tbl 0x443
#define wfrx_gidx_22_23_map_shift 0
#define wfrx_gidx_22_23_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_24_25_map_tbl 0x444
#define wfrx_gidx_24_25_map_shift 0
#define wfrx_gidx_24_25_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_26_27_map_tbl 0x445
#define wfrx_gidx_26_27_map_shift 0
#define wfrx_gidx_26_27_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_28_29_map_tbl 0x446
#define wfrx_gidx_28_29_map_shift 0
#define wfrx_gidx_28_29_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_30_31_map_tbl 0x447
#define wfrx_gidx_30_31_map_shift 0
#define wfrx_gidx_30_31_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_32_33_map_tbl 0x448
#define wfrx_gidx_32_33_map_shift 0
#define wfrx_gidx_32_33_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_34_35_map_tbl 0x449
#define wfrx_gidx_34_35_map_shift 0
#define wfrx_gidx_34_35_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_36_37_map_tbl 0x44a
#define wfrx_gidx_36_37_map_shift 0
#define wfrx_gidx_36_37_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_38_39_map_tbl 0x44b
#define wfrx_gidx_38_39_map_shift 0
#define wfrx_gidx_38_39_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_40_41_map_tbl 0x44c
#define wfrx_gidx_40_41_map_shift 0
#define wfrx_gidx_40_41_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_42_43_map_tbl 0x44d
#define wfrx_gidx_42_43_map_shift 0
#define wfrx_gidx_42_43_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_44_45_map_tbl 0x44e
#define wfrx_gidx_44_45_map_shift 0
#define wfrx_gidx_44_45_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_46_47_map_tbl 0x44f
#define wfrx_gidx_46_47_map_shift 0
#define wfrx_gidx_46_47_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_48_49_map_tbl 0x450
#define wfrx_gidx_48_49_map_shift 0
#define wfrx_gidx_48_49_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_50_51_map_tbl 0x451
#define wfrx_gidx_50_51_map_shift 0
#define wfrx_gidx_50_51_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_52_53_map_tbl 0x452
#define wfrx_gidx_52_53_map_shift 0
#define wfrx_gidx_52_53_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_54_55_map_tbl 0x453
#define wfrx_gidx_54_55_map_shift 0
#define wfrx_gidx_54_55_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_56_57_map_tbl 0x454
#define wfrx_gidx_56_57_map_shift 0
#define wfrx_gidx_56_57_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_58_59_map_tbl 0x455
#define wfrx_gidx_58_59_map_shift 0
#define wfrx_gidx_58_59_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_60_61_map_tbl 0x456
#define wfrx_gidx_60_61_map_shift 0
#define wfrx_gidx_60_61_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_62_63_map_tbl 0x457
#define wfrx_gidx_62_63_map_shift 0
#define wfrx_gidx_62_63_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_64_65_map_tbl 0x458
#define wfrx_gidx_64_65_map_shift 0
#define wfrx_gidx_64_65_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_66_67_map_tbl 0x459
#define wfrx_gidx_66_67_map_shift 0
#define wfrx_gidx_66_67_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_68_69_map_tbl 0x45a
#define wfrx_gidx_68_69_map_shift 0
#define wfrx_gidx_68_69_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_70_71_map_tbl 0x45b
#define wfrx_gidx_70_71_map_shift 0
#define wfrx_gidx_70_71_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_72_73_map_tbl 0x45c
#define wfrx_gidx_72_73_map_shift 0
#define wfrx_gidx_72_73_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_74_75_map_tbl 0x45d
#define wfrx_gidx_74_75_map_shift 0
#define wfrx_gidx_74_75_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_76_77_map_tbl 0x45e
#define wfrx_gidx_76_77_map_shift 0
#define wfrx_gidx_76_77_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_78_79_map_tbl 0x45f
#define wfrx_gidx_78_79_map_shift 0
#define wfrx_gidx_78_79_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_80_81_map_tbl 0x460
#define wfrx_gidx_80_81_map_shift 0
#define wfrx_gidx_80_81_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_82_83_map_tbl 0x461
#define wfrx_gidx_82_83_map_shift 0
#define wfrx_gidx_82_83_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_84_85_map_tbl 0x462
#define wfrx_gidx_84_85_map_shift 0
#define wfrx_gidx_84_85_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_86_87_map_tbl 0x463
#define wfrx_gidx_86_87_map_shift 0
#define wfrx_gidx_86_87_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_88_89_map_tbl 0x464
#define wfrx_gidx_88_89_map_shift 0
#define wfrx_gidx_88_89_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_90_91_map_tbl 0x465
#define wfrx_gidx_90_91_map_shift 0
#define wfrx_gidx_90_91_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_92_93_map_tbl 0x466
#define wfrx_gidx_92_93_map_shift 0
#define wfrx_gidx_92_93_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_94_95_map_tbl 0x467
#define wfrx_gidx_94_95_map_shift 0
#define wfrx_gidx_94_95_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_96_97_map_tbl 0x468
#define wfrx_gidx_96_97_map_shift 0
#define wfrx_gidx_96_97_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_98_99_map_tbl 0x469
#define wfrx_gidx_98_99_map_shift 0
#define wfrx_gidx_98_99_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_100_101_map_tbl 0x46a
#define wfrx_gidx_100_101_map_shift 0
#define wfrx_gidx_100_101_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_102_103_map_tbl 0x46b
#define wfrx_gidx_102_103_map_shift 0
#define wfrx_gidx_102_103_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_104_105_map_tbl 0x46c
#define wfrx_gidx_104_105_map_shift 0
#define wfrx_gidx_104_105_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_106_107_map_tbl 0x46d
#define wfrx_gidx_106_107_map_shift 0
#define wfrx_gidx_106_107_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_108_109_map_tbl 0x46e
#define wfrx_gidx_108_109_map_shift 0
#define wfrx_gidx_108_109_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_110_111_map_tbl 0x46f
#define wfrx_gidx_110_111_map_shift 0
#define wfrx_gidx_110_111_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_112_113_map_tbl 0x470
#define wfrx_gidx_112_113_map_shift 0
#define wfrx_gidx_112_113_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_114_115_map_tbl 0x471
#define wfrx_gidx_114_115_map_shift 0
#define wfrx_gidx_114_115_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_116_117_map_tbl 0x472
#define wfrx_gidx_116_117_map_shift 0
#define wfrx_gidx_116_117_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_118_119_map_tbl 0x473
#define wfrx_gidx_118_119_map_shift 0
#define wfrx_gidx_118_119_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_120_121_map_tbl 0x474
#define wfrx_gidx_120_121_map_shift 0
#define wfrx_gidx_120_121_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_122_123_map_tbl 0x475
#define wfrx_gidx_122_123_map_shift 0
#define wfrx_gidx_122_123_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_124_125_map_tbl 0x476
#define wfrx_gidx_124_125_map_shift 0
#define wfrx_gidx_124_125_map_mask ((1<<15)-1)

#define rfmux_wfrx_gidx_126_127_map_tbl 0x477
#define wfrx_gidx_126_127_map_shift 0
#define wfrx_gidx_126_127_map_mask ((1<<15)-1)

#define rfmux_tbl_lna_btrx_0_high_reg 0x478
#define tbl_lna_btrx_0_h_shift 0
#define tbl_lna_btrx_0_h_mask ((1<<16)-1)

#define rfmux_tbl_lna_btrx_0_low_reg 0x479
#define tbl_lna_btrx_0_l_shift 0
#define tbl_lna_btrx_0_l_mask ((1<<4)-1)

#define rfmux_tbl_lna_btrx_1_high_reg 0x47a
#define tbl_lna_btrx_1_h_shift 0
#define tbl_lna_btrx_1_h_mask ((1<<16)-1)

#define rfmux_tbl_lna_btrx_1_low_reg 0x47b
#define tbl_lna_btrx_1_l_shift 0
#define tbl_lna_btrx_1_l_mask ((1<<4)-1)

#define rfmux_tbl_lna_btrx_2_high_reg 0x47c
#define tbl_lna_btrx_2_h_shift 0
#define tbl_lna_btrx_2_h_mask ((1<<16)-1)

#define rfmux_tbl_lna_btrx_2_low_reg 0x47d
#define tbl_lna_btrx_2_l_shift 0
#define tbl_lna_btrx_2_l_mask ((1<<4)-1)

#define rfmux_tbl_lna_btrx_3_high_reg 0x47e
#define tbl_lna_btrx_3_h_shift 0
#define tbl_lna_btrx_3_h_mask ((1<<16)-1)

#define rfmux_tbl_lna_btrx_3_low_reg 0x47f
#define tbl_lna_btrx_3_l_shift 0
#define tbl_lna_btrx_3_l_mask ((1<<4)-1)

#define rfmux_tbl_lna_wfrx_0_high_reg 0x480
#define tbl_lna_wfrx_0_h_shift 0
#define tbl_lna_wfrx_0_h_mask ((1<<16)-1)

#define rfmux_tbl_lna_wfrx_0_low_reg 0x481
#define tbl_lna_wfrx_0_l_shift 0
#define tbl_lna_wfrx_0_l_mask ((1<<4)-1)

#define rfmux_tbl_lna_wfrx_1_high_reg 0x482
#define tbl_lna_wfrx_1_h_shift 0
#define tbl_lna_wfrx_1_h_mask ((1<<16)-1)

#define rfmux_tbl_lna_wfrx_1_low_reg 0x483
#define tbl_lna_wfrx_1_l_shift 0
#define tbl_lna_wfrx_1_l_mask ((1<<4)-1)

#define rfmux_tbl_lna_wfrx_2_high_reg 0x484
#define tbl_lna_wfrx_2_h_shift 0
#define tbl_lna_wfrx_2_h_mask ((1<<16)-1)

#define rfmux_tbl_lna_wfrx_2_low_reg 0x485
#define tbl_lna_wfrx_2_l_shift 0
#define tbl_lna_wfrx_2_l_mask ((1<<4)-1)

#define rfmux_tbl_lna_wfrx_3_high_reg 0x486
#define tbl_lna_wfrx_3_h_shift 0
#define tbl_lna_wfrx_3_h_mask ((1<<16)-1)

#define rfmux_tbl_lna_wfrx_3_low_reg 0x487
#define tbl_lna_wfrx_3_l_shift 0
#define tbl_lna_wfrx_3_l_mask ((1<<4)-1)

#define rfmux_tx_pa_bth_h_cfg 0x488
#define tbl_pa_bth_h_shift 0
#define tbl_pa_bth_h_mask ((1<<16)-1)

#define rfmux_tx_pa_bth_l_cfg 0x489
#define tbl_pa_bth_l_shift 0
#define tbl_pa_bth_l_mask ((1<<16)-1)

#define rfmux_tx_pa_btm_h_cfg 0x48a
#define tbl_pa_btm_h_shift 0
#define tbl_pa_btm_h_mask ((1<<16)-1)

#define rfmux_tx_pa_btm_l_cfg 0x48b
#define tbl_pa_btm_l_shift 0
#define tbl_pa_btm_l_mask ((1<<16)-1)

#define rfmux_tx_pa_btl_h_cfg 0x48c
#define tbl_pa_btl_h_shift 0
#define tbl_pa_btl_h_mask ((1<<16)-1)

#define rfmux_tx_pa_btl_l_cfg 0x48d
#define tbl_pa_btl_l_shift 0
#define tbl_pa_btl_l_mask ((1<<16)-1)

#define rfmux_tx_pad_bth_h_cfg 0x48e
#define tbl_pad_bth_h_shift 0
#define tbl_pad_bth_h_mask ((1<<16)-1)

#define rfmux_tx_pad_bth_l_cfg 0x48f
#define tbl_pad_bth_l_shift 0
#define tbl_pad_bth_l_mask ((1<<16)-1)

#define rfmux_tx_pad_btm_h_cfg 0x490
#define tbl_pad_btm_h_shift 0
#define tbl_pad_btm_h_mask ((1<<16)-1)

#define rfmux_tx_pad_btm_l_cfg 0x491
#define tbl_pad_btm_l_shift 0
#define tbl_pad_btm_l_mask ((1<<16)-1)

#define rfmux_tx_pad_btl_h_cfg 0x492
#define tbl_pad_btl_h_shift 0
#define tbl_pad_btl_h_mask ((1<<16)-1)

#define rfmux_tx_pad_btl_l_cfg 0x493
#define tbl_pad_btl_l_shift 0
#define tbl_pad_btl_l_mask ((1<<16)-1)

#define rfmux_tbl_pa_band0_wfbh_h_reg 0x494
#define tbl_pa_band0_wfbh_h_shift 0
#define tbl_pa_band0_wfbh_h_mask ((1<<16)-1)

#define rfmux_tbl_pa_band0_wfbh_l_reg 0x495
#define tbl_pa_band0_wfbh_l_shift 0
#define tbl_pa_band0_wfbh_l_mask ((1<<16)-1)

#define rfmux_tbl_pa_band0_wfbl_h_reg 0x496
#define tbl_pa_band0_wfbl_h_shift 0
#define tbl_pa_band0_wfbl_h_mask ((1<<16)-1)

#define rfmux_tbl_pa_band0_wfbl_l_reg 0x497
#define tbl_pa_band0_wfbl_l_shift 0
#define tbl_pa_band0_wfbl_l_mask ((1<<16)-1)

#define rfmux_tbl_pa_band0_wfgh_h_reg 0x498
#define tbl_pa_band0_wfgh_h_shift 0
#define tbl_pa_band0_wfgh_h_mask ((1<<16)-1)

#define rfmux_tbl_pa_band0_wfgh_l_reg 0x499
#define tbl_pa_band0_wfgh_l_shift 0
#define tbl_pa_band0_wfgh_l_mask ((1<<16)-1)

#define rfmux_tbl_pa_band0_wfgl_h_reg 0x49a
#define tbl_pa_band0_wfgl_h_shift 0
#define tbl_pa_band0_wfgl_h_mask ((1<<16)-1)

#define rfmux_tbl_pa_band0_wfgl_l_reg 0x49b
#define tbl_pa_band0_wfgl_l_shift 0
#define tbl_pa_band0_wfgl_l_mask ((1<<16)-1)

#define rfmux_tbl_pa_band1_wfbh_h_reg 0x49c
#define tbl_pa_band1_wfbh_h_shift 0
#define tbl_pa_band1_wfbh_h_mask ((1<<16)-1)

#define rfmux_tbl_pa_band1_wfbh_l_reg 0x49d
#define tbl_pa_band1_wfbh_l_shift 0
#define tbl_pa_band1_wfbh_l_mask ((1<<16)-1)

#define rfmux_tbl_pa_band1_wfbl_h_reg 0x49e
#define tbl_pa_band1_wfbl_h_shift 0
#define tbl_pa_band1_wfbl_h_mask ((1<<16)-1)

#define rfmux_tbl_pa_band1_wfbl_l_reg 0x49f
#define tbl_pa_band1_wfbl_l_shift 0
#define tbl_pa_band1_wfbl_l_mask ((1<<16)-1)

#define rfmux_tbl_pa_band1_wfgh_h_reg 0x4a0
#define tbl_pa_band1_wfgh_h_shift 0
#define tbl_pa_band1_wfgh_h_mask ((1<<16)-1)

#define rfmux_tbl_pa_band1_wfgh_l_reg 0x4a1
#define tbl_pa_band1_wfgh_l_shift 0
#define tbl_pa_band1_wfgh_l_mask ((1<<16)-1)

#define rfmux_tbl_pa_band1_wfgl_h_reg 0x4a2
#define tbl_pa_band1_wfgl_h_shift 0
#define tbl_pa_band1_wfgl_h_mask ((1<<16)-1)

#define rfmux_tbl_pa_band1_wfgl_l_reg 0x4a3
#define tbl_pa_band1_wfgl_l_shift 0
#define tbl_pa_band1_wfgl_l_mask ((1<<16)-1)

#define rfmux_tbl_pa_band2_wfbh_h_reg 0x4a4
#define tbl_pa_band2_wfbh_h_shift 0
#define tbl_pa_band2_wfbh_h_mask ((1<<16)-1)

#define rfmux_tbl_pa_band2_wfbh_l_reg 0x4a5
#define tbl_pa_band2_wfbh_l_shift 0
#define tbl_pa_band2_wfbh_l_mask ((1<<16)-1)

#define rfmux_tbl_pa_band2_wfbl_h_reg 0x4a6
#define tbl_pa_band2_wfbl_h_shift 0
#define tbl_pa_band2_wfbl_h_mask ((1<<16)-1)

#define rfmux_tbl_pa_band2_wfbl_l_reg 0x4a7
#define tbl_pa_band2_wfbl_l_shift 0
#define tbl_pa_band2_wfbl_l_mask ((1<<16)-1)

#define rfmux_tbl_pa_band2_wfgh_h_reg 0x4a8
#define tbl_pa_band2_wfgh_h_shift 0
#define tbl_pa_band2_wfgh_h_mask ((1<<16)-1)

#define rfmux_tbl_pa_band2_wfgh_l_reg 0x4a9
#define tbl_pa_band2_wfgh_l_shift 0
#define tbl_pa_band2_wfgh_l_mask ((1<<16)-1)

#define rfmux_tbl_pa_band2_wfgl_h_reg 0x4aa
#define tbl_pa_band2_wfgl_h_shift 0
#define tbl_pa_band2_wfgl_h_mask ((1<<16)-1)

#define rfmux_tbl_pa_band2_wfgl_l_reg 0x4ab
#define tbl_pa_band2_wfgl_l_shift 0
#define tbl_pa_band2_wfgl_l_mask ((1<<16)-1)

#define rfmux_tbl_pa_band3_wfbh_h_reg 0x4ac
#define tbl_pa_band3_wfbh_h_shift 0
#define tbl_pa_band3_wfbh_h_mask ((1<<16)-1)

#define rfmux_tbl_pa_band3_wfbh_l_reg 0x4ad
#define tbl_pa_band3_wfbh_l_shift 0
#define tbl_pa_band3_wfbh_l_mask ((1<<16)-1)

#define rfmux_tbl_pa_band3_wfbl_h_reg 0x4ae
#define tbl_pa_band3_wfbl_h_shift 0
#define tbl_pa_band3_wfbl_h_mask ((1<<16)-1)

#define rfmux_tbl_pa_band3_wfbl_l_reg 0x4af
#define tbl_pa_band3_wfbl_l_shift 0
#define tbl_pa_band3_wfbl_l_mask ((1<<16)-1)

#define rfmux_tbl_pa_band3_wfgh_h_reg 0x4b0
#define tbl_pa_band3_wfgh_h_shift 0
#define tbl_pa_band3_wfgh_h_mask ((1<<16)-1)

#define rfmux_tbl_pa_band3_wfgh_l_reg 0x4b1
#define tbl_pa_band3_wfgh_l_shift 0
#define tbl_pa_band3_wfgh_l_mask ((1<<16)-1)

#define rfmux_tbl_pa_band3_wfgl_h_reg 0x4b2
#define tbl_pa_band3_wfgl_h_shift 0
#define tbl_pa_band3_wfgl_h_mask ((1<<16)-1)

#define rfmux_tbl_pa_band3_wfgl_l_reg 0x4b3
#define tbl_pa_band3_wfgl_l_shift 0
#define tbl_pa_band3_wfgl_l_mask ((1<<16)-1)

#define rfmux_tbl_pad_band0_wfbh_h_reg 0x4b4
#define tbl_pad_band0_wfbh_h_shift 0
#define tbl_pad_band0_wfbh_h_mask ((1<<16)-1)

#define rfmux_tbl_pad_band0_wfbh_l_reg 0x4b5
#define tbl_pad_band0_wfbh_l_shift 0
#define tbl_pad_band0_wfbh_l_mask ((1<<16)-1)

#define rfmux_tbl_pad_band0_wfbl_h_reg 0x4b6
#define tbl_pad_band0_wfbl_h_shift 0
#define tbl_pad_band0_wfbl_h_mask ((1<<16)-1)

#define rfmux_tbl_pad_band0_wfbl_l_reg 0x4b7
#define tbl_pad_band0_wfbl_l_shift 0
#define tbl_pad_band0_wfbl_l_mask ((1<<16)-1)

#define rfmux_tbl_pad_band0_wfgh_h_reg 0x4b8
#define tbl_pad_band0_wfgh_h_shift 0
#define tbl_pad_band0_wfgh_h_mask ((1<<16)-1)

#define rfmux_tbl_pad_band0_wfgh_l_reg 0x4b9
#define tbl_pad_band0_wfgh_l_shift 0
#define tbl_pad_band0_wfgh_l_mask ((1<<16)-1)

#define rfmux_tbl_pad_band0_wfgl_h_reg 0x4ba
#define tbl_pad_band0_wfgl_h_shift 0
#define tbl_pad_band0_wfgl_h_mask ((1<<16)-1)

#define rfmux_tbl_pad_band0_wfgl_l_reg 0x4bb
#define tbl_pad_band0_wfgl_l_shift 0
#define tbl_pad_band0_wfgl_l_mask ((1<<16)-1)

#define rfmux_tbl_pad_band1_wfbh_h_reg 0x4bc
#define tbl_pad_band1_wfbh_h_shift 0
#define tbl_pad_band1_wfbh_h_mask ((1<<16)-1)

#define rfmux_tbl_pad_band1_wfbh_l_reg 0x4bd
#define tbl_pad_band1_wfbh_l_shift 0
#define tbl_pad_band1_wfbh_l_mask ((1<<16)-1)

#define rfmux_tbl_pad_band1_wfbl_h_reg 0x4be
#define tbl_pad_band1_wfbl_h_shift 0
#define tbl_pad_band1_wfbl_h_mask ((1<<16)-1)

#define rfmux_tbl_pad_band1_wfbl_l_reg 0x4bf
#define tbl_pad_band1_wfbl_l_shift 0
#define tbl_pad_band1_wfbl_l_mask ((1<<16)-1)

#define rfmux_tbl_pad_band1_wfgh_h_reg 0x4c0
#define tbl_pad_band1_wfgh_h_shift 0
#define tbl_pad_band1_wfgh_h_mask ((1<<16)-1)

#define rfmux_tbl_pad_band1_wfgh_l_reg 0x4c1
#define tbl_pad_band1_wfgh_l_shift 0
#define tbl_pad_band1_wfgh_l_mask ((1<<16)-1)

#define rfmux_tbl_pad_band1_wfgl_h_reg 0x4c2
#define tbl_pad_band1_wfgl_h_shift 0
#define tbl_pad_band1_wfgl_h_mask ((1<<16)-1)

#define rfmux_tbl_pad_band1_wfgl_l_reg 0x4c3
#define tbl_pad_band1_wfgl_l_shift 0
#define tbl_pad_band1_wfgl_l_mask ((1<<16)-1)

#define rfmux_tbl_pad_band2_wfbh_h_reg 0x4c4
#define tbl_pad_band2_wfbh_h_shift 0
#define tbl_pad_band2_wfbh_h_mask ((1<<16)-1)

#define rfmux_tbl_pad_band2_wfbh_l_reg 0x4c5
#define tbl_pad_band2_wfbh_l_shift 0
#define tbl_pad_band2_wfbh_l_mask ((1<<16)-1)

#define rfmux_tbl_pad_band2_wfbl_h_reg 0x4c6
#define tbl_pad_band2_wfbl_h_shift 0
#define tbl_pad_band2_wfbl_h_mask ((1<<16)-1)

#define rfmux_tbl_pad_band2_wfbl_l_reg 0x4c7
#define tbl_pad_band2_wfbl_l_shift 0
#define tbl_pad_band2_wfbl_l_mask ((1<<16)-1)

#define rfmux_tbl_pad_band2_wfgh_h_reg 0x4c8
#define tbl_pad_band2_wfgh_h_shift 0
#define tbl_pad_band2_wfgh_h_mask ((1<<16)-1)

#define rfmux_tbl_pad_band2_wfgh_l_reg 0x4c9
#define tbl_pad_band2_wfgh_l_shift 0
#define tbl_pad_band2_wfgh_l_mask ((1<<16)-1)

#define rfmux_tbl_pad_band2_wfgl_h_reg 0x4ca
#define tbl_pad_band2_wfgl_h_shift 0
#define tbl_pad_band2_wfgl_h_mask ((1<<16)-1)

#define rfmux_tbl_pad_band2_wfgl_l_reg 0x4cb
#define tbl_pad_band2_wfgl_l_shift 0
#define tbl_pad_band2_wfgl_l_mask ((1<<16)-1)

#define rfmux_tbl_pad_band3_wfbh_h_reg 0x4cc
#define tbl_pad_band3_wfbh_h_shift 0
#define tbl_pad_band3_wfbh_h_mask ((1<<16)-1)

#define rfmux_tbl_pad_band3_wfbh_l_reg 0x4cd
#define tbl_pad_band3_wfbh_l_shift 0
#define tbl_pad_band3_wfbh_l_mask ((1<<16)-1)

#define rfmux_tbl_pad_band3_wfbl_h_reg 0x4ce
#define tbl_pad_band3_wfbl_h_shift 0
#define tbl_pad_band3_wfbl_h_mask ((1<<16)-1)

#define rfmux_tbl_pad_band3_wfbl_l_reg 0x4cf
#define tbl_pad_band3_wfbl_l_shift 0
#define tbl_pad_band3_wfbl_l_mask ((1<<16)-1)

#define rfmux_tbl_pad_band3_wfgh_h_reg 0x4d0
#define tbl_pad_band3_wfgh_h_shift 0
#define tbl_pad_band3_wfgh_h_mask ((1<<16)-1)

#define rfmux_tbl_pad_band3_wfgh_l_reg 0x4d1
#define tbl_pad_band3_wfgh_l_shift 0
#define tbl_pad_band3_wfgh_l_mask ((1<<16)-1)

#define rfmux_tbl_pad_band3_wfgl_h_reg 0x4d2
#define tbl_pad_band3_wfgl_h_shift 0
#define tbl_pad_band3_wfgl_h_mask ((1<<16)-1)

#define rfmux_tbl_pad_band3_wfgl_l_reg 0x4d3
#define tbl_pad_band3_wfgl_l_shift 0
#define tbl_pad_band3_wfgl_l_mask ((1<<16)-1)

#define rfmux_tbl_bttx_loft_0_cw_reg 0x4d4
#define tbl_bttx_loft_0_cw_shift 0
#define tbl_bttx_loft_0_cw_mask ((1<<16)-1)

#define rfmux_tbl_bttx_loft_1_cw_reg 0x4d5
#define tbl_bttx_loft_1_cw_shift 0
#define tbl_bttx_loft_1_cw_mask ((1<<16)-1)

#define rfmux_tbl_bttx_loft_2_cw_reg 0x4d6
#define tbl_bttx_loft_2_cw_shift 0
#define tbl_bttx_loft_2_cw_mask ((1<<16)-1)

#define rfmux_tbl_bttx_loft_3_cw_reg 0x4d7
#define tbl_bttx_loft_3_cw_shift 0
#define tbl_bttx_loft_3_cw_mask ((1<<16)-1)

#define rfmux_tbl_wftx_loft_0_cw_reg 0x4d8
#define tbl_wftx_loft_0_cw_shift 0
#define tbl_wftx_loft_0_cw_mask ((1<<16)-1)

#define rfmux_tbl_wftx_loft_1_cw_reg 0x4d9
#define tbl_wftx_loft_1_cw_shift 0
#define tbl_wftx_loft_1_cw_mask ((1<<16)-1)

#define rfmux_tbl_wftx_loft_2_cw_reg 0x4da
#define tbl_wftx_loft_2_cw_shift 0
#define tbl_wftx_loft_2_cw_mask ((1<<16)-1)

#define rfmux_tbl_wftx_loft_3_cw_reg 0x4db
#define tbl_wftx_loft_3_cw_shift 0
#define tbl_wftx_loft_3_cw_mask ((1<<16)-1)

#define rfmux_tbl_pout_negm_0_entry_reg 0x4dc
#define tbl_pout_negm_0_entry_shift 0
#define tbl_pout_negm_0_entry_mask ((1<<16)-1)

#define rfmux_tbl_pout_negm_1_entry_reg 0x4dd
#define tbl_pout_negm_1_entry_shift 0
#define tbl_pout_negm_1_entry_mask ((1<<16)-1)

#define rfmux_tbl_pout_band0_gain0_entry_reg 0x4de
#define tbl_pout_band0_gain0_entry_shift 0
#define tbl_pout_band0_gain0_entry_mask ((1<<15)-1)

#define rfmux_tbl_pout_band0_gain1_entry_reg 0x4df
#define tbl_pout_band0_gain1_entry_shift 0
#define tbl_pout_band0_gain1_entry_mask ((1<<15)-1)

#define rfmux_tbl_pout_band0_gain2_entry_reg 0x4e0
#define tbl_pout_band0_gain2_entry_shift 0
#define tbl_pout_band0_gain2_entry_mask ((1<<15)-1)

#define rfmux_tbl_pout_band0_gain3_entry_reg 0x4e1
#define tbl_pout_band0_gain3_entry_shift 0
#define tbl_pout_band0_gain3_entry_mask ((1<<15)-1)

#define rfmux_tbl_pout_band1_gain0_entry_reg 0x4e2
#define tbl_pout_band1_gain0_entry_shift 0
#define tbl_pout_band1_gain0_entry_mask ((1<<15)-1)

#define rfmux_tbl_pout_band1_gain1_entry_reg 0x4e3
#define tbl_pout_band1_gain1_entry_shift 0
#define tbl_pout_band1_gain1_entry_mask ((1<<15)-1)

#define rfmux_tbl_pout_band1_gain2_entry_reg 0x4e4
#define tbl_pout_band1_gain2_entry_shift 0
#define tbl_pout_band1_gain2_entry_mask ((1<<15)-1)

#define rfmux_tbl_pout_band1_gain3_entry_reg 0x4e5
#define tbl_pout_band1_gain3_entry_shift 0
#define tbl_pout_band1_gain3_entry_mask ((1<<15)-1)

#define rfmux_tbl_pout_band2_gain0_entry_reg 0x4e6
#define tbl_pout_band2_gain0_entry_shift 0
#define tbl_pout_band2_gain0_entry_mask ((1<<15)-1)

#define rfmux_tbl_pout_band2_gain1_entry_reg 0x4e7
#define tbl_pout_band2_gain1_entry_shift 0
#define tbl_pout_band2_gain1_entry_mask ((1<<15)-1)

#define rfmux_tbl_pout_band2_gain2_entry_reg 0x4e8
#define tbl_pout_band2_gain2_entry_shift 0
#define tbl_pout_band2_gain2_entry_mask ((1<<15)-1)

#define rfmux_tbl_pout_band2_gain3_entry_reg 0x4e9
#define tbl_pout_band2_gain3_entry_shift 0
#define tbl_pout_band2_gain3_entry_mask ((1<<15)-1)

#define rfmux_tbl_pout_band3_gain0_entry_reg 0x4ea
#define tbl_pout_band3_gain0_entry_shift 0
#define tbl_pout_band3_gain0_entry_mask ((1<<15)-1)

#define rfmux_tbl_pout_band3_gain1_entry_reg 0x4eb
#define tbl_pout_band3_gain1_entry_shift 0
#define tbl_pout_band3_gain1_entry_mask ((1<<15)-1)

#define rfmux_tbl_pout_band3_gain2_entry_reg 0x4ec
#define tbl_pout_band3_gain2_entry_shift 0
#define tbl_pout_band3_gain2_entry_mask ((1<<15)-1)

#define rfmux_tbl_pout_band3_gain3_entry_reg_reg 0x4ed
#define tbl_pout_band3_gain3_entry_shift 0
#define tbl_pout_band3_gain3_entry_mask ((1<<15)-1)


#define  REG_BT_DCOC_PGA_IQ_CTRL     0x216
#define  REG_BT_DCOC_TIA_IQ_CTRL     0x217
#define  REG_WIFI_DCOC_PGA_IQ_CTRL     0x218
#define  REG_WIFI_DCOC_TIA_IQ_CTRL     0x219
#define  REG_FM_DCOC_IQ_CTRL   0x21c

#define  BIT_I_H  15
#define  BIT_Q_H  7
#define REG_RFCALI_FM_DCOC_QDAT         0x07c
#define REG_RFCALI_FM_DCOC_IDAT       0x07b

#define BT_RX_GAIN_TBL_START_ADDR   0x640
//regAddr(btRxIdx)= BT_RX_GAIN_TBL_START_ADDR + btRxIdx*4
#define WIFI_RX_GAIN_TBL_START_ADDR 0x6c0
// regAddr(wifiRxIdx)= WIFI_RX_GAIN_TBL_START_ADDR + wifiTxIdx*4
#define FM_RX_GAIN_TBL_START_ADDR   0x600
#define WIFI_TX_GAIN_TBL_START_ADDR  0x760
// regAddr(wifiTxIdx)= WIFI_TX_GAIN_TBL_START_ADDR + wifiTxIdx*2
#define BT_TX_GAIN_TBL_START_ADDR       0x740
// regAddr(btTxIdx)= BT_TX_GAIN_TBL_START_ADDR + btTxIdx
#if 0
#define LOFT_DAC_I_RNG 30
#define LOFT_DAC_Q_RNG 12
#else
#define LOFT_DAC_I_RNG 30
#define LOFT_DAC_Q_RNG 16
#endif
//MXD_U32 gDcoc31Ch16=0x8080,gDcoc31Ch17=0x8080,gDcoc30Ch16=0x8080,gDcoc30Ch17=0x8080;
//MXD_U32 ArrayDcoc30[18]={0};
//MXD_U32 ArrayDcoc31[18]={0};
//void GetDcoc30Array(MXD_U32 tiaCh16,MXD_U32 tiaCh17);
//void GetDcoc31Array(MXD_U32 tiaCh16,MXD_U32 tiaCh17);
MXD_U32 gWfChDcoc[]=
{
    // freqMHz, dc28, dc29   dcocIdx30,  dcocIdx31
    /*1  */ 2412, 0x8080, 0x8080, 0x8080, 0x8080,
    /* 2 */ 2417, 0x8080, 0x8080, 0x8080, 0x8080,
    /* 3 */ 2422, 0x8080, 0x8080, 0x8080, 0x8080,
    /* 4 */ 2427, 0x8080, 0x8080, 0x8080, 0x8080,
    /* 5 */ 2432, 0x8080, 0x8080, 0x8080, 0x8080,
    /* 6 */ 2437, 0x8080, 0x8080, 0x8080, 0x8080,
    /* 7 */ 2442, 0x8080, 0x8080, 0x8080, 0x8080,
    /* 8 */ 2447, 0x8080, 0x8080, 0x8080, 0x8080,
    /* 9 */ 2452, 0x8080, 0x8080, 0x8080, 0x8080,
    /* 10 */2457, 0x8080, 0x8080, 0x8080, 0x8080,
    /* 11 */2462, 0x8080, 0x8080, 0x8080, 0x8080,
    /* 12 */2467, 0x8080, 0x8080, 0x8080, 0x8080,
    /* 13 */2472, 0x8080, 0x8080, 0x8080, 0x8080,
    /* 14 */2484, 0x8080, 0x8080, 0x8080, 0x8080,
    /* 15 */2352, 0x8080, 0x8080, 0x8080, 0x8080, //add 0409 //15
    /* 16 */2380, 0x8080, 0x8080, 0x8080, 0x8080,
    /* 17 */2508, 0x8080, 0x8080, 0x8080, 0x8080,
};
//   sdm_syn_divn_cfg5_reg0x60,sdm_syn_divn_cfg6_reg0x61,afc_fvco_wifi_config0x69
MXD_U16 gFreqWifiCh[]=
{
    //0x201[10:9] = 2b'3
    //0x21f[2] = 2b'1
    //FreqMHz  reg60 reg61 reg_69_3
    /* 1  2412*/   0x01EE,0xC4EC,0x1EEC,
    /* 2  2417*/   0x01EF,0xCB7C,0x1EFC,
    /* 3  2422*/   0x01F0,0xD20D,0x1F0D,
    /* 4  2427*/   0x01F1,0xD89D,0x1F1D,
    /* 5  2432*/   0x01F2,0xDF2D,0x1F2D,
    /* 6  2437*/   0x01F3,0xE5BE,0x1F3E,
    /* 7  2442*/   0x01F4,0xEC4E,0x1F4E,
    /* 8  2447*/   0x01F5,0xF2DF,0x1F5F,
    /* 9  2452*/   0x01F6,0xF96F,0x1F6F,
    /* 10  2457*/   0x01F8,0x0000,0x1F80,
    /* 11  2462*/   0x01F9,0x0690,0x1F90,
    /* 12  2467*/   0x01FA,0x0D20,0x1FA0,
    /* 13  2472*/   0x01FB,0x13B1,0x1FB1,

    ///*2477*/   0x01FC,0x1A41,0x1FC1,      0331
    /* 14  2484*/   0x01FD,0x89D8,0x1FD8,

    /* 15  2352*/ 0x01e2,0x7627,0x1e27,  //add 0409 //15
    /* 16  2380*/ 0x01E8,0x3483,0x1E83,
    /* 17  2508*/ 0x0202,0x7627,0x2027,

};


//   sdm_syn_divn_cfg3_reg0x5e,sdm_syn_divn_cfg4_reg0x5f,afc_fvco_bt_config0x68
//   DIVN_MSB_10Bit,DIVN_LSB_16Bit,fvco_target_bt
MXD_U16 gFreqMhzBt[]=
{
    //0x201[8:7] = 2b'3
    //0x21f[1] = 2b'1
    //Freq  reg5e reg5f reg68_1
    /*2400*/    0x01EC,    0x4EC0,    0x07B1,
    /*2401*/    0x01EC,    0x8344,    0x07B2,//
    /*2402*/    0x01EC,    0xB7CB,    0x07B2,
    /*2403*/    0x01EC,    0xEC4E,    0x07B3,
    /*2404*/    0x01ED,    0x20D2,    0x07B4,
    /*2405*/    0x01ED,    0x5555,    0x07B5,
    /*2406*/    0x01ED,    0x89D8,    0x07B6,
    /*2407*/    0x01ED,    0xBE5B,    0x07B6,
    /*2408*/    0x01ED,    0xF2DF,    0x07B7,
    /*2409*/    0x01EE,    0x2762,    0x07B8,
    /*2410*/    0x01EE,    0x5BE5,    0x07B9,
    /*2411*/    0x01EE,    0x9069,    0x07BA,
    /*2412*/    0x01EE,    0xC4EC,    0x07BB,
    /*2413*/    0x01EE,    0xF96F,    0x07BB,
    /*2414*/    0x01EF,    0x2DF2,    0x07BC,
    /*2415*/    0x01EF,    0x6276,    0x07BD,
    /*2416*/    0x01EF,    0x96F9,    0x07BE,
    /*2417*/    0x01EF,    0xCB7C,    0x07BF,
    /*2418*/    0x01F0,    0x0000,    0x07C0,
    /*2419*/    0x01F0,    0x3483,    0x07C0,
    /*2420*/    0x01F0,    0x6906,    0x07C1,
    /*2421*/    0x01F0,    0x9D89,    0x07C2,
    /*2422*/    0x01F0,    0xD20D,    0x07C3,
    /*2423*/    0x01F1,    0x0690,    0x07C4,
    /*2424*/    0x01F1,    0x3B13,    0x07C4,
    /*2425*/    0x01F1,    0x6F96,    0x07C5,
    /*2426*/    0x01F1,    0xA41A,    0x07C6,
    /*2427*/    0x01F1,    0xD89D,    0x07C7,
    /*2428*/    0x01F2,    0x0D20,    0x07C8,
    /*2429*/    0x01F2,    0x41A4,    0x07C9,
    /*2430*/    0x01F2,    0x7627,    0x07C9,
    /*2431*/    0x01F2,    0xAAAA,    0x07CA,
    /*2432*/    0x01F2,    0xDF2D,    0x07CB,
    /*2433*/    0x01F3,    0x13B1,    0x07CC,
    /*2434*/    0x01F3,    0x4834,    0x07CD,
    /*2435*/    0x01F3,    0x7CB7,    0x07CD,
    /*2436*/    0x01F3,    0xB13B,    0x07CE,
    /*2437*/    0x01F3,    0xE5BE,    0x07CF,
    /*2438*/    0x01F4,    0x1A41,    0x07D0,
    /*2439*/    0x01F4,    0x4EC4,    0x07D1,
    /*2440*/    0x01F4,    0x8348,    0x07D2,
    /*2441*/    0x01F4,    0xB7CB,    0x07D2,
    /*2442*/    0x01F4,    0xEC4E,    0x07D3,
    /*2443*/    0x01F5,    0x20D2,    0x07D4,
    /*2444*/    0x01F5,    0x5555,    0x07D5,
    /*2445*/    0x01F5,    0x89D8,    0x07D6,
    /*2446*/    0x01F5,    0xBE5B,    0x07D6,
    /*2447*/    0x01F5,    0xF2DF,    0x07D7,
    /*2448*/    0x01F6,    0x2762,    0x07D8,
    /*2449*/    0x01F6,    0x5BE5,    0x07D9,
    /*2450*/    0x01F6,    0x9069,    0x07DA,
    /*2451*/    0x01F6,    0xC4EC,    0x07DB,
    /*2452*/    0x01F6,    0xF96F,    0x07DB,
    /*2453*/    0x01F7,    0x2DF2,    0x07DC,
    /*2454*/    0x01F7,    0x6276,    0x07DD,
    /*2455*/    0x01F7,    0x96F9,    0x07DE,
    /*2456*/    0x01F7,    0xCB7C,    0x07DF,
    /*2457*/    0x01F8,    0x0000,    0x07E0,
    /*2458*/    0x01F8,    0x3483,    0x07E0,
    /*2459*/    0x01F8,    0x6906,    0x07E1,
    /*2460*/    0x01F8,    0x9D89,    0x07E2,
    /*2461*/    0x01F8,    0xD20D,    0x07E3,
    /*2462*/    0x01F9,    0x0690,    0x07E4,
    /*2463*/    0x01F9,    0x3B13,    0x07E4,
    /*2464*/    0x01F9,    0x6F96,    0x07E5,
    /*2465*/    0x01F9,    0xA41A,    0x07E6,
    /*2466*/    0x01F9,    0xD89D,    0x07E7,
    /*2467*/    0x01FA,    0x0D20,    0x07E8,
    /*2468*/    0x01FA,    0x41A4,    0x07E9,
    /*2469*/    0x01FA,    0x7627,    0x07E9,
    /*2470*/    0x01FA,    0xAAAA,    0x07EA,
    /*2471*/    0x01FA,    0xDF2D,    0x07EB,
    /*2472*/    0x01FB,    0x13B1,    0x07EC,
    /*2473*/    0x01FB,    0x4834,    0x07ED,
    /*2474*/    0x01FB,    0x7CB7,    0x07ED,
    /*2475*/    0x01FB,    0xB13B,    0x07EE,
    /*2476*/    0x01FB,    0xE5BE,    0x07EF,
    /*2477*/    0x01FC,    0x1A41,    0x07F0,
    /*2478*/    0x01FC,    0x4EC4,    0x07F1,
    /*2479*/    0x01FC,    0x8348,    0x07F2,
    /*2480*/ 0x01FC, 0xB7CB, 0x07F2,
    /*2481*/ 0x01FC, 0xEC4E, 0x07F3,
    /*2482*/ 0x01FD, 0x20D2, 0x07F4,
    /*2483*/ 0x01FD, 0x5555, 0x07F5,
    /*2484*/ 0x01FD, 0x89D8, 0x07F6,
    /*2485*/ 0x01FD, 0xBE5B, 0x07F6,
    /*2486*/ 0x01FD, 0xF2DF, 0x07F7,
    /*2487*/ 0x01FE, 0x2762, 0x07F8,
    /*2488*/ 0x01FE, 0x5BE5, 0x07F9,
    /*2489*/ 0x01FE, 0x9069, 0x07FA,
    /*2490*/ 0x01FE, 0xC4EC, 0x07FB,
    /*2491*/ 0x01FE, 0xF96F, 0x07FB,
    /*2492*/ 0x01FF, 0x2DF2, 0x07FC,
    /*2493*/ 0x01FF, 0x6276, 0x07FD,
    /*2494*/ 0x01FF, 0x96F9, 0x07FE,
    /*2495*/ 0x01FF, 0xCB7C, 0x07FF,
    /*2380*/ 0x01E8, 0x3483, 0x07A0,
};



//   sdm_syn_divn_cfg1_reg0x5c,sdm_syn_divn_cfg2_reg0x5d,afc_fvco_fm_config0x67
//   DIVN_MSB_10Bit,DIVN_LSB_16Bit,fvco_target_fm
MXD_U16 gFmFreqHKhz[]=
{
    //0x201[6:5] = 2b'3
    //0x227[6] = 2b'3
    //Freq  reg5c reg5d reg67_0 reg67_1 reg67_2 reg67_3
    /*70.0  */  0x01AE,   0xC4EC,    0x1AEC,
    /*70.1  */  0x01AF,   0x6276,    0x1AF6,
    /*70.2  */  0x01B0,   0x0000,    0x1B00,
    /*70.3  */  0x01B0,   0x9D89,    0x1B09,
    /*70.4  */  0x01B1,   0x3B13,    0x1B13,
    /*70.5  */  0x01B1,   0xD89D,    0x1B1D,
    /*70.6  */  0x01B2,   0x7627,    0x1B27,
    /*70.7  */  0x01B3,   0x13B1,    0x1B31,
    /*70.8  */  0x01B3,   0xB13B,    0x1B3B,
    /*70.9  */  0x01B4,   0x4EC4,    0x1B44,
    /*71.0  */  0x01B4,   0xEC4E,    0x1B4E,
    /*71.1  */  0x01B5,   0x89D8,    0x1B58,
    /*71.2  */  0x01B6,   0x2762,    0x1B62,
    /*71.3  */  0x01B6,   0xC4EC,    0x1B6C,
    /*71.4  */  0x01B7,   0x6276,    0x1B76,
    /*71.5  */  0x01B8,   0x0000,    0x1B80,
    /*71.6  */  0x01B8,   0x9D89,    0x1B89,
    /*71.7  */  0x01B9,   0x3B13,    0x1B93,
    /*71.8  */  0x01B9,   0xD89D,    0x1B9D,
    /*71.9  */  0x01BA,   0x7627,    0x1BA7,
    /*72.0  */  0x01BB,   0x13B1,    0x1BB1,
    /*72.1  */  0x01BB,   0xB13B,    0x1BBB,
    /*72.2  */  0x01BC,   0x4EC4,    0x1BC4,
    /*72.3  */  0x01BC,   0xEC4E,    0x1BCE,
    /*72.4  */  0x01BD,   0x89D8,    0x1BD8,
    /*72.5  */  0x01BE,   0x2762,    0x1BE2,
    /*72.6  */  0x01BE,   0xC4EC,    0x1BEC,
    /*72.7  */  0x01BF,   0x6276,    0x1BF6,
    /*72.8  */  0x01C0,   0x0000,    0x1C00,
    /*72.9  */  0x01C0,   0x9D89,    0x1C09,
    /*73.0  */  0x01C1,   0x3B13,    0x1C13,
    /*73.1  */  0x01C1,   0xD89D,    0x1C1D,
    /*73.2  */  0x01C2,   0x7627,    0x1C27,
    /*73.3  */  0x01C3,   0x13B1,    0x1C31,
    /*73.4  */  0x01C3,   0xB13B,    0x1C3B,
    /*73.5  */  0x01C4,   0x4EC4,    0x1C44,
    /*73.6  */  0x01C4,   0xEC4E,    0x1C4E,
    /*73.7  */  0x01C5,   0x89D8,    0x1C58,
    /*73.8  */  0x01C6,   0x2762,    0x1C62,
    /*73.9  */  0x01C6,   0xC4EC,    0x1C6C,
    /*74.0  */  0x01C7,   0x6276,    0x1C76,
    /*74.1  */  0x01C8,   0x0000,    0x1C80,
    /*74.2  */  0x01C8,   0x9D89,    0x1C89,
    /*74.3  */  0x01C9,   0x3B13,    0x1C93,
    /*74.4  */  0x01C9,   0xD89D,    0x1C9D,
    /*74.5  */  0x01CA,   0x7627,    0x1CA7,
    /*74.6  */  0x01CB,   0x13B1,    0x1CB1,
    /*74.7  */  0x01CB,   0xB13B,    0x1CBB,
    /*74.8  */  0x01CC,   0x4EC4,    0x1CC4,
    /*74.9  */  0x01CC,   0xEC4E,    0x1CCE,
    /*75.0  */  0x01CD,   0x89D8,    0x1CD8,
    /*75.1  */  0x01CE,   0x2762,    0x1CE2,
    /*75.2  */  0x01CE,   0xC4EC,    0x1CEC,
    /*75.3  */  0x01CF,   0x6276,    0x1CF6,
    /*75.4  */  0x01D0,   0x0000,    0x1D00,
    /*75.5  */  0x01D0,   0x9D89,    0x1D09,
    /*75.6  */  0x01D1,   0x3B13,    0x1D13,
    /*75.7  */  0x01D1,   0xD89D,    0x1D1D,
    /*75.8  */  0x01D2,   0x7627,    0x1D27,
    /*75.9  */  0x01D3,   0x13B1,    0x1D31,
    /*76.0  */  0x01D3,   0xB13B,    0x1D3B,
    /*76.1  */  0x01D4,   0x4EC4,    0x1D44,
    /*76.2  */  0x01D4,   0xEC4E,    0x1D4E,
    /*76.3  */  0x01D5,   0x89D8,    0x1D58,
    /*76.4  */  0x01D6,   0x2762,    0x1D62,
    /*76.5  */  0x01D6,   0xC4EC,    0x1D6C,
    /*76.6  */  0x01D7,   0x6276,    0x1D76,
    /*76.7  */  0x01D8,   0x0000,    0x1D80,
    /*76.8  */  0x01D8,   0x9D89,    0x1D89,
    /*76.9  */  0x01D9,   0x3B13,    0x1D93,
    /*77.0  */  0x01D9,   0xD89D,    0x1D9D,
    /*77.1  */  0x01DA,   0x7627,    0x1DA7,
    /*77.2  */  0x01DB,   0x13B1,    0x1DB1,
    /*77.3  */  0x01DB,   0xB13B,    0x1DBB,
    /*77.4  */  0x01DC,   0x4EC4,    0x1DC4,
    /*77.5  */  0x01DC,   0xEC4E,    0x1DCE,
    /*77.6  */  0x01DD,   0x89D8,    0x1DD8,
    /*77.7  */  0x01DE,   0x2762,    0x1DE2,
    /*77.8  */  0x01DE,   0xC4EC,    0x1DEC,
    /*77.9  */  0x01DF,   0x6276,    0x1DF6,
    /*78.0  */  0x01E0,   0x0000,    0x1E00,
    /*78.1  */  0x01E0,   0x9D89,    0x1E09,
    /*78.2  */  0x01E1,   0x3B13,    0x1E13,
    /*78.3  */  0x01E1,   0xD89D,    0x1E1D,
    /*78.4  */  0x01E2,   0x7627,    0x1E27,
    /*78.5  */  0x01E3,   0x13B1,    0x1E31,
    /*78.6  */  0x01E3,   0xB13B,    0x1E3B,
    /*78.7  */  0x01E4,   0x4EC4,    0x1E44,
    /*78.8  */  0x01E4,   0xEC4E,    0x1E4E,
    /*78.9  */  0x01E5,   0x89D8,    0x1E58,
    /*79.0  */  0x01E6,   0x2762,    0x1E62,
    /*79.1  */  0x01E6,   0xC4EC,    0x1E6C,
    /*79.2  */  0x01E7,   0x6276,    0x1E76,
    /*79.3  */  0x01E8,   0x0000,    0x1E80,
    /*79.4  */  0x01E8,   0x9D89,    0x1E89,
    /*79.5  */  0x01E9,   0x3B13,    0x1E93,
    /*79.6  */  0x01E9,   0xD89D,    0x1E9D,
    /*79.7  */  0x01EA,   0x7627,    0x1EA7,
    /*79.8  */  0x01EB,   0x13B1,    0x1EB1,
    /*79.9  */  0x01EB,   0xB13B,    0x1EBB,
    /*80.0  */  0x01EC,   0x4EC4,    0x1EC4,
    /*80.1  */  0x01EC,   0xEC4E,    0x1ECE,
    /*80.2  */  0x01ED,   0x89D8,    0x1ED8,
    /*80.3  */  0x01EE,   0x2762,    0x1EE2,
    /*80.4  */  0x01EE,   0xC4EC,    0x1EEC,
    /*80.5  */  0x01EF,   0x6276,    0x1EF6,
    /*80.6  */  0x01F0,   0x0000,    0x1F00,
    /*80.7  */  0x01F0,   0x9D89,    0x1F09,
    /*80.8  */  0x01F1,   0x3B13,    0x1F13,
    /*80.9  */  0x01F1,   0xD89D,    0x1F1D,
    /*81.0  */  0x01F2,   0x7627,    0x1F27,
    /*81.1  */  0x01F3,   0x13B1,    0x1F31,
    /*81.2  */  0x01F3,   0xB13B,    0x1F3B,
    /*81.3  */  0x01F4,   0x4EC4,    0x1F44,
    /*81.4  */  0x01F4,   0xEC4E,    0x1F4E,
    /*81.5  */  0x01F5,   0x89D8,    0x1F58,
    /*81.6  */  0x01F6,   0x2762,    0x1F62,
    /*81.7  */  0x01F6,   0xC4EC,    0x1F6C,
    /*81.8  */  0x01F7,   0x6276,    0x1F76,
    /*81.9  */  0x01F8,   0x0000,    0x1F80,
    /*82.0  */  0x01F8,   0x9D89,    0x1F89,
    /*82.1  */  0x01F9,   0x3B13,    0x1F93,
    /*82.2  */  0x01F9,   0xD89D,    0x1F9D,
    /*82.3  */  0x01FA,   0x7627,    0x1FA7,
    /*82.4  */  0x01FB,   0x13B1,    0x1FB1,
    /*82.5  */  0x01FB,   0xB13B,    0x1FBB,
    /*82.6  */  0x01FC,   0x4EC4,    0x1FC4,  //*zl@0909**//
    /*82.7  */  0x01FC,   0xEC4E,    0x1FCE,
    /*82.8  */  0x01FD,   0x89D8,    0x1FD8,
    /*82.9  */  0x01FE,   0x2762,    0x1FE2,
    /*83.0  */  0x01FE,   0xC4EC,    0x1FEC,
    /*83.1  */  0x01FF,   0x6276,    0x1FF6,
    /*83.2  */  0x0200,   0x0000,    0x2000,
    /*83.3  */  0x0200,   0x9D89,    0x2009,
    /*83.4  */  0x0201,   0x3B13,    0x2013,
    /*83.5  */  0x0201,   0xD89D,    0x201D,
    /*83.6  */  0x0202,   0x7627,    0x2027,
    /*83.7  */  0x0203,   0x13B1,    0x2031,
    /*83.8  */  0x0203,   0xB13B,    0x203B,
    /*83.9  */  0x0204,   0x4EC4,    0x2044,
    /*84.0  */  0x0204,   0xEC4E,    0x204E,
    /*84.1  */  0x0205,   0x89D8,    0x2058,
    /*84.2  */  0x0206,   0x2762,    0x2062,
    /*84.3  */  0x0206,   0xC4EC,    0x206C,
    /*84.4  */  0x0207,   0x6276,    0x2076,
    /*84.5  */  0x0208,   0x0000,    0x2080,
    /*84.6  */  0x0208,   0x9D89,    0x2089,
    /*84.7  */  0x0209,   0x3B13,    0x2093,
    /*84.8  */  0x0209,   0xD89D,    0x209D,
    /*84.9  */  0x020A,   0x7627,    0x20A7,
    /*85.0  */  0x020B,   0x13B1,    0x20B1,
    /*85.1  */  0x020B,   0xB13B,    0x20BB,
    /*85.2  */  0x020C,   0x4EC4,    0x20C4,
    /*85.3  */  0x020C,   0xEC4E,    0x20CE,
    /*85.4  */  0x020D,   0x89D8,    0x20D8,
    /*85.5  */  0x020E,   0x2762,    0x20E2,
    /*85.6  */  0x020E,   0xC4EC,    0x20EC,
    /*85.7  */  0x020F,   0x6276,    0x20F6,
    /*85.8  */  0x0210,   0x0000,    0x2100,
    /*85.9  */  0x0210,   0x9D89,    0x2109,
    /*86.0  */  0x0211,   0x3B13,    0x2113,
    /*86.1  */  0x0211,   0xD89D,    0x211D,
    /*86.2  */  0x0212,   0x7627,    0x2127,
    /*86.3  */  0x0213,   0x13B1,    0x2131,
    /*86.4  */  0x0213,   0xb13b,    0x213b, /*zl@  0408   0x01A9,   0x5A95,    0x1A95,  */
    /*86.5  */  0x01A9,   0xD89D,    0x1A9D,    //*zl@0909**//
    /*86.6  */  0x01AA,   0x56A5,    0x1AA5,
    /*86.7  */  0x01AA,   0xD4AD,    0x1AAD,
    /*86.8  */  0x01AB,   0x52B5,    0x1AB5,
    /*86.9  */  0x01AB,   0xD0BD,    0x1ABD,
    /*87.0  */  0x01AC,   0x4EC4,    0x1AC4,
    /*87.1  */  0x01AC,   0xCCCC,    0x1ACC,
    /*87.2  */  0x01AD,   0x4AD4,    0x1AD4,
    /*87.3  */  0x01AD,   0xC8DC,    0x1ADC,
    /*87.4  */  0x01AE,   0x46E4,    0x1AE4,
    /*87.5  */  0x01AE,   0xC4EC,    0x1AEC,
    /*87.6  */  0x01AF,   0x42F4,    0x1AF4,
    /*87.7  */  0x01AF,   0xC0FC,    0x1AFC,
    /*87.8  */  0x01B0,   0x3F03,    0x1B03,
    /*87.9  */  0x01B0,   0xBD0B,    0x1B0B,
    /*88.0  */  0x01B1,   0x3B13,    0x1B13,
    /*88.1  */  0x01B1,   0xB91B,    0x1B1B,
    /*88.2  */  0x01B2,   0x3723,    0x1B23,
    /*88.3  */  0x01B2,   0xB52B,    0x1B2B,
    /*88.4  */  0x01B3,   0x3333,    0x1B33,
    /*88.5  */  0x01B3,   0xB13B,    0x1B3B,
    /*88.6  */  0x01B4,   0x2F42,    0x1B42,
    /*88.7  */  0x01B4,   0xAD4A,    0x1B4A,
    /*88.8  */  0x01B5,   0x2B52,    0x1B52,
    /*88.9  */  0x01B5,   0xA95A,    0x1B5A,
    /*89.0  */  0x01B6,   0x2762,    0x1B62,
    /*89.1  */  0x01B6,   0xA56A,    0x1B6A,
    /*89.2  */  0x01B7,   0x2372,    0x1B72,
    /*89.3  */  0x01B7,   0xA17A,    0x1B7A,
    /*89.4  */  0x01B8,   0x1F81,    0x1B81,
    /*89.5  */  0x01B8,   0x9D89,    0x1B89,
    /*89.6  */  0x01B9,   0x1B91,    0x1B91,
    /*89.7  */  0x01B9,   0x9999,    0x1B99,
    /*89.8  */  0x01BA,   0x17A1,    0x1BA1,
    /*89.9  */  0x01BA,   0x95A9,    0x1BA9,
    /*90.0  */  0x01BB,   0x13B1,    0x1BB1,
    /*90.1  */  0x01BB,   0x91B9,    0x1BB9,
    /*90.2  */  0x01BC,   0x0FC0,    0x1BC0, //*zl@0909**//
    /*90.3  */  0x01BC,   0x8DC8,    0x1BC8,
    /*90.4  */  0x01BD,   0x0BD0,    0x1BD0,
    /*90.5  */  0x01BD,   0x89D8,    0x1BD8, //*zl@0909**//
    /*90.6  */  0x01BE,   0x07E0,    0x1BE0,//*zl@0909**//
    /*90.7  */  0x01BE,   0x85E8,    0x1BE8,
    /*90.8  */  0x01BF,   0x03F0,    0x1BF0,
    /*90.9  */  0x01BF,   0x81F8,    0x1BF8,
    /*91.0  */  0x01C0,   0x0000,    0x1C00,
    /*91.1  */  0x01C0,   0x7E07,    0x1C07,
    /*91.2  */  0x01C0,   0xFC0F,    0x1C0F,
    /*91.3  */  0x01C1,   0x7A17,    0x1C17,
    /*91.4  */  0x01C1,   0xF81F,    0x1C1F,
    /*91.5  */  0x01C2,   0x7627,    0x1C27,
    /*91.6  */  0x01C2,   0xF42F,    0x1C2F,
    /*91.7  */  0x01C3,   0x7237,    0x1C37,
    /*91.8  */  0x01C3,   0xF03F,    0x1C3F,
    /*91.9  */  0x01C4,   0x6E46,    0x1C46,
    /*92.0  */  0x01C4,   0xEC4E,    0x1C4E,
    /*92.1  */  0x01C5,   0x6A56,    0x1C56,
    /*92.2  */  0x01C5,   0xE85E,    0x1C5E,
    /*92.3  */  0x01C6,   0x6666,    0x1C66,
    /*92.4  */  0x01C6,   0xE46E,    0x1C6E,
    /*92.5  */  0x01C7,   0x6276,    0x1C76,
    /*92.6  */  0x01C7,   0xE07E,    0x1C7E,//*zl@0909**//
    /*92.7  */  0x01C8,   0x5E85,    0x1C85,
    /*92.8  */  0x01C8,   0xDC8D,    0x1C8D,
    /*92.9  */  0x01C9,   0x5A95,    0x1C95,
    /*93.0  */  0x01C9,   0xD89D,    0x1C9D,
    /*93.1  */  0x01CA,   0x56A5,    0x1CA5,
    /*93.2  */  0x01CA,   0xD4AD,    0x1CAD,
    /*93.3  */  0x01CB,   0x52B5,    0x1CB5,
    /*93.4  */  0x01CB,   0xD0BD,    0x1CBD,
    /*93.5  */  0x01CC,   0x4EC4,    0x1CC4,
    /*93.6  */  0x01CC,   0xCCCC,    0x1CCC,
    /*93.7  */  0x01CD,   0x4AD4,    0x1CD4,
    /*93.8  */  0x01CD,   0xC8DC,    0x1CDC,
    /*93.9  */  0x01CE,   0x46E4,    0x1CE4,
    /*94.0  */  0x01CE,   0xC4EC,    0x1CEC,
    /*94.1  */  0x01CF,   0x42F4,    0x1CF4,
    /*94.2  */  0x01CF,   0xC0FC,    0x1CFC,
    /*94.3  */  0x01D0,   0x3F03,    0x1D03,
    /*94.4  */  0x01D0,   0xBD0B,    0x1D0B,
    /*94.5  */  0x01D1,   0x3B13,    0x1D13,
    /*94.6  */  0x01D1,   0xB91B,    0x1D1B,
    /*94.7  */  0x01D2,   0x3723,    0x1D23,
    /*94.8  */  0x01D2,   0xB52B,    0x1D2B,
    /*94.9  */  0x01D3,   0x3333,    0x1D33,
    /*95.0  */  0x01D3,   0xB13B,    0x1D3B,
    /*95.1  */  0x01D4,   0x2F42,    0x1D42,
    /*95.2  */  0x01D4,   0xAD4A,    0x1D4A,
    /*95.3  */  0x01D5,   0x2B52,    0x1D52,
    /*95.4  */  0x01D5,   0xA95A,    0x1D5A,
    /*95.5  */  0x01D6,   0x2762,    0x1D62,
    /*95.6  */  0x01D6,   0xA56A,    0x1D6A,
    /*95.7  */  0x01D7,   0x2372,    0x1D72,
    /*95.8  */  0x01D7,   0xA17A,    0x1D7A,
    /*95.9  */  0x01D8,   0x1F81,    0x1D81,//*zl@0909**//
    /*96.0  */  0x01D8,   0x9D89,    0x1D89,
    /*96.1  */  0x01D9,   0x1B91,    0x1D91,
    /*96.2  */  0x01D9,   0x9999,    0x1D99,
    /*96.3  */  0x01DA,   0x17A1,    0x1DA1,
    /*96.4  */  0x01DA,   0x95A9,    0x1DA9,
    /*96.5  */  0x01DB,   0x13B1,    0x1DB1,
    /*96.6  */  0x01DB,   0x91B9,    0x1DB9,
    /*96.7  */  0x01DC,   0x0FC0,    0x1DC0,//*zl@0909**//
    /*96.8  */  0x01DC,   0x8DC8,    0x1DC8,
    /*96.9  */  0x01DD,   0x0BD0,    0x1DD0,
    /*97.0  */  0x01DD,   0x89D8,    0x1DD8,
    /*97.1  */  0x01DE,   0x07E0,    0x1DE0,
    /*97.2  */  0x01DE,   0x85E8,    0x1DE8,
    /*97.3  */  0x01DF,   0x03F0,    0x1DF0,
    /*97.4  */  0x01DF,   0x81F8,    0x1DF8,
    /*97.5  */  0x01E0,   0x0000,    0x1E00,
    /*97.6  */  0x01E0,   0x7E07,    0x1E07,
    /*97.7  */  0x01E0,   0xFC0F,    0x1E0F,
    /*97.8  */  0x01E1,   0x7A17,    0x1E17,
    /*97.9  */  0x01E1,   0xF81F,    0x1E1F,
    /*98.0  */  0x01E2,   0x7627,    0x1E27,
    /*98.1  */  0x01E2,   0xF42F,    0x1E2F,
    /*98.2  */  0x01E3,   0x7237,    0x1E37,
    /*98.3  */  0x01E3,   0xF03F,    0x1E3F,
    /*98.4  */  0x01E4,   0x6E46,    0x1E46,
    /*98.5  */  0x01E4,   0xEC4E,    0x1E4E,
    /*98.6  */  0x01E5,   0x6A56,    0x1E56,
    /*98.7  */  0x01E5,   0xE85E,    0x1E5E,
    /*98.8  */  0x01E6,   0x6666,    0x1E66,
    /*98.9  */  0x01E6,   0xE46E,    0x1E6E,
    /*99.0  */  0x01E7,   0x6276,    0x1E76,
    /*99.1  */  0x01E7,   0xE07E,    0x1E7E, //*zl@0909**//
    /*99.2  */  0x01E8,   0x5E85,    0x1E85,
    /*99.3  */  0x01E8,   0xDC8D,    0x1E8D,
    /*99.4  */  0x01E9,   0x5A95,    0x1E95,
    /*99.5  */  0x01E9,   0xD89D,    0x1E9D,
    /*99.6  */  0x01EA,   0x56A5,    0x1EA5,
    /*99.7  */  0x01EA,   0xD4AD,    0x1EAD,
    /*99.8  */  0x01EB,   0x52B5,    0x1EB5,
    /*99.9  */  0x01EB,   0xD0BD,    0x1EBD,
    /*100.0 */  0x01EC,   0x4EC4,    0x1EC4,
    /*100.1 */  0x01EC,   0xCCCC,    0x1ECC,
    /*100.2 */  0x01ED,   0x4AD4,    0x1ED4,
    /*100.3 */  0x01ED,   0xC8DC,    0x1EDC,
    /*100.4 */  0x01EE,   0x46E4,    0x1EE4,
    /*100.5 */  0x01EE,   0xC4EC,    0x1EEC,
    /*100.6 */  0x01EF,   0x42F4,    0x1EF4,
    /*100.7 */  0x01EF,   0xC0FC,    0x1EFC,
    /*100.8 */  0x01F0,   0x3F03,    0x1F03,
    /*100.9 */  0x01F0,   0xBD0B,    0x1F0B,
    /*101.0 */  0x01F1,   0x3B13,    0x1F13,
    /*101.1 */  0x01F1,   0xB91B,    0x1F1B,
    /*101.2 */  0x01F2,   0x3723,    0x1F23,
    /*101.3 */  0x01F2,   0xB52B,    0x1F2B,
    /*101.4 */  0x01F3,   0x3333,    0x1F33,
    /*101.5 */  0x01F3,   0xB13B,    0x1F3B,
    /*101.6 */  0x01F4,   0x2F42,    0x1F42,
    /*101.7 */  0x01F4,   0xAD4A,    0x1F4A,
    /*101.8 */  0x01F5,   0x2B52,    0x1F52,
    /*101.9 */  0x01F5,   0xA95A,    0x1F5A,
    /*102.0 */  0x01F6,   0x2762,    0x1F62,
    /*102.1 */  0x01F6,   0xA56A,    0x1F6A,
    /*102.2 */  0x01F7,   0x2372,    0x1F72,
    /*102.3 */  0x01F7,   0xA17A,    0x1F7A,
    /*102.4 */  0x01F8,   0x1F81,    0x1F81,//*zl@0909**//
    /*102.5 */  0x01F8,   0x9D89,    0x1F89,
    /*102.6 */  0x01F9,   0x1B91,    0x1F91,
    /*102.7 */  0x01F9,   0x9999,    0x1F99,
    /*102.8 */  0x01FA,   0x17A1,    0x1FA1,
    /*102.9 */  0x01FA,   0x95A9,    0x1FA9,
    /*103.0 */  0x01FB,   0x13B1,    0x1FB1,
    /*103.1 */  0x01FB,   0x91B9,    0x1FB9,
    /*103.2 */  0x01FC,   0x0FC0,    0x1FC0,//*zl@0909**//
    /*103.3 */  0x01FC,   0x8DC8,    0x1FC8,
    /*103.4 */  0x01FD,   0x0BD0,    0x1FD0,
    /*103.5 */  0x01FD,   0x89D8,    0x1FD8,
    /*103.6 */  0x01FE,   0x07E0,    0x1FE0,
    /*103.7 */  0x01FE,   0x85E8,    0x1FE8,//*zl@0909**//
    /*103.8 */  0x01FF,   0x03F0,    0x1FF0,//*zl@0909**//
    /*103.9 */  0x01FF,   0x81F8,    0x1FF8,
    /*104.0 */  0x0200,   0x0000,    0x2000,
    /*104.1 */  0x0200,   0x7E07,    0x2007,
    /*104.2 */  0x0200,   0xFC0F,    0x200F,
    /*104.3 */  0x0201,   0x7A17,    0x2017,
    /*104.4 */  0x0201,   0xF81F,    0x201F,
    /*104.5 */  0x0202,   0x7627,    0x2027,
    /*104.6 */  0x0202,   0xF42F,    0x202F,
    /*104.7 */  0x0203,   0x7237,    0x2037,
    /*104.8 */  0x0203,   0xF03F,    0x203F,//*zl@0909**//
    /*104.9 */  0x0204,   0x6E46,    0x2046,
    /*105.0 */  0x0204,   0xEC4E,    0x204E,
    /*105.1 */  0x0205,   0x6A56,    0x2056,
    /*105.2 */  0x0205,   0xE85E,    0x205E,
    /*105.3 */  0x0206,   0x6666,    0x2066,
    /*105.4 */  0x0206,   0xE46E,    0x206E,
    /*105.5 */  0x0207,   0x6276,    0x2076,
    /*105.6 */  0x0207,   0xE07E,    0x207E,//*zl@0909**//
    /*105.7 */  0x0208,   0x5E85,    0x2085,
    /*105.8 */  0x0208,   0xDC8D,    0x208D,
    /*105.9 */  0x0209,   0x5A95,    0x2095,//*zl@0909**//
    /*106.0 */  0x0209,   0xD89D,    0x209D,
    /*106.1 */  0x020A,   0x56A5,    0x20A5,
    /*106.2 */  0x020A,   0xD4AD,    0x20AD,
    /*106.3 */  0x020B,   0x52B5,    0x20B5,
    /*106.4 */  0x020B,   0xD0BD,    0x20BD,
    /*106.5 */  0x020C,   0x4EC4,    0x20C4,
    /*106.6 */  0x020C,   0xCCCC,    0x20CC,
    /*106.7 */  0x020D,   0x4AD4,    0x20D4,
    /*106.8 */  0x020D,   0xC8DC,    0x20DC,
    /*106.9 */  0x020E,   0x46E4,    0x20E4,
    /*107.0 */  0x020E,   0xC4EC,    0x20EC,//*zl@0909**//
    /*107.1 */  0x020F,   0x42F4,    0x20F4,
    /*107.2 */  0x020F,   0xC0FC,    0x20FC,
    /*107.3 */  0x0210,   0x3F03,    0x2103,
    /*107.4 */  0x0210,   0xBD0B,    0x210B,
    /*107.5 */  0x0211,   0x3B13,    0x2113,
    /*107.6 */  0x0211,   0xB91B,    0x211B,
    /*107.7 */  0x0212,   0x3723,    0x2123,
    /*107.8 */  0x0212,   0xB52B,    0x212B,
    /*107.9 */  0x0213,   0x3333,    0x2133,
    /*108.0 */  0x0213,   0xB13B,    0x213B,
};

MXD_U16 gFmFreqHKhzForSpur[]=
{
#if 0
    /*82.6  */  0x01FC,   0x4EC4,    0x1FC4, //*zl@0909**//
    /*86.5  */  0x01A9,   0xD89D,    0x1A9D, //*zl@0909**//
    /*90.2  */  0x01BC,   0x0FC0,    0x1BC0, //*zl@0909**//
    /*90.5  */  0x01BD,   0x89D8,    0x1BD8, //*zl@0909**//
    /*90.6  */  0x01BE,   0x07E0,    0x1BE0,//*zl@0909**//
    /*92.6  */  0x01C7,   0xE07E,    0x1C7E,//*zl@0909**//
    /*95.9  */  0x01D8,   0x1F81,    0x1D81,//*zl@0909**//
    /*96.7  */  0x01DC,   0x0FC0,    0x1DC0,//*zl@0909**//
    /*99.1  */  0x01E7,   0xE07E,    0x1E7E,//*zl@0909**//
    /*102.4 */  0x01F8,   0x1F81,    0x1F81,//*zl@0909**//
    /*103.2 */  0x01FC,   0x0FC0,    0x1FC0,//*zl@0909**//
    /*103.7 */  0x01FE,   0x85E8,    0x1FE8,//*zl@0909**//
    /*103.8 */  0x01FF,   0x03F0,    0x1FF0,//*zl@0909**//
    /*104.8 */  0x0203,   0xF03F,    0x203F,//*zl@0909**//
    /*105.6 */  0x0207,   0xE07E,    0x207E,//*zl@0909**//
    /*105.9 */  0x0209,   0x5A95,    0x2095,//*zl@0909**//
    /*107.0 */  0x020E,   0xC4EC,    0x20EC,//*zl@0909**//
#endif
	826 , 865 , 880 , 886 , 892 , 900 , 
	904 , 902 , 906 , 907 , 911 , 916 , 
	921 , 926 , 935 , 941 , 943 , 944 , 
	953 , 956 , 959 , 961 , 967 , 968 , 
	973 , 974 , 983 , 984 , 991 , 992 , 
	995 , 996 , 1004,1011,1019,1024,
	1026,1032,1037,1038,1044,1048,
	1051,1056,1058,1059,1068,1069,
	1070,1072,1073,1074,1077,1078,
};
MXD_U16 gCastorRegComInit[]=
{
// fd config 0331

    0x0201, 0x3fff,  //ldo all on;   0331
    // set mxd mode config,  zl@@@0327
    0x0400, 0x0000,  // init tbl h
    0x0401, 0x0000,  // init tbl m
    0x0402, 0x0003,  // init tbl l zl@1014  3:chipC7/C8/CA   0:chipC7
    0x0403, 0x0331,  // wftx tbl h
    0x0404, 0xe011,  // wftx tbl m
    0x0405, 0x8003,  // wftx tbl l zl@1014  8003:chipC7/C8/CA   8000:chipC7
    0x0406, 0x0339,  // bttx tbl h
    0x0407, 0x1c12,  // bttx tbl m
    0x0408, 0x8003,  // bttx tbl lzl@1014  80033:chipC7/C8/CA   8000:chipC7
    0x409,0x01c3,//0503  0x0409, 0x00c3,  ->1c3 // wfrx tbl h  //?????c3 is good than 1c3
    0x040a, 0x0389,  // wfrx tbl m
    0x040b, 0xca00,  // wfrx tbl l
    0x40c,0x01c5,// 0503 0x040c, 0x00c5,->1c5  // btrx tbl h  //
    0x040d, 0x0066,  // btrx tbl m
    0x040e, 0xb400,  // btrx tbl l
    0x040f, 0x01c7, //0506 0x00c7->1c7 // wtco tbl h        //?????
    0x0410, 0x03ef,  // wtco tbl m
    0x0411, 0xfe00,  // wtco tbl l
    0x0412, 0x0000,  // bak6 tbl h
    0x0413, 0x0000,  // bak6 tbl m
    0x0414, 0x0000,  // bak6 tbl l
    0x0415, 0x0000,  // bak7 tbl h
    0x0416, 0x0000,  // bak7 tbl m
    0x0417, 0x0000,  // bak7 tbl l
// set control c source
    0x00c2, 0x00ff,  // 0331
    0x00c7, 0x0000,
    0x021F, 0x0101,  // Set nolap LO LO mixer enalbe
    0x0209,0xc0c1,//    0x209,0xc0c1,  //0503  0x0209, 0x00c1,  // Config LNA SW bias CTRL_VB2
    0x020a, 0xa4a4,  // Config LNA VB1_Main VB1_AUX
    0x020F, 0x030d,  // Enable WF gmctrl 11 hgm 11 hvsat 011
    0x0210, 0xa75B,  // LNA 0dB TIA 32dB PGA 0dB -38dBm

    0x0208, 0x022F,  // Enable PA config
    0x0213, 0x000f,  // Disable negm tuning gm output cap
    0x0211, 0x003d,  // WF TX pga: 0dB; TxGm 0dB
//  BT pga 2dB       0331
    0x021a, 0x8080,  //
//BT LOFT DAC config                          0331
    0x021b, 0x8080,  // WF LOFT DAC config
// PA config  c

    0x020b, 0xa15a,
    0x020c, 0x5847,     // 7f67
    0x020d, 0xd27f,     //0xbc7f,  // bc4f gdyupdate0408 email
    0x020e, 0x89cd,     //0x807c,  // 5f00 gdyupdate0408 email

//0x00bd, 0x0018,  // sprd mode     //
//       0331
//0000be, 0x6b8c,  // sprd mode definiton 10/00 bt t/r 01/11 wf t/r // 0331
    0x0418, 0x0000,  // fm dcoc,
    //0331
    0x0419, 0x0000,  // fm dcoc,
    0x041A, 0x0000,  // fm dcoc,
    0x041B, 0x0000,  // bt dcoc,
    0x041C, 0x0000,  // bt dcoc,
    0x041D, 0x0000,  // bt dcoc,
    0x041E, 0x0000,  // wf dcoc,
    0x041F, 0x0000,  // wf dcoc,
    0x0420, 0x0000,  // wf dcoc,   //0331
    0x0421, 0x0112, // bt loft,
    0x0422, 0x9E47, // bt loft,
    0x0423, 0x0607, // bt loft,
    0x0424, 0x0112, // wf loft,
    0x0425, 0x9A78, // wf loft,
    0x0426, 0x0607, // wf loft,
    0x0427, 0x0112, // bt tx imb,
    0x0428, 0x9E47, // bt tx imb,
    0x0429, 0x0607, // bt tx imb,
    0x042A, 0x1112, // wf tx imb,
    0x042B, 0x9A78, // wf tx imb,
    0x042C, 0x0607, // wf tx imb,
    0x042D, 0x0112, // pout,
    0x042E, 0x9A78, // pout,
    0x042F, 0x0607, // pout,
    // wf rx gain map 0~127 map,
// 0404update to linear agc 0331 gdy @@
    0x0477,  0x7F7E,
    0x0476,  0x7D7C,
    0x0475,  0x5F5E,
    0x0474,  0x5D5C,
    0x0473,  0x5B5A,
    0x0472,  0x5958,
    0x0471,  0x5756,
    0x0470,  0x5554,
    0x046F,  0x5352,
    0x046E,  0x5150,
    0x046D,  0x4F4E,
    0x046C,  0x4D4C,
    0x046B,  0x1110,
    0x046A,  0x0F0E,
    0x0469,  0x0D0D,
    0x0468,  0x0D0D,
    0x0467,  0x7F7E,
    0x0466,  0x7D7C,
    0x0465,  0x5F5E,
    0x0464,  0x5D5C,
    0x0463,  0x5B5A,
    0x0462,  0x5958,
    0x0461,  0x5756,
    0x0460,  0x5554,
    0x045F,  0x5352,
    0x045E,  0x5150,
    0x045D,  0x4F4E,
    0x045C,  0x4D4C,
    0x045B,  0x1110,
    0x045A,  0x0F0E,
    0x0459,  0x0D0D,
    0x0458,  0x0D0D,
    0x0457,  0x7F7E,
    0x0456,  0x7D7C,
    0x0455,  0x5F5E,
    0x0454,  0x5D5C,
    0x0453,  0x5B5A,
    0x0452,  0x5958,
    0x0451,  0x5756,
    0x0450,  0x5554,
    0x044F,  0x5352,
    0x044E,  0x5150,
    0x044D,  0x4F4E,
    0x044C,  0x4D4C,
    0x044B,  0x1110,
    0x044A,  0x0F0E,
    0x0449,  0x0D0D,
    0x0448,  0x0D0D,


// MR@@422
#ifndef  TROUT2_CHIP_VER_V2
    0x447 , 0x7f7f, // xiaomei 0422
    0x446 , 0x7f7e,
    0x445 , 0x7d7c,
    0x444 , 0x7B7A ,  //7B7A
    0x443 , 0x7978 ,  //7978
    0x442 , 0x7676 ,  //7776  confirmed on 51dbm cant work perfect
    0x441 , 0x7558 ,  //7554
    0x440 , 0x5756 ,  //add  0407
    0x43f , 0x5536,   //add  36 ok
    0x43e , 0x5352 ,
    0x43d , 0x5150 ,
    0x43c , 0x4f4e ,
    0x43b , 0x4d68 ,  //add1110 ,
    0x43a , 0x1110 ,
    0x439 , 0x0f0e ,
    0x438 , 0x0d0d ,
#else
//0527 debug for mobile Plan2   // xiao mei 0524plan1       //    old
// for noLna config
    0x0447, 0x7F7F,                     //0x0447, 0x7F7F,                 //0x0447, 0x7f7f,      //    0x0447, 0x7F7F,
    0x0446, 0x7E7D,                     //0x0446, 0x7E7D,                 //0x0446, 0x7f7e,      //    0x0446, 0x7e7d,
    0x0445, 0x7D7C,                     //0x0445, 0x7D7C,                 //0x0445, 0x7d7c,      //    0x0445, 0x7D7C,
    0x0444, 0x7B7A,                     //0x0444, 0x7B7A,                 //0x0444, 0x7B7A,      //    0x0444, 0x7B7A,
    0x0443, 0x7978,                     //0x0443, 0x7978,                 //0x0443, 0x7978,      //    0x0443, 0x7978,
    0x0442, 0x7776,                     //0x0442, 0x7776,                 //0x0442, 0x7776,      //    0x0442, 0x7776,
    0x0441, 0x7574,                     //0x0441, 0x7554,                 //0x0441, 0x7558,      //    0x0441, 0x7554,
    0x0440, 0x7372,                     //0x0440, 0x5352,                 //0x0440, 0x5756,      //    0x0440, 0x5352,
    0x043F, 0x7110,                     //0x043F, 0x5150,                 //0x043F, 0x5536,      //    0x043F, 0x5251,
    0x043E, 0x0F0E,                     //0x043E, 0x4F4E,                 //0x043E, 0x5352,      //    0x043E, 0x504F,
    0x043D, 0x0D0C,                     //0x043D, 0x4D4C,                 //0x043D, 0x5150,      //    0x043D, 0x4E4D,
    0x043C, 0x0B0A,                     //0x043C, 0x0B0A,                 //0x043C, 0x4f4e,      //    0x043C, 0x4C0B,
    0x043B, 0x0908,                     //0x043B, 0x0908,                 //0x043B, 0x4d09,      //    0x043B, 0x0A09,
    0x043A, 0x0706,                     //0x043A, 0x0706,                 //0x043A, 0x0810,      //    0x043A, 0x0807,
    0x0439, 0x0504,                     //0x0439, 0x0504,                 //0x0439, 0x0f0e,      //    0x0439, 0x0605,
    0x0438, 0x0302,                     //0x0438, 0x0302,                 //0x0438, 0x0d0d,      //    0x0438, 0x0403,

#endif
    // wfrx gain table,
    //  0331
    0x0486, 0xa4a4,  // lna gid3
    0x0487, 0x0004,
    0x0484, 0xa4a4, //0xa4a4,  //0xf6f6,
    0x0485, 0x0002,
    0x0482, 0xa4a4,
    0x0483, 0x0001,
    0x0480, 0xa4a4,
    0x0481, 0x0000,

#ifndef TROUT2_CHIP_VER_V2
    // V2b
    0x073C, 0x1BD7,  // if gid31
    0x073D, 0x8080,
    0x073E, 0x8080,
    0x0738, 0x1BCF,
    0x0739, 0x8080,
    0x073A, 0x8080,
//0x0734, 0x1BC7, original
    0x0734, 0x1BD3, //0417
    0x0735, 0x8080,
    0x0736, 0x8080,
//0x0730, 0x1BBF, original
    0x0730, 0x1BCB,
    0x0731, 0x8080,
    0x0732, 0x8080,
//0x072C, 0x1BB7, original
    0x072C, 0x1BD2, //0417
    0x072D, 0x8080,
    0x072E, 0x8080,
//0x0728, 0x1BAF, original
    0x0728, 0x1BCA,  //0417
    0x0729, 0x8080,
    0x072A, 0x8080,
//0x0724, 0x1BB3, original
    0x0724, 0x1BD1,//0417
    0x0725, 0x8080,
    0x0726, 0x8080,
//0x0720, 0x1BAB, original
    0x0720, 0x1BC9,//0417
    0x0721, 0x8080,
    0x0722, 0x8080,
//0x071C, 0x1BC1, ORIGINAL
    0x071C, 0x1BD0, //0417
    0x071D, 0x8080,
    0x071E, 0x8080,
//0x0718, 0x1BB9, ORIGINAL
    0x0718, 0x1BC8,  //0417
    0x0719, 0x8080,
    0x071A, 0x8080,
//0x0714, 0x1BB1, ORIGINAL
    0x0714, 0x1BC0, //0417
    0x0715, 0x8080,
    0x0716, 0x8080,
    0x0710, 0x1BB0,
    0x0711, 0x8080,
    0x0712, 0x8080,
    0x070C, 0x1BB0,
    0x070D, 0x8080,
    0x070E, 0x8080,
    0x0708, 0x1BA8,
    0x0709, 0x8080,
    0x070A, 0x8080,
    0x0704, 0x0130,
    0x0705, 0x8080,
    0x0706, 0x8080,
    0x0700, 0x0128,
    0x0701, 0x8080,
    0x0702, 0x8080,
    0x06FC, 0x0120,
    0x06FD, 0x8080,
    0x06FE, 0x8080,
    0x06F8, 0x0118,
    0x06F9, 0x8080,
    0x06FA, 0x8080,
    0x06F4, 0x0110,
    0x06F5, 0x8080,
    0x06F6, 0x8080,
    0x06F0, 0x0698,
    0x06F1, 0x8080,
    0x06F2, 0x8080,
    0x06EC, 0x0690,
    0x06ED, 0x8080,
    0x06EE, 0x8080,
    0x06E8, 0x0098,
    0x06E9, 0x8080,
    0x06EA, 0x8080,
    0x06E4, 0x0090,
    0x06E5, 0x8080,
    0x06E6, 0x8080,
    0x06E0, 0x0088,
    0x06E1, 0x8080,
    0x06E2, 0x8080,
    0x06DC, 0x0080,
    0x06DD, 0x8080,
    0x06DE, 0x8080,
    0x06D8, 0x0080,
    0x06D9, 0x8080,
    0x06DA, 0x8080,
    0x06D4, 0x0080,
    0x06D5, 0x8080,
    0x06D6, 0x8080,
    0x06D0, 0x0080,
    0x06D1, 0x8080,
    0x06D2, 0x8080,
    0x06CC, 0x0080,
    0x06CD, 0x8080,
    0x06CE, 0x8080,
    0x06C8, 0x0080,
    0x06C9, 0x8080,
    0x06CA, 0x8080,
    0x06C4, 0x0080,
    0x06C5, 0x8080,
    0x06C6, 0x8080,
    0x06C0, 0x0080,
    0x06C1, 0x8080,
    0x06C2, 0x8080,
#else
    //for chip v2x
    0x073C,0x1BD7,
    0x073D,0x8080,
    0x073E,0x8080,

    0x0738,0x1BCF,
    0x0739,0x8080,
    0x073A,0x8080,

    0x0734,0x1BD3,
    0x0735,0x8080,
    0x0736,0x8080,

    0x0730,0x1BCB,
    0x0731,0x8080,
    0x0732,0x8080,

    0x072C,0x1BD2,
    0x072D,0x8080,
    0x072E,0x8080,

    0x0728,0x1BCA,
    0x0729,0x8080,
    0x072A,0x8080,

    0x0724,0x1BD1,
    0x0725,0x8080,
    0x0726,0x8080,

    0x0720,0x1BC9,
    0x0721,0x8080,
    0x0722,0x8080,

    0x071C,0x1BD0,
    0x071D,0x8080,
    0x071E,0x8080,

    0x0718,0x1BC8,
    0x0719,0x8080,
    0x071A,0x8080,

    0x0714,0x1BC0,
    0x0715,0x8080,
    0x0716,0x8080,

    0x0710,0x1BB8,
    0x0711,0x8080,
    0x0712,0x8080,

    0x070C,0x1BB0,
    0x070D,0x8080,
    0x070E,0x8080,

    0x0708,0x1BA8,
    0x0709,0x8080,
    0x070A,0x8080,

    0x0704,0x1BA0,
    0x0705,0x8080,
    0x0706,0x8080,

    0x0700,0x1bcb,
    0x0701,0x8080,
    0x0702,0x8080,

    0x06FC,0x1bc3,
    0x06FD,0x8080,
    0x06FE,0x8080,

    0x06F8,0x1bca,
    0x06F9,0x8080,
    0x06FA,0x8080,

    0x06F4,0x1bc2,
    0x06F5,0x8080,
    0x06F6,0x8080,

    0x06F0,0x1bc9,
    0x06F1,0x8080,
    0x06F2,0x8080,

    0x06EC,0x1bc1,
    0x06ED,0x8080,
    0x06EE,0x8080,

    0x06E8,0x1bc8,
    0x06E9,0x8080,
    0x06EA,0x8080,

    0x06E4,0x1bc0,
    0x06E5,0x8080,
    0x06E6,0x8080,

    0x06E0,0x1bb8,
    0x06E1,0x8080,
    0x06E2,0x8080,

    0x06DC,0x1bb0,
    0x06DD,0x8080,
    0x06DE,0x8080,

    0x06D8,0x1bA0,
    0x06D9,0x8080,
    0x06DA,0x8080,

    0x06D4,0x1b98,
    0x06D5,0x8080,
    0x06D6,0x8080,

    0x06D0,0x1b90,
    0x06D1,0x8080,
    0x06D2,0x8080,

    0x06CC,0x1b88,
    0x06CD,0x8080,
    0x06CE,0x8080,

    0x06C8,0x1b80,
    0x06C9,0x8080,
    0x06CA,0x8080,

    0x06C4,0x1b80,
    0x06C5,0x8080,
    0x06C6,0x8080,

    0x06C0,0x1b80,
    0x06C1,0x8080,
    0x06C2,0x8080,

#endif
    // btrx gain table,
    0x047E, 0xa4a4,
    0x047F, 0x0004,
    0x047C, 0xa4a4,
    0x047D, 0x0002,
    0x047A, 0xa4a4,
    0x047B, 0x0001,
    0x0478, 0xa4a4,
    0x0479, 0x0000,
#if 0


#else       //update repair
    0x06BC,0x13EB, // idx =1f
    0x06BD,0x8080,
    0x06BE,0x8080,

    0x06B8,0x13E3, // idx =1e
    0x06B9,0x8080,
    0x06BA,0x8080,

    0x06B4,0x13DB,
    0x06B5,0x8080,
    0x06B6,0x8080, // idx =1d

    0x06B0,0x13D3,      //idx =1c
    0x06B1,0x8080,
    0x06B2,0x8080,

    0x06AC,0x13CB,   //idx = 1b
    0x06AD,0x8080,
    0x06AE,0x8080,
    0x06A8,0x13C3,   //idx = 1a
    0x06A9,0x8080,
    0x06AA,0x8080,
    0x06A4,0x13EA,   //idx = 19
    0x06A5,0x8080,
    0x06A6,0x8080,
    0x06A0,0X1B87,//0x13E2, //idx = 18
    0x06A1,0x8080,
    0x06A2,0x8080,
    0x069C,0x13DA,   //idx = 17
    0x069D,0x8080,
    0x069E,0x8080,
    0x0698,0x13D2,   //idx = 16
    0x0699,0x8080,
    0x069A,0x8080,
    0x0694,0x13CA,   //idx = 15
    0x0695,0x8080,
    0x0696,0x8080,
    0x0690,0x13C2,   //idx = 14
    0x0691,0x8080,
    0x0692,0x8080,
    0x068C,0x13E9,   //idx = 13
    0x068D,0x8080,
    0x068E,0x8080,
    0x0688,0x13E1,   //idx = 12
    0x0689,0x8080,
    0x068A,0x8080,
    0x0684,0x13D9,   //idx = 11
    0x0685,0x8080,
    0x0686,0x8080,
    0x0680,0x13D1,   //idx = 10
    0x0681,0x8080,
    0x0682,0x8080,
    0x067C,0x13C9,   //idx = f
    0x067D,0x8080,
    0x067E,0x8080,
    0x0678,0x13C1,   //idx =e
    0x0679,0x8080,
    0x067A,0x8080,
    0x0674,0x13E8,   //idx =d
    0x0675,0x8080,
    0x0676,0x8080,
    0x0670,0x13E0,   //idx = c
    0x0671,0x8080,
    0x0672,0x8080,
    0x066C,0x13D8,   //idx = b
    0x066D,0x8080,
    0x066E,0x8080,
    0x0668,0x13D0,   //idx =a
    0x0669,0x8080,
    0x066A,0x8080,
    0x0664,0x13C8,   //idx =9
    0x0665,0x8080,
    0x0666,0x8080,
    0x0660,0x13C0,//0x13C0,        //idx =8
    0x0661,0x8080,
    0x0662,0x8080,
    0x065C,0x13C0,    //idx =7
    0x065D,0x8080,
    0x065E,0x8080,
    0x0658,0x13C0,    //idx =6
    0x0659,0x8080,
    0x065A,0x8080,
    0x0654,0x13C0,    //idx =5
    0x0655,0x8080,
    0x0656,0x8080,
    0x0650,0x13C0,    //idx =4
    0x0651,0x8080,
    0x0652,0x8080,
    0x064C,0x13C0,    //idx =3
    0x064D,0x8080,
    0x064E,0x8080,
    0x0648,0x13C0,    //idx =2
    0x0649,0x8080,
    0x064A,0x8080,
    0x0644,0x13C0,  //idx =1
    0x0645,0x8080,
    0x0646,0x8080,
    0x0640,0x13C0,  //idx =0
    0x0641,0x8080,
    0x0642,0x8080,

#endif
    // fmrx gain table,
    0x0626, 0x0024,
    0x0627, 0x8080,
    0x0624, 0x001C,
    0x0625, 0x8080,
    0x0622, 0x001C,
    0x0623, 0x8080,
    0x0620, 0x0014,
    0x0621, 0x8080,
    0x061E, 0x0014,
    0x061F, 0x8080,
    0x061C, 0x000C,
    0x061D, 0x8080,
    0x061A, 0x000C,
    0x061B, 0x8080,
    0x0618, 0x0004,
    0x0619, 0x8080,
    0x0616, 0x0003,
    0x0617, 0x8080,
    0x0614, 0x0053,
    0x0615, 0x8080,
    0x0612, 0x004B,
    0x0613, 0x8080,
    0x0610, 0x004A,
    0x0611, 0x8080,
    0x060E, 0x0042,
    0x060F, 0x8080,
    0x060C, 0x0041,
    0x060D, 0x8080,
    0x060A, 0x0040,
    0x060B, 0x8080,
    0x0608, 0x0040,
    0x0609, 0x8080,
    0x0606, 0x0040,
    0x0607, 0x8080,
    0x0604, 0x0040,
    0x0605, 0x8080,
    0x0602, 0x0040,
    0x0603, 0x8080,
    0x0600, 0x0040,
    0x0601, 0x8080,
    // bttx gain table & config,

//btTx PA PAD config
#ifndef TROUT2_CHIP_VER_V2
//bttxPaTbl
    0x488,   0xb761,   // PaHH
    0x489,   0x779d,
#else
    0x0488, 0xb761,// PaHH  //gdy@0524
    0x0489, 0x7fd8,
#endif

    0x48a,   0xb751,  // PaMM
    0x48b,   0x578d,
    0x48c,   0x9731,  // PaLL
    0x48d,   0x376d,

//bttxPadTbl
    0x48e,   0x4758,    //PAD_HH
    0x48f,   0x5AA1,

    0x490,   0x3758,   //PAD_MM
    0x491,   0x3A41,
    0x492,   0x3738,   //PAD_LL
    0x493,   0x2a21,

//btTxGainTbl
    0x075F, 0x0235,   //idx =31
    0x075E, 0x0235,   //
    0x075D, 0x0235,
    0x075C, 0x0634,
    0x075B, 0x0634,
    0x075A, 0x0634,
    0x0759, 0x0633,
    0x0758, 0x0633,  //idx =24

#ifndef TROUT2_CHIP_VER_V2
    0x74E,  0x064D, //idx=14    0x74E,  0x0645, //idx=14 txGain=4
    0x74D,  0x065D, //idx=13    0x74D,  0x064D, //idx=13 txGain=-18
    0x74C,  0x0534, //idx=12    0x74C,  0x065D, //idx=12 txGain=-14
    0x74B,  0x0544, //idx=11    0x74B,  0x0534, //idx=11 txGain=-11
    0x74A,  0x053D, //idx=10    0x74A,  0x0544, //idx=10 txGain=-7
    0x749,  0x054D, //idx=9     0x749,  0x0535, //idx=9  txGain=-5
    0x748,  0x04B5, //idx=8     0x748,  0x054D, //idx=8  txGain=0
    0x747,  0x04C5, //idx=7     0x747,  0x04B5, //idx=7  txGain=5
    0x746,  0x04D5, //idx=6     0x746,  0x04C5, //idx=6  txGain=10
    0x745,  0x04DD, //idx=5     0x745,  0x0515, //idx=5  txGain=-8
    0x744,  0x055D, //idx=4     0x744,  0x0525, //idx=4  txGain=-4
    0x743,  0x065D, //idx=3     0x743,  0x0625, //idx=3  txGain=-4
    0x742,  0x01D5, //idx=2     0x742,  0x0635, //idx=2  txGain=0
    0x741,  0x0555, //idx=1     0x741,  0x0645, //idx=1  txGain=4
    0x740,  0x0655, //idx=0     0x740,  0x0665, //idx=0  txGain=12
#else
// bttx power 0528
    0x0740,0x04DD, // idx=0   power=30
    0x0741,0x04CD, // idx=1   power=27
    0x0742,0x04BD, // idx=2   power=23
    0x0743,0x055D, // idx=3   power=19

    0x0744,0x04E6, // idx=4    =18  2013-05-28
    0x0745,0x04E5, // idx=5    =13  2013-05-28
    0x0746,0x04DD, // idx=6    =10  2013-05-28
    0x0747,0x04CD, // idx=7    =7   2013-05-28
    0x0748,0x04BD, // idx=8    =3   2013-05-28
    0x0749,0x055D, // idx=9    =-1  2013-05-28
    0x074A,0x054D, // idx=10   =-4.5  2013-05-28
    0x074B,0x0554, // idx=11   =-8   2013-05-28
    0x074C,0x0544, // idx=12   =-13  2013-05-28
    0x074D,0x054B, // idx=13   =-17  2013-05-28
    0x074E,0x0655, // idx=14   =-21  2013-05-28
    0x074F,0x0645, // idx=15   =-25  2013-05-28
    0x0750,0x0635, // idx=16   =-28  2013-05-28
    0x0751,0x063C, // idx=17   =-32  2013-05-28
    0x0752,0x0643, // idx=18   =-36  2013-05-28
    0x0753,0x0633, // idx=19   =-40  2013-05-28
    0x0754,0x062B, // idx=20   =-44  2013-05-28
    0x0755,0x0632, // idx=21   =-48  2013-05-28
    0x0756,0x0641, // idx=22   =-52  2013-05-28
    0x0757,0x0631, // idx=23   =-56  2013-05-28
#endif
    // wftx gain table config,
    // pa band cfg,
#ifndef TROUT2_CHIP_VER_V2
//#if    0
    0x0494, 0xcd89,
    0x0495, 0x7fd2,
    0x0496, 0xcd89,
    0x0497, 0x7fd2,
    0x0498, 0xcd89,
    0x0499, 0x7fd2,
    0x049A, 0xcd89,
    0x049B, 0x7fd2,
    0x049C, 0xcd89,
    0x049D, 0x7fd2,
    0x049E, 0xcd89,
    0x049F, 0x7fd2,
    0x04A0, 0xcd89,
    0x04A1, 0x7fd2,
    0x04A2, 0xcd89,
    0x04A3, 0x7fd2,
    0x04A4, 0xcd89,
    0x04A5, 0x7fd2,
    0x04A6, 0xcd89,
    0x04A7, 0x7fd2,
    0x04A8, 0xcd89,
    0x04A9, 0x7fd2,
    0x04AA, 0xcd89,
    0x04AB, 0x7fd2,
    0x04AC, 0xcd89,
    0x04AD, 0x7fd2,
    0x04AE, 0xcd89,
    0x04AF, 0x7fd2,
    0x04B0, 0xcd89,
    0x04B1, 0x7fd2,
    0x04B2, 0xcd89,
    0x04B3, 0x7fd2,
    0x04B4, 0x4758,
    // pad band cfg,
    0x04B5, 0x5aa1,
    0x04B6, 0x4758,
    0x04B7, 0x5aa1,
    0x04B8, 0x4758,
    0x04B9, 0x5aa1,
    0x04BA, 0x4758,
    0x04BB, 0x5aa1,
    0x04BC, 0x4758,
    0x04BD, 0x5aa1,
    0x04BE, 0x4758,
    0x04BF, 0x5aa1,
    0x04C0, 0x4758,
    0x04C1, 0x5aa1,
    0x04C2, 0x4758,
    0x04C3, 0x5aa1,
    0x04C4, 0x4758,
    0x04C5, 0x5aa1,
    0x04C6, 0x4758,
    0x04C7, 0x5aa1,
    0x04C8, 0x4758,
    0x04C9, 0x5aa1,
    0x04CA, 0x4758,
    0x04CB, 0x5aa1,
    0x04CC, 0x4758,
    0x04CD, 0x5aa1,
    0x04CE, 0x4758,
    0x04CF, 0x5aa1,
    0x04D0, 0x4758,
    0x04D1, 0x5aa1,
    0x04D2, 0x4758,
    0x04D3, 0x5aa1,
    0x04DC, 0x8080,
    0x04DD, 0x8080,
    0x04DE, 0x0008,
    0x04DF, 0x0008,
    0x04E0, 0x0008,
    0x04E1, 0x0008,
    0x04E2, 0x0008,
    0x04E3, 0x0008,
    0x04E4, 0x0008,
    0x04E5, 0x0008,
    0x04E6, 0x0008,
    0x04E7, 0x0008,
    0x04E8, 0x0008,
    0x04E9, 0x0008,
    0x04EA, 0x0008,
    0x04EB, 0x0008,
    0x04EC, 0x0008,
    0x04ED, 0x0008,
    // tx gain table,
    0x07DE, 0x007D,
    0x07DF, 0x0001,
    0x07DC, 0x007D,
    0x07DD, 0x0001,
    0x07DA, 0x007D,
    0x07DB, 0x0001,
    0x07D8, 0x007D,
    0x07D9, 0x0001,
    0x07D6, 0x007D,
    0x07D7, 0x0001,
    0x07D4, 0x007D,
    0x07D5, 0x0001,
    0x07D2, 0x007D,
    0x07D3, 0x0001,
    0x07D0, 0x007D,
    0x07D1, 0x0001,
    0x07CE, 0x007D,
    0x07CF, 0x0001,
    0x07CC, 0x007D,
    0x07CD, 0x0001,
    0x07CA, 0x007D,
    0x07CB, 0x0001,
    0x07C8, 0x007D,
    0x07C9, 0x0001,
    0x07C6, 0x007D,
    0x07C7, 0x0001,
    0x07C4, 0x007D,
    0x07C5, 0x0001,
    0x07C2, 0x007D,
    0x07C3, 0x0001,
    0x07C0, 0x007D,
    0x07C1, 0x0001,
    0x07BE, 0x0005,
    0x07BF, 0x0012,
    0x07BC, 0x0005,
    0x07BD, 0x0012,
    0x07BA, 0x0005,
    0x07BB, 0x0012,
    0x07B8, 0x0005,
    0x07B9, 0x0012,
    0x07B6, 0x0005,
    0x07B7, 0x0012,
    0x07B4, 0x0005,
    0x07B5, 0x0012,
    0x07B2, 0x0005,
    0x07B3, 0x0012,
    0x07B0, 0x0005,
    0x07B1, 0x0012,
    0x07AE, 0x0005,
    0x07AF, 0x0012,
    0x07AC, 0x0005,
    0x07AD, 0x0012,
    0x07AA, 0x0005,
    0x07AB, 0x0012,
    0x07A8, 0x0005,
    0x07A9, 0x0012,
    0x07A6, 0x0005,
    0x07A7, 0x0012,
    0x07A4, 0x0005,
    0x07A5, 0x0012,
    0x07A2, 0x0005,
    0x07A3, 0x0012,
    0x07A0, 0x0005,
    0x07A1, 0x0012,
    0x079E, 0x001D,
    0x079F, 0x0024,
    0x079C, 0x001D,
    0x079D, 0x0024,
    0x079A, 0x001D,
    0x079B, 0x0024,
    0x0798, 0x001D,
    0x0799, 0x0024,
    0x0796, 0x001D,
    0x0797, 0x0024,
    0x0794, 0x001D,
    0x0795, 0x0024,
    0x0792, 0x001D,
    0x0793, 0x0024,
    0x0790, 0x001D,
    0x0791, 0x0024,
    0x078E, 0x001D,
    0x078F, 0x0024,
    0x078C, 0x001D,
    0x078D, 0x0024,
    0x078A, 0x001D,
    0x078B, 0x0024,
    0x0788, 0x001D,
    0x0789, 0x0024,
    0x0786, 0x001D,
    0x0787, 0x0024,
    0x0784, 0x001D,
    0x0785, 0x0024,
    0x0782, 0x001D,
    0x0783, 0x0024,
    0x0780, 0x001D,
    0x0781, 0x0024,
    0x077E, 0x0004,
    0x077F, 0x0038,
    0x077C, 0x0004,
    0x077D, 0x0038,
    0x077A, 0x0004,
    0x077B, 0x0038,
    0x0778, 0x0004,
    0x0779, 0x0038,
    0x0776, 0x0004,
    0x0777, 0x0038,
    0x0774, 0x0004,
    0x0775, 0x0038,
    0x0772, 0x0004,
    0x0773, 0x0038,
    0x0770, 0x0004,
    0x0771, 0x0038,
    0x076E, 0x0004,
    0x076F, 0x0038,
    0x076C, 0x0004,
    0x076D, 0x0038,
    0x076A, 0x0004,
    0x076B, 0x0038,
    0x0768, 0x0004,
    0x0769, 0x0038,
    0x0766, 0x0004,
    0x0767, 0x0038,
    0x0764, 0x0004,
    0x0765, 0x0038,
    0x0762, 0x0004,
    0x0763, 0x0038,
    0x0760, 0x0004,
    0x0761, 0x0038,
#else
// chip V2x
    0x0494, 0x3371,
    0x0495, 0xe780,
    0x0496, 0x7800,
    0x0497, 0x6f80,
    0x0498, 0xb761,//gdy@0524   0x0498, 0x87e7,
    0x0499, 0x7fd8,
    0x049A, 0x7800,
    0x049B, 0x6fa0,
    0x049C, 0x3371,
    0x049D, 0xe780,
    0x049E, 0x7800,
    0x049F, 0x6f80,
    0x04A0, 0xb761,//gdy@0524 0x04A0, 0x87e7,
    0x04A1, 0x7fd8,
    0x04A2, 0x7800,
    0x04A3, 0x6fa0,
    0x04A4, 0x3371,
    0x04A5, 0xe780,
    0x04A6, 0x7800,
    0x04A7, 0x6f80,
    0x04A8, 0xb761,//gdy@0524 0x04A8, 0x87e7,
    0x04A9, 0x7fd8,
    0x04AA, 0x7800,
    0x04AB, 0x6fa0,
    0x04AC, 0x3371,
    0x04AD, 0xe780,
    0x04AE, 0x7800,
    0x04AF, 0x6f80,
    0x04B0, 0xb761,//gdy@0524 0x04B0, 0x87e7,
    0x04B1, 0x7fd8,
    0x04B2, 0x7800,
    0x04B3, 0x6fa0,
    0x04B4, 0x4748,
    0x04B5, 0x5aa1,
    0x04B6, 0x2700,
    0x04B7, 0x2a91,
    0x04B8, 0x4758,
    0x04B9, 0x5aa1,
    0x04BA, 0x2700,
    0x04BB, 0x2a91,
    0x04BC, 0x4748,
    0x04BD, 0x5aa1,
    0x04BE, 0x2700,
    0x04BF, 0x2a91,
    0x04C0, 0x4758,
    0x04C1, 0x5aa1,
    0x04C2, 0x2700,
    0x04C3, 0x2a91,
    0x04C4, 0x4748,
    0x04C5, 0x5aa1,
    0x04C6, 0x2700,
    0x04C7, 0x2a91,
    0x04C8, 0x4758,
    0x04C9, 0x5aa1,
    0x04CA, 0x2700,
    0x04CB, 0x2a91,
    0x04CC, 0x4748,
    0x04CD, 0x5aa1,
    0x04CE, 0x2700,
    0x04CF, 0x2a91,
    0x04D0, 0x4758,
    0x04D1, 0x5aa1,
    0x04D2, 0x2700,
    0x04D3, 0x2a91,
    0x04DC, 0x8080,
    0x04DD, 0x8080,
    0x04DE, 0x0008,
    0x04DF, 0x0008,
    0x04E0, 0x0008,
    0x04E1, 0x0008,
    0x04E2, 0x0008,
    0x04E3, 0x0008,
    0x04E4, 0x0008,
    0x04E5, 0x0008,
    0x04E6, 0x0008,
    0x04E7, 0x0008,
    0x04E8, 0x0008,
    0x04E9, 0x0008,
    0x04EA, 0x0008,
    0x04EB, 0x0008,
    0x04EC, 0x0008,
    0x04ED, 0x0008,



    0x07DE, 0x0035,
    0x07DF, 0x0041,
    0x07DC, 0x0035,
    0x07DD, 0x0041,
    0x07DA, 0x0035,
    0x07DB, 0x0041,
    0x07D8, 0x0035,
    0x07D9, 0x0041,
    0x07D6, 0x0035,
    0x07D7, 0x0041,
    0x07D4, 0x0035,
    0x07D5, 0x0041,
    0x07D2, 0x0035,
    0x07D3, 0x0041,
    0x07D0, 0x0035,
    0x07D1, 0x0041,
    0x07CE, 0x0035,
    0x07CF, 0x0041,
    0x07CC, 0x0035,
    0x07CD, 0x0041,
    0x07CA, 0x0035,
    0x07CB, 0x0041,
    0x07C8, 0x0035,
    0x07C9, 0x0041,
    0x07C6, 0x0035,
    0x07C7, 0x0041,
    0x07C4, 0x0035,
    0x07C5, 0x0041,
    0x07C2, 0x0035,
    0x07C3, 0x0041,
    0x07C0, 0x0035,
    0x07C1, 0x0041,
    0x07BE, 0x0074,
    0x07BF, 0x0052,
    0x07BC, 0x0074,
    0x07BD, 0x0052,
    0x07BA, 0x0074,
    0x07BB, 0x0052,
    0x07B8, 0x0074,
    0x07B9, 0x0052,
    0x07B6, 0x0074,
    0x07B7, 0x0052,
    0x07B4, 0x0074,
    0x07B5, 0x0052,
    0x07B2, 0x0074,
    0x07B3, 0x0052,
    0x07B0, 0x0074,
    0x07B1, 0x0052,
    0x07AE, 0x0074,
    0x07AF, 0x0052,
    0x07AC, 0x0074,
    0x07AD, 0x0052,
    0x07AA, 0x0074,
    0x07AB, 0x0052,
    0x07A8, 0x0074,
    0x07A9, 0x0052,
    0x07A6, 0x0074,
    0x07A7, 0x0052,
    0x07A4, 0x0074,
    0x07A5, 0x0052,
    0x07A2, 0x0074,
    0x07A3, 0x0052,
    0x07A0, 0x0074,
    0x07A1, 0x0052,
    0x079E, 0x0035,
    0x079F, 0x0064,
    0x079C, 0x0035,
    0x079D, 0x0064,
    0x079A, 0x0035,
    0x079B, 0x0064,
    0x0798, 0x0035,
    0x0799, 0x0064,
    0x0796, 0x0035,
    0x0797, 0x0064,
    0x0794, 0x0035,
    0x0795, 0x0064,
    0x0792, 0x0035,
    0x0793, 0x0064,
    0x0790, 0x0035,
    0x0791, 0x0064,
    0x078E, 0x0035,
    0x078F, 0x0064,
    0x078C, 0x0035,
    0x078D, 0x0064,
    0x078A, 0x0035,
    0x078B, 0x0064,
    0x0788, 0x0035,
    0x0789, 0x0064,
    0x0786, 0x0035,
    0x0787, 0x0064,
    0x0784, 0x0035,
    0x0785, 0x0064,
    0x0782, 0x0035,
    0x0783, 0x0064,
    0x0780, 0x0035,
    0x0781, 0x0064,
    0x077E, 0x0074,
    0x077F, 0x0078,
    0x077C, 0x0074,
    0x077D, 0x0078,
    0x077A, 0x0074,
    0x077B, 0x0078,
    0x0778, 0x0074,
    0x0779, 0x0078,
    0x0776, 0x0074,
    0x0777, 0x0078,
    0x0774, 0x0074,
    0x0775, 0x0078,
    0x0772, 0x0074,
    0x0773, 0x0078,
    0x0770, 0x0074,
    0x0771, 0x0078,
    0x076E, 0x0074,
    0x076F, 0x0078,
    0x076C, 0x0074,
    0x076D, 0x0078,
    0x076A, 0x0074,
    0x076B, 0x0078,
    0x0768, 0x0074,
    0x0769, 0x0078,
    0x0766, 0x0074,
    0x0767, 0x0078,
    0x0764, 0x0074,
    0x0765, 0x0078,
    0x0762, 0x0074,
    0x0763, 0x0078,
    0x0760, 0x0055,   //dayu@0524 update
    0x0761, 0x0041,  //dayu@0524 update
#endif
};

MXD_U16 gCastorRegInitForWifiRxLna[]=
{
    //0628 for lna8205 //0527 debug for mobile Plan2   // xiao mei 0524plan1       //    old
    0x0447, 0x7F7F,
    0x0446, 0x7F7E,  //0619 PM     //0x0446, 0x7E7D,    USED
    0x0445, 0x7D7C,
    0x0444, 0x7B7A,
    0x0443, 0x7978,
    0x0442, 0x7776,
    0x0441, 0x7574,
    0x0440, 0x7372,
    0x043F, 0x7110,
    0x043E, 0x0F0E,
    0x043D, 0x0D0C,
    0x043C, 0x0B0A,
    0x043B, 0x0908,
    0x043A, 0x0706,
    0x0439, 0x0504,
    0x0438, 0x0302,

    // wf rxGainIdx
    0x073C,0x1BD7, //wf Gain idx= 31
    0x0738,0x1BCF, //wf Gain idx= 30
    0x0734,0x1BD3, //wf Gain idx= 29
    0x0730,0x1BCB, //wf Gain idx= 28
    0x072C,0x1BD2, //wf Gain idx= 27
    0x0728,0x1BCA, //wf Gain idx= 26
    0x0724,0x1BD1, //wf Gain idx= 25
    0x0720,0x1BC9, //wf Gain idx= 24
    0x071C,0x1BD0, //wf Gain idx= 23
    0x0718,0x1BC8, //wf Gain idx= 22
    0x0714,0x1BC0, //wf Gain idx= 21
    0x0710,0x1BB8, //wf Gain idx= 20
    0x070C,0x1BB0, //wf Gain idx= 19
    0x0708,0x1BA8, //wf Gain idx= 18
    0x0704,0x1BA0, //wf Gain idx= 17
    0x0700,0x1bcb, //wf Gain idx= 16
    0x06FC,0x1bc3, //wf Gain idx= 15
    0x06F8,0x1bca, //wf Gain idx= 14
    0x06F4,0x1bc2, //wf Gain idx= 13
    0x06F0,0x1bc9, //wf Gain idx= 12
    0x06EC,0x1bc1, //wf Gain idx= 11
    0x06E8,0x1bc8, //wf Gain idx= 10
    0x06E4,0x1bc0, //wf Gain idx= 9
    0x06E0,0x1bb8, //wf Gain idx= 8
    0x06DC,0x1bb0, //wf Gain idx= 7
#if 0        /*zl@1021 change gain*/

   #else    /*zl@1021*/
       0x06D8,0x1ba8, //wf Gain idx= 6
       0x06D4,0x1bA0, //wf Gain idx= 5
       0x06D0,0x1b98, //wf Gain idx= 4
       0x06CC,0x1b90, //wf Gain idx= 3
       0x06C8,0x1b88, //wf Gain idx= 2
    0x06C4,0x1b80, //wf Gain idx= 1
       0x06C0,0x1b80, //wf Gain idx= 0
   #endif
   
};


MXD_U16 gCastorRegInitLna_None[]=
{
    // wf rx gain map 0~127 map,  0x438~477 now use 0~31, zl@0626
    //0527 debug for mobile Plan2   // xiao mei 0524plan1
    0x0447, 0x7F7F,                     //0x0447, 0x7F7F,                 //0x0447, 0x7f7f,      //    0x0447, 0x7F7F,
    0x0446, 0x7E7D,                     //0x0446, 0x7E7D,                 //0x0446, 0x7f7e,      //    0x0446, 0x7e7d,
    0x0445, 0x7D7C,                     //0x0445, 0x7D7C,                 //0x0445, 0x7d7c,      //    0x0445, 0x7D7C,
    0x0444, 0x7B7A,                     //0x0444, 0x7B7A,                 //0x0444, 0x7B7A,      //    0x0444, 0x7B7A,
    0x0443, 0x7978,                     //0x0443, 0x7978,                 //0x0443, 0x7978,      //    0x0443, 0x7978,
    0x0442, 0x7776,                     //0x0442, 0x7776,                 //0x0442, 0x7776,      //    0x0442, 0x7776,
    0x0441, 0x7574,                     //0x0441, 0x7554,                 //0x0441, 0x7558,      //    0x0441, 0x7554,
    0x0440, 0x7372,                     //0x0440, 0x5352,                 //0x0440, 0x5756,      //    0x0440, 0x5352,
    0x043F, 0x7110,                     //0x043F, 0x5150,                 //0x043F, 0x5536,      //    0x043F, 0x5251,
    0x043E, 0x0F0E,                     //0x043E, 0x4F4E,                 //0x043E, 0x5352,      //    0x043E, 0x504F,
    0x043D, 0x0D0C,                     //0x043D, 0x4D4C,                 //0x043D, 0x5150,      //    0x043D, 0x4E4D,
    0x043C, 0x0B0A,                     //0x043C, 0x0B0A,                 //0x043C, 0x4f4e,      //    0x043C, 0x4C0B,
    0x043B, 0x0908,                     //0x043B, 0x0908,                 //0x043B, 0x4d09,      //    0x043B, 0x0A09,
    0x043A, 0x0706,                     //0x043A, 0x0706,                 //0x043A, 0x0810,      //    0x043A, 0x0807,
    0x0439, 0x0504,                     //0x0439, 0x0504,                 //0x0439, 0x0f0e,      //    0x0439, 0x0605,
    0x0438, 0x0302,                     //0x0438, 0x0302,                 //0x0438, 0x0d0d,      //    0x0438, 0x0403,
};

// For wfTxGain init
MXD_U16 gCastorRegInitForWifiTxGain[]=
{
//pa band0 (494-49B), cfg
    0x0494, 0xf9f9,//0x89b9, 11b mask
    0x0495, 0xd7e8,//0xfcd8,11b mask
    0x0496, 0x7800,//@dayu0801, 0x2949,
    0x0497, 0x6fa0,//@dayu0801, 0x8c68
    0x0498, 0x89b9,
    0x0499, 0xfcd8,
    0x049A, 0x2949,
    0x049B, 0x8c68,
//pa band1 (49c-4A3), cfg
    0x049C,  0xf9f9,//0x89b9,11b mask
    0x049D,  0xd7e8,//0xfcd8,11b mask
    0x049E,  0x7800,//dayu0801 0x2949
    0x049F,  0x6fa0,//dayu0801 0x8c68
    0x04A0,  0x89b9,
    0x04A1,  0xfcd8,
    0x04A2,  0x2949,
    0x04A3,  0x8c68,
//pa band2 (4A4-4AB), cfg
    0x04A4, 0xf9f9,//0x89b9,11b mask
    0x04A5, 0xd7e8,//0xfcd8,11b mask
    0x04A6, 0x7800,//dayu0801 0x2949
    0x04A7, 0x6fa0,//dayu0801 0x8c68
    0x04A8, 0x89b9,
    0x04A9, 0xfcd8,
    0x04AA, 0x2949,
    0x04AB, 0x8c68,
//pa band3 (4AC-4B3), cfg
    0x04AC, 0xf9f9,//0x89b9,11b mask
    0x04AD, 0xd7e8,//0xfcd8,11b mask
    0x04AE, 0x7800,//dayu0801 0x2949
    0x04AF, 0x6fa0,//dayu0801 0x8c68
    0x04B0, 0x89b9,
    0x04B1, 0xfcd8,
    0x04B2, 0x2949,
    0x04B3, 0x8c68,

//pad cfg  - band0 : 4b4~4bb
    0x04B4, 0x4758,//0x4748,11b mask
    0x04B5, 0x5aa1,
    0x04B6, 0x2700,//@dayu0801 0x7348
    0x04B7, 0x2a91,//@dayu0801 0x3858
    0x04B8, 0x4758,
    0x04B9, 0x5aa1,
    0x04BA, 0x7348,
    0x04BB, 0x3858,
//pad cfg -band1 : 4bc~4c3
    0x04BC, 0x4758,//0x4748,11b mask
    0x04BD, 0x5aa1,
    0x04BE, 0x2700,//@dayu0801 0x7348
    0x04BF, 0x2a91,//@dayu0801 0x3858
    0x04C0, 0x4758,
    0x04C1, 0x5aa1,
    0x04C2, 0x7348,
    0x04C3, 0x3858,
//pad cfg -band2 :4c4~4cb
    0x04C4, 0x4758,//0x4748,11b mask
    0x04C5, 0x5aa1,
    0x04C6, 0x2700,//@dayu0801 0x7348
    0x04C7, 0x2a91,//@dayu0801 0x3858
    0x04C8, 0x4758,
    0x04C9, 0x5aa1,
    0x04CA, 0x7348,
    0x04CB, 0x3858,
//pad cfg -band3: 4cc~4d3
    0x04CC, 0x4758,//0x4748,11b mask
    0x04CD, 0x5aa1,
    0x04CE, 0x2700,//@dayu0801 0x7348
    0x04CF, 0x2a91,//@dayu0801 0x3858
    0x04D0, 0x4758,
    0x04D1, 0x5aa1,
    0x04D2, 0x7348,
    0x04D3, 0x3858,

    0x04DC, 0x8080,
    0x04DD, 0x8080,
    0x04DE, 0x0008,
    0x04DF, 0x0008,
    0x04E0, 0x0008,
    0x04E1, 0x0008,
    0x04E2, 0x0008,
    0x04E3, 0x0008,
    0x04E4, 0x0008,
    0x04E5, 0x0008,
    0x04E6, 0x0008,
    0x04E7, 0x0008,
    0x04E8, 0x0008,
    0x04E9, 0x0008,
    0x04EA, 0x0008,
    0x04EB, 0x0008,
    0x04EC, 0x0008,
    0x04ED, 0x0008,

//2013-07-31   for 11b,cfg phybank1_71   Pa_In_Chip     prw
// 2013 -0912  for new table
    0x0760, 0x0074, 0x0761, 0x0004,  // wfTxIdx= 0   11.5
    0x0762, 0x0023, 0x0763, 0x0041,  // wfTxIdx= 1   5
    0x0764, 0x0033, 0x0765, 0x0041,  // wfTxIdx= 2   -
    0x0766, 0x0043, 0x0767, 0x0041,  // wfTxIdx= 3   -
    0x0768, 0x0053, 0x0769, 0x0041,  // wfTxIdx= 4   -
    0x076A, 0x0004, 0x076B, 0x0041,  // wfTxIdx= 5   -
    0x076C, 0x0014, 0x076D, 0x0041,  // wfTxIdx= 6   -
    0x076E, 0x0024, 0x076F, 0x0041,  // wfTxIdx= 7   -
    0x0770, 0x0034, 0x0771, 0x0041,  // wfTxIdx= 8   -
    0x0772, 0x0044, 0x0773, 0x0041,  // wfTxIdx= 9   -
    0x0774, 0x0054, 0x0775, 0x0041,  // wfTxIdx= A   -
    0x0776, 0x0005, 0x0777, 0x0041,  // wfTxIdx= B   -
    0x0778, 0x0015, 0x0779, 0x0041,  // wfTxIdx= C   -
    0x077A, 0x0025, 0x077B, 0x0041,  // wfTxIdx= D   -
    0x077C, 0x0035, 0x077D, 0x0041,  // wfTxIdx= E   -
    0x077E, 0x0045, 0x077F, 0x0041,  // wfTxIdx= F   17

//2013-07-04 // for 11b, cfg phybank1_71 ext PA        prw
    0x0780, 0x0042, 0x0781, 0x0048,  // wfTxIdx= 10  -13
    0x0782, 0x0052, 0x0783, 0x0048,  // wfTxIdx= 11  -
    0x0784, 0x0062, 0x0785, 0x0048,  // wfTxIdx= 12  -
    0x0786, 0x0072, 0x0787, 0x0048,  // wfTxIdx= 13  -
    0x0788, 0x0003, 0x0789, 0x0048,  // wfTxIdx= 14  -
    0x078A, 0x0013, 0x078B, 0x0048,  // wfTxIdx= 15  -
    0x078C, 0x0023, 0x078D, 0x0048,  // wfTxIdx= 16  -
    0x078E, 0x0033, 0x078F, 0x0048,  // wfTxIdx= 17  -
    0x0790, 0x0043, 0x0791, 0x0048,  // wfTxIdx= 18  -
    0x0792, 0x0053, 0x0793, 0x0048,  // wfTxIdx= 19  -
    0x0794, 0x0063, 0x0795, 0x0048,  // wfTxIdx= 1A  -
    0x0796, 0x0073, 0x0797, 0x0048,  // wfTxIdx= 1B  -
    0x0798, 0x0024, 0x0799, 0x0048,  // wfTxIdx= 1C  -
    0x079A, 0x0034, 0x079B, 0x0048,  // wfTxIdx= 1D  -
    0x079C, 0x0044, 0x079D, 0x0048,  // wfTxIdx= 1E  -
    0x079E, 0x0054, 0x079F, 0x0048,  // wfTxIdx= 1F** 1

//2013-07-31   for 11g,cfg phybank1_70    Pa_In_Chip   prw
    0x07A0, 0x0042, 0x07A1, 0x0041,  // wfTxIdx= 20  0.5
    0x07A2, 0x0052, 0x07A3, 0x0041,  // wfTxIdx= 21  -
    0x07A4, 0x0062, 0x07A5, 0x0041,  // wfTxIdx= 22  -
    0x07A6, 0x0072, 0x07A7, 0x0041,  // wfTxIdx= 23  -
    0x07A8, 0x0003, 0x07A9, 0x0041,  // wfTxIdx= 24  -
    0x07AA, 0x0013, 0x07AB, 0x0041,  // wfTxIdx= 25  -
    0x07AC, 0x0023, 0x07AD, 0x0041,  // wfTxIdx= 26  -
    0x07AE, 0x0033, 0x07AF, 0x0041,  // wfTxIdx= 27  -
    0x07B0, 0x0043, 0x07B1, 0x0041,  // wfTxIdx= 28  -
    0x07B2, 0x0053, 0x07B3, 0x0041,  // wfTxIdx= 29  -
    0x07B4, 0x0004, 0x07B5, 0x0041,  // wfTxIdx= 2A  -
    0x07B6, 0x0014, 0x07B7, 0x0041,  // wfTxIdx= 2B  -
    0x07B8, 0x0024, 0x07B9, 0x0041,  // wfTxIdx= 2C  -
    0x07BA, 0x0034, 0x07BB, 0x0041,  // wfTxIdx= 2D  -
    0x07BC, 0x0044, 0x07BD, 0x0041,  // wfTxIdx= 2E  -
    0x07BE, 0x0054, 0x07BF, 0x0041,  // wfTxIdx= 2F  10

// for 11g,cfg phybank1_70 ext PA                       prw
    0x07C0, 0x0042, 0x07C1, 0x0042,  // wfTxIdx= 30  -16
    0x07C2, 0x0052, 0x07C3, 0x0042,  // wfTxIdx= 31  -
    0x07C4, 0x0062, 0x07C5, 0x0042,  // wfTxIdx= 32  -
    0x07C6, 0x0072, 0x07C7, 0x0042,  // wfTxIdx= 33  -
    0x07C8, 0x0003, 0x07C9, 0x0042,  // wfTxIdx= 34  -
    0x07CA, 0x0013, 0x07CB, 0x0042,  // wfTxIdx= 35  -
    0x07CC, 0x0023, 0x07CD, 0x0042,  // wfTxIdx= 36  -
    0x07CE, 0x0033, 0x07CF, 0x0042,  // wfTxIdx= 37  -
    0x07D0, 0x0043, 0x07D1, 0x0042,  // wfTxIdx= 38  -
    0x07D2, 0x0053, 0x07D3, 0x0042,  // wfTxIdx= 39  -
    0x07D4, 0x0004, 0x07D5, 0x0042,  // wfTxIdx= 3A  -
    0x07D6, 0x0014, 0x07D7, 0x0042,  // wfTxIdx= 3B  -
    0x07D8, 0x0024, 0x07D9, 0x0042,  // wfTxIdx= 3C  -
    0x07DA, 0x0034, 0x07DB, 0x0042,  // wfTxIdx= 3D  -
    0x07DC, 0x0044, 0x07DD, 0x0042,  // wfTxIdx= 3E  -
    0x07DE, 0x0054, 0x07DF, 0x0042,  // wfTxIdx= 3F  1
};

MXD_U16 gWifiTxGaincfg[]=
{  // +1db
    0x02, 0x12, 0x22, 0x32, 0x42, 0x52,
    0x62, 0x72, 0x72, 0x72, 0x72, 0x03,
    0x13, 0x23, 0x33, 0x43, 0x53, 0x04, 
    0x14, 0x24, 0x34, 0x44, 0x54, 0x05,
    0x15, 0x25, 0x35, 0x45, 0x55, 0x65,
    0x75,
};
MXD_U16 gCastorRegInitNoPa[]=
{
    0x01,0x01,
};



#endif
/* _CASTOR_REG_H_ */
