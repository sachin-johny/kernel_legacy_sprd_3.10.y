#ifndef WIFI_NVM_DATA_H
#define WIFI_NVM_DATA_H

typedef struct {
	char ref_clock;
	char fem_status;
	char wifi_tx;
	char bt_tx;
	char wifi_rx;
	char bt_rx;
	char wb_lna_bypass;
	char gain_lna;
	char il_wb_lna_bypass;
	char rx_adaptive;
	char up_bypass_switching_point0;
	char low_bypass_switching_point0;
} fem_ref_clk_conf_t;

typedef struct {
	char data_rate_power;
	char channel_num;
	char channel_range[6];
	char _11b_tx_power_dr0[3];
	char _11b_tx_power_dr1[3];
	char _11g_tx_power_dr0[3];
	char _11g_tx_power_dr1[3];
	char _11g_tx_power_dr2[3];
	char _11g_tx_power_dr3[3];
	char _11n_tx_power_dr0[3];
	char _11n_tx_power_dr1[3];
	char _11n_tx_power_dr2[3];
	char _11n_tx_power_dr3[3];
} tx_power_config_t;

typedef struct {
	tx_power_config_t tx_power_config;
	char adaptive_tx_power;
	char adaptive_tx_index_offset0;
	char adaptive_rx_rssi0;
	char adaptive_tx_index_offset1;
	char adaptive_rx_rssi1;
} tx_power_control_t;

typedef struct {
	char phy_init_reg_num;
	unsigned short phy_init_reg_array[10];
	char rf_init_reg_num;
	unsigned int rf_init_reg_array[10];
} init_register_t;

typedef struct {
	char extreme_tx_config;
	char temp_index[11];
	char _11b_tx_power_compensation[11];
	char _11gn_tx_power_compensation[11];
	char extreme_rx_config;
	char rx_rssi_compensation[11];
	char rx_temp_switching_point[2];
	char phy_reg_num;
	unsigned short phy_init_reg_array_1[10];
	unsigned short phy_init_reg_array_2[10];
} extreme_parameter_t;

typedef struct {
	char e_cali_channel[3];
	tx_power_config_t equipment_tx_power_config;
} equipment_cali_para_t;

typedef struct {
	char cali_version;
	char data_init_ok;
	char tpc_enable;
	fem_ref_clk_conf_t fem_ref_clk_conf;
	tx_power_control_t tx_power_control;
	init_register_t init_register;
	extreme_parameter_t extreme_parameter;
	equipment_cali_para_t equipment_cali_para;
} wifi_nvm_data;

#endif
