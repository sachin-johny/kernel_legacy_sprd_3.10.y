#ifdef TROUT_WIFI_NPI

#include <linux/uaccess.h>
#include "trout_wifi_npi.h"
#include "mac_string.h"
#include "iconfig.h"
#include "itm_wifi_iw.h"
#include "mh.h"
#include "npi_interface.h"
#include <linux/semaphore.h>
#include <linux/interrupt.h>
#include "qmu_if.h"

 

extern UWORD32 g_rx_right_start_count;
extern UWORD32 g_rx_error_start_count;

extern UWORD8 g_mac_addr[6];
extern unsigned char * g_itm_config_buf;
UWORD8 g_connect_ok_flag = 0;
extern int g_npi_scan_flag ;
extern UWORD32 pwr_level_b_val;
extern UWORD32 pwr_level_a_val;

extern int g_rx_flag;
extern UWORD32 g_save_reg1;
extern UWORD32 g_save_reg2;
extern UWORD32 g_sys_reg;
extern volatile int g_tx_flag;
extern UWORD8 g_user_tx_rate;
int npi_tx_data_init_flag = 0;
struct workqueue_struct *npi_dwork_wq = NULL;
struct delayed_work npi_dwork;
extern int itm_scan_flag;

extern void tx_packet_test(void);
u16 rssi_to_str(int i, char *string);

#define NPI_DEBUG 

UWORD8 get_tx_mode(UWORD8 tx_rate)
{
	if (tx_rate == 1 || tx_rate == 2 || tx_rate == 5
		|| tx_rate == 11)
	{
		printk("is b\n");
		return TYPE_80211_B;
	}
	else if (tx_rate == 6 || tx_rate == 9 || tx_rate == 18
		|| tx_rate == 24 || tx_rate == 36 || tx_rate == 48
		||tx_rate == 54)
	{
		printk("is g\n");
		return TYPE_80211_G;
	}
	else if (tx_rate == 7  || tx_rate == 13 || tx_rate == 19
		|| tx_rate == 26 || tx_rate == 39 || tx_rate == 52
		||tx_rate == 58 || tx_rate == 65)
	{
		printk("is n\n");
		return TYPE_80211_N;
	}
	else
	{
		printk("nothing\n");
		return TYPE_80211_INVALID;
	}
}

void clr_rx_count_reg(void)
{
	printk("zhuyg: %s\n", __FUNCTION__);
	host_write_trout_reg(0, (UWORD32)rMAC_RX_FRAME_FILTER_COUNTER);
	host_write_trout_reg(0, (UWORD32)rMAC_RX_MAC_HEADER_FILTER_COUNTER);
	host_write_trout_reg(0, (UWORD32)rMAC_RXQ_FULL_COUNTER);
	host_write_trout_reg(0, (UWORD32)rMAC_RX_RAM_PACKET_COUNTER);
	host_write_trout_reg(0, (UWORD32)rMAC_RX_END_COUNT);
	host_write_trout_reg(0, (UWORD32)rMAC_RX_ERROR_END_COUNT);
	host_write_trout_reg(0, (UWORD32)rMAC_DUP_DET_COUNT);
	host_write_trout_reg(0, (UWORD32)rMAC_AMPDU_RXD_COUNT);
	host_write_trout_reg(0, (UWORD32)rMAC_RX_MPDUS_IN_AMPDU_COUNT);
	host_write_trout_reg(0, (UWORD32)rMAC_FCS_FAIL_COUNT);
}

UWORD32 int_to_str(UWORD32 i, char *string)
{
	UWORD32 len = 0;
	UWORD32 power, j;
	j = i;
	for (power=1; j >=10; j /= 10)
		power *= 10;
	for (; power > 0; power /= 10)
	{
		*string++ = '0' + i/power;
		i %= power;
		len++;
	}
	*string = '\0';

	return len;
}
extern int g_npi_tx_pkt_count;
UWORD8 *  create_tx_packet(int pkt_len,UWORD8 * q_num);
/* schedule Tx data  */
void do_npi_tx_data_work(struct work_struct *work)
{
    //int ms_delay = 2;
    //UWORD8 tx_mode = 0;
	//static int i=1;
	UWORD8 q_num = 0;
	UWORD8 *tx_descr = NULL;
	UWORD32 reg_data = 0;
	//static int mode_flag = 0;
	
    set_current_state(TASK_INTERRUPTIBLE);
    do{
		tx_descr  = create_tx_packet(50,&q_num);
		reg_data = host_read_trout_reg((UWORD32)rMAC_TSF_CON);
		if ((reg_data & BIT0) == 0)
		{
			reg_data = reg_data | BIT0;
			host_write_trout_reg(reg_data, (UWORD32)rMAC_TSF_CON);
		}
		if(qmu_add_tx_packet(&g_q_handle.tx_handle, q_num, tx_descr) != QMU_OK)
		{
			free_tx_dscr((UWORD32 *)tx_descr);
		}
        	g_npi_tx_pkt_count++;
    }while(g_tx_flag);
}

void npi_tx_data_init(void)
{

    if(npi_tx_data_init_flag == 0)
    {
        npi_dwork_wq = create_singlethread_workqueue("npi_delaywork_wq");
        if(!npi_dwork_wq){  
            printk(KERN_ERR "error! No memory for npi_delaywork_wq workqueue!\n");  
            return;
        }
    	INIT_DELAYED_WORK(&(npi_dwork), do_npi_tx_data_work);
        npi_tx_data_init_flag = 1;
    }
}
void npi_tx_data_release(void)
{
    if(npi_tx_data_init_flag)
    {
        int ret = 0;
        ret = cancel_delayed_work(&npi_dwork);
        printk("npi %s: cancel_delayed_work ret=%d\n", __FUNCTION__, ret); 
        
        flush_workqueue(npi_dwork_wq);
        destroy_workqueue(npi_dwork_wq);
        npi_tx_data_init_flag = 0;
    }
}

// queue work
void npi_tx_data_qwk(void)
{
    int ret = 0;

    ret = queue_delayed_work(npi_dwork_wq, &npi_dwork, msecs_to_jiffies(30));  // 30 ms
    printk("npi %s: queue_delayed_work ret=%d\n", __FUNCTION__, ret); 

}

// cancel work
void npi_tx_data_cwk(void)
{
    int ret = 0;
    
    ret = cancel_delayed_work(&npi_dwork);
    
    flush_workqueue(npi_dwork_wq);
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

static void set_result_code(PNPI_CMD  cmd,int result)
{
	if(cmd && cmd->result_code){
		if(copy_to_user(cmd->result_code,&result,sizeof(result))){
			printk("[npi]:copy_to_user error\n");
		}
	}
}

static void set_result_buffer(PNPI_CMD  cmd,char * buffer,int len)
{
	if(cmd && cmd->result.buffer){
		copy_to_user(cmd->result.buffer,buffer,len);
	}
	if(cmd && cmd->result_buffer_len){
		copy_to_user(cmd->result_buffer_len,&len,sizeof(len));
	}
}


static int npi_start_rx_data(PNPI_CMD  cmd)
{
	UWORD32 tmp = 0;
	printk("[npi]: %s  \n", __FUNCTION__);
	printk("g_mac_stats.pewrx %u\n", g_mac_stats.pewrx);
	g_npi_scan_flag = 0;
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
	// for cmcc 11n receive test[zhongli 20130925]
	write_dot11_phy_reg(0xFF, 0x00);
    	write_dot11_phy_reg(0xF0, 0x65);
	g_rx_flag = 1;
	set_result_code(cmd,0);
	return 0;
}

static int npi_stop_rx_data(PNPI_CMD  cmd)
{
	printk("[npi]: %s  \n", __FUNCTION__);
	// for cmcc 11n receive test[zhongli 20130925]
	write_dot11_phy_reg(0xFF, 0x00);
    	write_dot11_phy_reg(0xF0, 0x01);
	g_rx_flag = 0;
	set_result_code(cmd,0);
	return 0;
}
static int npi_set_tx_rate(PNPI_CMD  cmd)
{
	printk("[npi]: %s  \n", __FUNCTION__);

	printk("[npi]: rate is: %u\n", cmd->param.int_val);
	set_tx_rate(cmd->param.int_val);

	g_user_tx_rate = cmd->param.int_val;
	set_result_code(cmd,0);
	
	return 0;
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

static int npi_start_tx_data(PNPI_CMD  cmd)
{
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

	printk("[npi]: %s\n", __FUNCTION__);
	
	g_npi_scan_flag = 0;

	restart_mac(&g_mac, 0);
	if (flag == 1){
		g_save_reg1 = host_read_trout_reg((0x4072 <<2));//dft wave len 
		g_save_reg2 = host_read_trout_reg((0x4071 <<2));
		g_sys_reg = host_read_trout_reg((UWORD32)rSYSREG_WIFI_CFG);
		printk("g_save_reg1 :%u\n", g_save_reg1);
		printk("g_save_reg2 :%u\n", g_save_reg2);
		flag = 2;
	}
	if (cmd->param.int_val == 1){
		write_sys_value = g_sys_reg | 1 | (1 << 7) | (1 << 8);
		printk("write value is: %x\n", write_sys_value);
		host_write_trout_ram((void*)CW_TX_BEGIN,(void*)wifi_ram_test,sizeof(wifi_ram_test));
		host_write_trout_reg(write_sys_value, (UWORD32)rSYSREG_WIFI_CFG);
		host_write_trout_reg(0x27,(0x4072 <<2));//dft wave len 
		host_write_trout_reg(1,(0x4071 <<2)); //dft controll enable
	}else{
		printk(" 0\n");
		printk("g_save_reg1 :%u\n", g_save_reg1);
		printk("g_save_reg2 :%u\n", g_save_reg2);
		printk("g_sys_reg :%u\n", g_sys_reg);
		host_write_trout_reg(g_sys_reg, (UWORD32)rSYSREG_WIFI_CFG);
		host_write_trout_reg(g_save_reg1, (0x4072 <<2));
		host_write_trout_reg(g_save_reg2, (0x4071 <<2));
		g_tx_flag = 1;
		tx_packet_test();
		//npi_tx_data_qwk();
	}
	set_result_code(cmd,0);
	return 0;
}

static int npi_stop_tx_data(PNPI_CMD  cmd)
{
	g_tx_flag = 0;
	//npi_tx_data_cwk();
	host_write_trout_reg(g_sys_reg, (UWORD32)rSYSREG_WIFI_CFG);
	host_write_trout_reg(g_save_reg1, (0x4072 <<2));
	host_write_trout_reg(g_save_reg2, (0x4071 <<2));
	set_result_code(cmd,0);
       g_npi_tx_pkt_count = 0;
	//meter_end("npi_tx");
	return 0;
}
static int npi_set_mac(PNPI_CMD  cmd)
{
	UWORD8 mac_addr[6]={0,};
	
	printk("[npi]: %s  \n", __FUNCTION__);
	
	if(cmd && (cmd->param.buffer== NULL || cmd->param_buffer_len <=0)) return -EINVAL;

	if (copy_from_user(mac_addr, cmd->param.buffer, cmd->param_buffer_len)){
		set_result_code(cmd,-1);
    		return 0;
    	}
	
	//hexstr2bin(mac_addr,mac_addr,sizeof(mac_addr));

#ifdef NPI_DEBUG
	printk("set_mac: %x-%x-%x-%x-%x-%x\n", mac_addr[0], mac_addr[1], mac_addr[2],
			mac_addr[3], mac_addr[4], mac_addr[5]);
#endif

	mset_StationID(mac_addr);
	memcpy(g_mac_addr, mac_addr, 6);

	set_result_code(cmd,0);

	return 0;
}


static int npi_set_rx_count(PNPI_CMD  cmd)
{
	UWORD32 fcs_error_count = 0;
	UWORD32 rx_end_count = 0;

	printk("[npi]: %s\n", __FUNCTION__);

	clr_rx_count_reg();

	rx_end_count = get_machw_rx_end_count();

	fcs_error_count = get_fcs_count();
	printk("[npi]: fcs_error_count: %u\n", fcs_error_count);
	printk("[npi]: rx_end count is: %u\n", rx_end_count);
	
	g_rx_right_start_count = rx_end_count;
	g_rx_right_start_count -= fcs_error_count;
	g_rx_error_start_count = get_machw_rx_error_end_count();
	g_rx_error_start_count += fcs_error_count;

	set_result_code(cmd,0);
	
	return 0;
}


static int npi_set_tx_mode(PNPI_CMD  cmd)
{
	printk("%s\n", __FUNCTION__);

	write_dot11_phy_reg(0x8d, 0x0);
	write_dot11_phy_reg(0xFF, 0x00);

	switch(cmd->param.int_val)
	{
		case 0://send normal data
		{
			printk("send normal data \n");
			write_dot11_phy_reg(0x8d, 0);	
		}
		break;
		
		case 1://send random data
		{
			printk("send random data \n");
			write_dot11_phy_reg(0x8c, 0x41);
			write_dot11_phy_reg(0x8d, 0x8d);	
		}
		break;

		case 2: //send all number 1
		{
			printk("send all number 1 \n");
			write_dot11_phy_reg(0x8c, 0x1);
			write_dot11_phy_reg(0x8d, 0x8d);	
		}
		break;

		case 3://send all number 0
		{
			printk("send all number 1 \n");
			write_dot11_phy_reg(0x8c, 0x31);
			write_dot11_phy_reg(0x8d, 0x8d);
		}
		break;

		case 4://send all number 01010101
		{
			printk("send all number 01010101 \n");
			write_dot11_phy_reg(0x8c, 0xF1);
			write_dot11_phy_reg(0x8d, 0x8d);
		}
		break;
		
		default:
		{
			printk("[npi]: wrong code type \n");
			set_result_code(cmd,-1);
			return 0;
		}
	}
	set_result_code(cmd,0);
	return 0;
}

static int npi_set_tx_power(PNPI_CMD  cmd)
{
	int ret = 0;
	UWORD8 reg_value = 0;
	
	printk("enter %s\n", __FUNCTION__);

	printk("[npi]: level = %d \n", cmd->param.int_val);

	read_dot11_phy_reg((UWORD32)0xFF, &reg_value);
	write_dot11_phy_reg((UWORD32)0xFF, (UWORD32)0);

	print_power_info();

	switch(cmd->param.int_val)
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
			printk("[npi]: set error power level!\n");
			ret = -1;		
		}
	}

	set_result_code(cmd,ret);
	print_power_info();
	write_dot11_phy_reg((UWORD32)0xFF, (UWORD32)reg_value);
	
	return 0;
}

static int npi_connect_ap(PNPI_CMD  cmd)
{
	UWORD8 ssid[64] = {0};
	UWORD8 i = 0;
	UWORD8 retry_count = 10;
	int ret = -1;

	printk("enter: %s\n", __FUNCTION__);

	if(cmd->param.buffer == NULL ) return -EINVAL;

	g_default_scan_limit = 0;
	g_connect_ok_flag = 0;
	
	copy_from_user(ssid,cmd->param.buffer,cmd->param_buffer_len);

	printk("npi: connect ap is: %s\n", ssid);
	mset_DesiredSSID(ssid);
	g_npi_scan_flag = 1;
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
	
	set_result_code(cmd,ret);
	return 0;
}


static int npi_disconnect_ap(PNPI_CMD  cmd)
{
	printk("enter: %s\n", __FUNCTION__);
#ifdef IBSS_BSS_STATION_MODE
	if (itm_scan_flag == 1)
	{
		printk("is aready scan......\n");
		set_result_code(cmd,-1);
		return 0;
	}
#endif
	restart_mac_plus(&g_mac, 0);
	set_result_code(cmd,0);
	return 0;
}

static int npi_set_channel(PNPI_CMD  cmd)
{
	UWORD8 * host_req  = NULL;
	UWORD16  trout_rsp_len = 0;
	int len = 0;
	int ret = 0;

	host_req = g_itm_config_buf;

	if(host_req == NULL)
	{
		ret =  -ENOMEM;
	}

	host_req[len+0] = WID_PRIMARY_CHANNEL & 0xFF;
	host_req[len+1] = (WID_PRIMARY_CHANNEL & 0xFF00) >> 8;
	host_req[len+2] = 1;
	host_req[len+3] = cmd->param.int_val & 0xFF;
	len += WID_CHAR_CFG_LEN;

	host_req[len+0] = WID_RESET & 0xFF;
	host_req[len+1] = (WID_RESET & 0xFF00) >> 8;
	host_req[len+2] = 1;
	host_req[len+3] = 0; //DONT_RESET
	len += WID_CHAR_CFG_LEN;

	config_if_for_iw(&g_mac,host_req,len,'W',&trout_rsp_len);

	if( trout_rsp_len != 1 )
	{
		ret = -1;
	}
out:	
	set_result_code(cmd,ret);
	return 0;
}
static int npi_get_channel(PNPI_CMD  cmd)
{

        UWORD8 * host_req  = NULL;
        UWORD8 * trout_rsp = NULL;
        UWORD16  trout_rsp_len = 0;
        int channel = 0;
	int ret = 0;

	printk("%s\n", __FUNCTION__);
	
        host_req = g_itm_config_buf;

        if(host_req == NULL)
        {
        	ret = -ENOMEM;
                goto out;
        }
        host_req[0] = WID_PRIMARY_CHANNEL & 0xFF;
        host_req[1] = (WID_PRIMARY_CHANNEL & 0xFF00) >> 8;

        trout_rsp = config_if_for_iw(&g_mac,host_req,2,'Q',&trout_rsp_len);

        if( trout_rsp == NULL )
        {
        	ret = -EINVAL;
                goto out;
        }

        trout_rsp_len -= MSG_HDR_LEN;
        if((ret = get_trour_rsp_data((UWORD8*)(&channel),sizeof(UWORD8),&trout_rsp[MSG_HDR_LEN], trout_rsp_len,WID_CHAR))  ==  0)
        {
                printk("get_trour_rsp_data == fail\n");
                ret =  -EFAULT;
        }else{
        	if(cmd->result.int_val){
			copy_to_user(cmd->result.int_val,&channel,sizeof(channel));
			ret = 0;
		}else{
			ret = -EINVAL;
		}
        }

        pkt_mem_free(trout_rsp);

out:
	set_result_code(cmd,ret);

        return 0;
}

static int npi_get_rssi(PNPI_CMD  cmd)
{
	u32 reg_data;
	u16 len = 0;
	unsigned int rssi = 0;
	char tmp_data[128];

	printk("%s\n", __FUNCTION__);
	
   	write_dot11_phy_reg(0xff, 0x0);
    	read_dot11_phy_reg(0x78, &reg_data);
    	
    	printk("[npi]: read 0x78 is %d\n",reg_data);	

   	if(reg_data<256) {
        	rssi=256-reg_data;
        	len=rssi_to_str(rssi, tmp_data);
    	}else {
        	rssi=reg_data-256;
		len = int_to_str(rssi, tmp_data);
    	}
	set_result_buffer(cmd,tmp_data,len);
	set_result_code(cmd, 0);
	printk("[npi]:return rssi is -%s dbm\n",(char*)tmp_data);
    	return 0;
}

static int npi_get_rx_error(PNPI_CMD  cmd)
{
	UWORD32 len = 0;
	UWORD32 all_len = 0;
	UWORD8  tmp_data[64] = {0,};
	UWORD32 rx_end_cnt = 0; 
	UWORD32 rx_err_end_cnt = 0;
	UWORD32 fcs_err_cnt = 0;

	printk("[%s] enter\n", __FUNCTION__);

	disable_machw_phy_and_pa();
	rx_end_cnt = get_machw_rx_end_count();
	rx_err_end_cnt = get_machw_rx_error_end_count();
	fcs_err_cnt = get_fcs_count();
	enable_machw_phy_and_pa();
	
	len = int_to_str(rx_end_cnt, tmp_data);
	all_len += len;
	tmp_data[all_len] = ',';
	all_len += 1;
	len = int_to_str(rx_err_end_cnt, tmp_data+all_len);
	all_len += len;
	tmp_data[all_len] = ',';
	all_len += 1;
	len = int_to_str(fcs_err_cnt, tmp_data+all_len);
	all_len += len;
	tmp_data[all_len] = '\0';
	printk("[%s] %s\n", __FUNCTION__, tmp_data);
	set_result_buffer(cmd,tmp_data,all_len);
	set_result_code(cmd,0);
	return 0;
}

static int npi_get_rx_count(PNPI_CMD  cmd)
{
	UWORD32 rx_count = 0;
	UWORD32 fcs_error_count = 0;
	char tmp_data[64]={0,};
	int len = 0;
	
	printk("%s\n", __FUNCTION__);
	
	disable_machw_phy_and_pa();
	rx_count = get_machw_rx_end_count();
	printk("rx end count is: %u\n", rx_count);
	fcs_error_count = get_fcs_count();
	enable_machw_phy_and_pa();

	rx_count = rx_count - fcs_error_count;

	len = int_to_str(rx_count, tmp_data);
	
	set_result_buffer(cmd,tmp_data,len);
	set_result_code(cmd,0);
	
	printk("fcs error count is: %u\n", fcs_error_count);
	
	return 0;
}

static int npi_get_tx_power(PNPI_CMD  cmd)
{
    u16 len = 0;
    UWORD8  tmp_data[128]={0,};
    UWORD8 wifi_mode = 0;
    UWORD8 reg_value = 0;
    UWORD8 power_level_b = 0;
    UWORD8 power_level_a = 0;
    int    power_level = 0;
    int    level_a = 0, level_b = 0;

    printk("%s\n", __FUNCTION__);
	
    wifi_mode = get_tx_mode(g_user_tx_rate);
    printk("wifi mode is: %u\n", wifi_mode);

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

     set_result_buffer(cmd, tmp_data, len);
     set_result_code(cmd,0);
	
     printk("power level value is: %u,%u (%#x, %#x), ret=%s.\n", 
		level_b, level_a,
		power_level_b, power_level_a, tmp_data);
     return 0;
}

static int npi_get_rx_status(PNPI_CMD  cmd)
{
	int ret = 0;
	u16 len = 0;
	u16 all_len = 0;

	UWORD8  tmp_data[128]={0,};
	u32 fcs_fail_cnt = 0, frame_filter = 0, frame_header_filter = 0,
		rx_q_lost_cnt = 0, rx_q_cnt = 0, rx_end_cnt = 0, rx_end_error_cnt = 0, 
		duplicate_cnt = 0, ampdu_rx_cnt = 0, mpdu_in_ampdu_cnt = 0, crc_pass_cnt = 0, rx_all_cnt = 0;

	printk("npi : enter: %s\n", __FUNCTION__);
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

	printk("[%s] tmp_data: %s  all_len = %d\n", __FUNCTION__, tmp_data, all_len);
      set_result_buffer(cmd, tmp_data, all_len);
      set_result_code(cmd,0);
	return ret;
}


static struct cmd_entry cmd_table[MAX_CMD_CODE]={
	ENTRY(SET_MAC,npi_set_mac),
	ENTRY(TX_START,npi_start_tx_data),
	ENTRY(TX_STOP, npi_stop_tx_data),
	ENTRY(RX_START,npi_start_rx_data),
	ENTRY(RX_STOP,npi_stop_rx_data),
	ENTRY(SET_RATE,npi_set_tx_rate),
	ENTRY(SET_CHANNEL,npi_set_channel),
	ENTRY(GET_CHANNEL,npi_get_channel),
	ENTRY(SET_TX_POWER,npi_set_tx_power),
	ENTRY(GET_TX_POWER,npi_get_tx_power),
	ENTRY(GET_RX_COUNT,npi_get_rx_count),
	ENTRY(SET_RX_COUNT,npi_set_rx_count),
	ENTRY(GET_RX_ERROR,npi_get_rx_error),
	ENTRY(GET_RSSI,npi_get_rssi),
	ENTRY(SET_TX_MODE,npi_set_tx_mode),
	ENTRY(CONNECT_AP,npi_connect_ap),
	ENTRY(DISCONNECT_AP,npi_disconnect_ap),
	ENTRY(GET_RX_STATUS,npi_get_rx_status),	
};

int npi_dispatch(struct ifreq * req)
{
	NPI_CMD npi_cmd;
	int ret =  -EFAULT;
	int i = 0;
	
	printk("[npi]: %s\n", __FUNCTION__);
	
	if(req && req->ifr_data == NULL) return -EINVAL;

	if (copy_from_user(&npi_cmd, req->ifr_data, sizeof(npi_cmd))){
    		return -EFAULT;
    	}

	for(i = 0;i<MAX_CMD_CODE;i++){
		if(npi_cmd.cmd == cmd_table[i].cmd){
			ret = cmd_table[i].op(&npi_cmd);
			break;
		}
	}

	if(i >= MAX_CMD_CODE){
		printk("[npi] : wrong cmd error. cmd = %d\n",npi_cmd.cmd);
	}
	
	return ret;
}
#endif
