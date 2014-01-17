/*****************************************************************************/
/*                                                                           */
/*                     Ittiam 802.11 MAC SOFTWARE                            */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2005                               */
/*                                                                           */
/*  This program  is  proprietary to  Ittiam  Systems  Private  Limited  and */
/*  is protected under Indian  Copyright Law as an unpublished work. Its use */
/*  and  disclosure  is  limited by  the terms  and  conditions of a license */
/*  agreement. It may not be copied or otherwise  reproduced or disclosed to */
/*  persons outside the licensee's organization except in accordance with the*/
/*  terms  and  conditions   of  such  an  agreement.  All  copies  and      */
/*  reproductions shall be the property of Ittiam Systems Private Limited and*/
/*  must bear this notice in its entirety.                                   */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/
/*                                                                           */
/*  File Name         : trout_rf_test_mode.c                                             */
/*                                                                           */
/*  Description       : This file contains the initialization related        */
/*                      trout_rf_test_mode                                   */
/*                                                                           */
/*  List of Functions :                                                      */
/*                                                       */
/*                                                      */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef TROUT_RF_TEST

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "trout_rf_test_mode.h"
#include "imem.h"
#include "core_mode_if.h"
#include "mib_11n.h"
#include "reg_domains.h"
#include "iconfig.h"

#include <linux/workqueue.h>

#define ENABLE_NPI_OUTPUT_LOG

#undef RF_TEST_MODE_TX_PROBE_REQ
#define RF_TEST_SCHED_TO

// caisf add for test rf performance
#define RF_TEST_PKT_LEN     (2*1024)

UWORD16 rf_param_st_magic = 0x9812;
UWORD8 *rf_stop_magic  = "now-stop-work";   // stop
UWORD8 *rf_nstop_magic = "non-stop-work";   // non stop
volatile int trout_rf_work_flag = 0;

UWORD16 g_rf_test_rsp = 0;
char *  g_rf_test_pkt = NULL;
UWORD16 g_rf_test_pkt_len = 0;
char *  g_rf_test_log = NULL;
UWORD16 g_rf_test_log_len = 0;
UWORD16 max_rf_test_log_len = 1536; // 1.5k

static unsigned char TestTxData_QAM64[48] = 
{
	0x00,0x10,0x83,0x10,0x51,0x87,//0-7
	0x20,0x90,0x8B,0x30,0xD3,0x8F,//8-15
	0x41,0x14,0x93,0x51,0x55,0x97,//16-23
	0x61,0x96,0x9B,0x71,0xD7,0x9F,//24-31
	0x82,0x18,0xA3,0x92,0x59,0xA7,//32-39
	0xA2,0x9A,0xAB,0xB2,0xDB,0xAF,//40-47
	0xC3,0x1C,0xB3,0xD3,0x5D,0xB7,//48-55
	0xE3,0x9E,0xBB,0xF3,0xDF,0xBF //56-63
};

// duration id: 48ms
static char g_rf_test_data_pkt_hdr[] = {
    0x08, 0x01, 0x2c, 0x00, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x05, 0x06, 0x06, 0x06, 0x06, 0x05,
    0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x00, 0x00
};

static char g_rf_test_qos_data_pkt_hdr[] = {
    0x88, 0x01, 0x2c, 0x00, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x05, 0x06, 0x06, 0x06, 0x06, 0x05,
    0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x00, 0x00, 0x00, 0x00
};


int trout_rf_test_record(const char *fmt, ...)
{
	va_list args;
	int i;
    char *buf = NULL;
    
    if(g_rf_test_log == NULL){
        printk("%s: trout rf tool record error!\n", __FUNCTION__);
        return -1;
    }
    
    buf = g_rf_test_log + strlen(g_rf_test_log);

	va_start(args, fmt);
	i = vsprintf(buf, fmt, args);
	va_end(args);
    
    //if(max_rf_test_log_len < (g_rf_test_log_len+i))
    {
        //printk("%s: ERROR! len=%d (%d)\n",__FUNCTION__,g_rf_test_log_len,i);
        //return -1;
    }
    g_rf_test_log_len = strlen(buf) + 1;
    
	return i;
}

void trout_rf_test_print_param(struct rf_test_param *p_param)
{
    UWORD8 *pwr = NULL;
    UWORD32 ret32 = 0, i = 0;
    
    trout_rf_test_record("\n== TROUT RF TEST PARAMS ==\n");

    pwr = get_rx_power_level();
    if(pwr)
    {
        trout_rf_test_record("rx power level: %d, val: ",pwr[0]);
        for(i=1; i<=pwr[0]; i++)
        {
            trout_rf_test_record("0x%x, ",*(pwr+i));
        }
        trout_rf_test_record("\n");
    }
    
    ret32 = get_tx_power_levels_dbm();
    //trout_rf_test_record("tx power level: 0x%02x, 0x%02x, 0x%02x, 0x%02x\n",
    //    ret32&&(0xFF<<24),ret32&&(0xFF<<16),ret32&&(0xFF<<8),ret32&&0xFF);
    trout_rf_test_record("tx power level: 0x%08x\n",ret32);
}

// build header data fcs memory, return pkt pkt_len
UWORD32 rf_test_build_package(struct rf_test_param *p_param)
{
    char *ptr = NULL;
    UWORD32 pkt_len = 0, tmp_len = 0, hdr_len = 0, frame_len = 0;
    UWORD32 i = 0;
    
    if(g_rf_test_pkt!=NULL)
    {
        printk("%s: g_rf_test_pkt not null.\n",__FUNCTION__);
        return 0;
    }

#if 1
    if(p_param->phy_mode == 2)
        hdr_len = sizeof(g_rf_test_qos_data_pkt_hdr); // qos header
    else
        hdr_len = sizeof(g_rf_test_data_pkt_hdr); // normal header
#else
    hdr_len = sizeof(g_rf_test_data_pkt_hdr);
#endif
    frame_len = hdr_len+p_param->tx_pkt_size+4;

#if 0
    g_rf_test_pkt = (UWORD8 *)mem_alloc(g_shared_pkt_mem_handle,
                                     frame_len);
	if(g_rf_test_pkt == NULL)
    {
        /* Error condition: Reset the whole MAC */
		printk(KERN_ERR "mem_alloc for g_rf_test_pkt failed! len=%d\n",
		            frame_len);
        raise_system_error(NO_SHRED_MEM);
        return 0;
    }
#else
    g_rf_test_pkt = (char *)kmalloc(frame_len, GFP_KERNEL);
	if(g_rf_test_pkt == NULL)
	{
		printk(KERN_ERR "mem_alloc for g_rf_test_pkt failed! len=%d\n",
		            frame_len);
		return 0;
    }
#endif
    memset(g_rf_test_pkt, 0, frame_len);

    ptr = g_rf_test_pkt;

    if(p_param->phy_mode == 2)
    { // bgn mode, build qos data frame 
        memcpy(ptr, g_rf_test_qos_data_pkt_hdr, sizeof(g_rf_test_qos_data_pkt_hdr));
    }
    else
    { // bgn mode, build normal data frame 
        memcpy(ptr, g_rf_test_data_pkt_hdr, sizeof(g_rf_test_data_pkt_hdr));
    }
    pkt_len += hdr_len;

    ptr += 4;
    memcpy(ptr, p_param->dmac, 6);
    //ptr += 12;
    //memcpy(ptr, p_param->dmac, 6);
    
    ptr = g_rf_test_pkt + pkt_len;
    tmp_len = p_param->tx_pkt_size;

#if 0
    do{
        if(sizeof(TestTxData_QAM64) < tmp_len)
        {
            memcpy(ptr, TestTxData_QAM64, sizeof(TestTxData_QAM64));
            
            ptr += sizeof(TestTxData_QAM64);
            pkt_len += sizeof(TestTxData_QAM64);
            
            tmp_len -= sizeof(TestTxData_QAM64);
        }
        else
        {
            memcpy(ptr, TestTxData_QAM64, tmp_len);
            
            ptr += tmp_len;
            pkt_len += tmp_len;
            
            tmp_len = 0;
        }
    }while(tmp_len != 0);
#else
    if(sizeof(TestTxData_QAM64) < tmp_len)
    {
        memcpy(ptr, TestTxData_QAM64, sizeof(TestTxData_QAM64));
        
        ptr += sizeof(TestTxData_QAM64);
        pkt_len += sizeof(TestTxData_QAM64);
        
        tmp_len -= sizeof(TestTxData_QAM64);

        for(i = 0; i<tmp_len; i++)
        {
            *ptr = i+1;
            pkt_len++;
            
            ptr++;
        }
    }
    else
    {
        memcpy(ptr, TestTxData_QAM64, tmp_len);
        
        ptr += tmp_len;
        pkt_len += tmp_len;
        
        tmp_len = 0;
    }
#endif

    pkt_len += 4; // fcs len
    
    if(pkt_len != frame_len)
    {
        trout_rf_test_record("there is a bug pkt_len (%d) != frame_len (%d)!\n",
            pkt_len,frame_len);
        printk("there is a bug pkt_len (%d) != frame_len (%d)!\n",
            pkt_len,frame_len);
    }
    
    g_rf_test_pkt_len = pkt_len;

    hex_dump("rf_test_build_package", g_rf_test_pkt, g_rf_test_pkt_len);
    return pkt_len;
}

long rf_rx_fcs_err_count = 0;
long rf_rx_end_count = 0;
long rf_rx_error_count = 0;
int trout_rf_test_get_ret(__user char *buf, int buf_len, int request)
{
    struct trout_private *tp = netdev_priv(g_mac_dev);
    struct rf_test_param *p_param = &tp->rf_param;
    struct mutex *p_rf_lock = &tp->rf_lock;
    int len = 0;

    if(trout_rf_work_flag != 0)
    {
        printk("%s: trout rf tool working! Wait a moment.\n", __FUNCTION__);
        return -1;
    }
    #if 0
    if(g_rf_test_rsp == 0)
    {
        printk("%s: g_rf_test_rsp = 0, you should check the logic!\n", __FUNCTION__);
        return -1;
    }
    #endif
    

    // print Tx\Rx info
    if(p_param->type == 1)
    {
        trout_rf_test_record("\n====  Transmit statistics log ====\n");
#ifdef DEBUG_MODE
        trout_rf_test_record("TxC Interrupts = %d\n", g_mac_stats.itxc);
        trout_rf_test_record("TBTT Interrupts = %d\n", g_mac_stats.itbtt);
        trout_rf_test_record("tx is busy!count = %d\n", g_mac_stats.litxe); //dumy add for tx busy count
#endif /* DEBUG_MODE */
        trout_rf_test_record("Tx Fragments = %d\n", mget_TransmittedFragmentCount());
        trout_rf_test_record("Tx Frames = %d\n", mget_TransmittedFrameCount());
        trout_rf_test_record("ACK Failures = %d\n", mget_ACKFailureCount());
        trout_rf_test_record("Failed Frames = %d\n", mget_FailedCount());
        trout_rf_test_record("Retried Frames = %d\n", mget_RetryCount());
        trout_rf_test_record("Multiple Retried Frames = %d\n", mget_MultipleRetryCount());
#ifdef PHY_802_11n
        trout_rf_test_record("Tx AMSDU Frames = %d\n", mget_TransmittedAMSDUCount());
        trout_rf_test_record("Failed AMSDU Frames = %d\n", mget_FailedAMSDUCount());
        trout_rf_test_record("Retried AMSDU Frames = %d\n", mget_RetryAMSDUCount());
        trout_rf_test_record("Mult Ret AMSDU Frames = %d\n", mget_MultipleRetryAMSDUCount());
        trout_rf_test_record("AMSDU ACK Failures = %d\n", mget_AMSDUAckFailureCount());
        trout_rf_test_record("Tx AMPDU Frames = %d\n", mget_TransmittedAMPDUCount());
#endif /* PHY_802_11n */

        trout_rf_test_record("Tx b,g,n power level(dbm): 0x%02x, 0x%02x, 0x%02x\n", 
            get_curr_tx_power_dbm_11b(),get_curr_tx_power_dbm_11a(),get_curr_tx_power_dbm_11n());

        trout_rf_test_record("==================================\n");
        
    }
    else if(p_param->type == 2)
    {
        UWORD32 error_rate = 0;

        trout_rf_test_record("\n====  Receive statistics log ===\n");
#ifdef DEBUG_MODE
        trout_rf_test_record("RxC Interrupt = %d\n", g_mac_stats.irxc);
        trout_rf_test_record("HighPri RxC Interrupt = %d\n", g_mac_stats.ihprxc);
        trout_rf_test_record("invalid rx complete isr 000!count = %d\n", g_mac_stats.lirrd); //dumy add for invalid rx complete isr 000 count
#endif /* DEBUG_MODE */
        //trout_rf_test_record("Rx Frames = %d\n", mget_ReceivedFragmentCount());
        trout_rf_test_record("FCS Failures = %d\n", get_fcs_count());

        trout_rf_test_record("rx_frame_into_ram_count = %d\n", get_rx_ram_packet_count());
        trout_rf_test_record("rx_frame_filter_count = %d\n", get_rx_frame_filter_count());
        trout_rf_test_record("rx_mac_header_filter_count = %d\n", get_rx_mac_header_filter_count());
        trout_rf_test_record("rxq_full_filter_count = %d\n", get_rxq_full_filter_count());
        
        trout_rf_test_record("Duplicate frames = %d\n", get_dup_count());

    	trout_rf_test_record("PA Ctrl reg value = %d\n", get_machw_pa_reg_value());	//add by chengwg.

#if 0
        set_rx_frame_filter_count(0);
        set_rx_mac_header_filter_count(0);
        set_rxq_full_filter_count(0);
#endif
        
#ifdef PHY_802_11n
        trout_rf_test_record("Rx AMSDU Frames = %d\n", mget_ReceivedAMSDUCount());
        trout_rf_test_record("Rx AMPDU Frames = %d\n", mget_AMPDUReceivedCount());
#endif /* PHY_802_11n */

        if(g_mac_stats.rx_data_count != 0)
        {
            UWORD32 val = ((g_mac_stats.rx_data_dscr_total_num*10) / g_mac_stats.rx_data_count);
            trout_rf_test_record("DMA Data num_dscr average val  = %d\n", val); //dumy add for rx data 
        }
        if(g_mac_stats.rx_manage_count != 0)
        {
            UWORD32 val = ((g_mac_stats.rx_manage_dscr_total_num*10) / g_mac_stats.rx_manage_count);
            trout_rf_test_record("WiFi Rx Data num_dscr average val = %d\n", val); //dumy add for rx manage 
        }
        #if 0
        PRINTK("=============Num-Dscr=====================\n");
        for(i=0;i<2000;i++)
        {
            PRINTK("%02d ", g_mac_stats.num_dsc[i]);
            g_mac_stats.num_dsc[i] = 0;
            if((i > 0)&& ((i %19) == 0))
            {
                PRINTK("\n\r");
            }
        }
        PRINTK("\n=========================================\n");
        #endif
        
        trout_rf_test_record("WiFi Rx Data num_dscr = 1 = %d\n", g_mac_stats.num_1_count);    
        trout_rf_test_record("WiFi Rx Data num_dscr = 2 = %d\n", g_mac_stats.num_2_count);        
        trout_rf_test_record("WiFi Rx Data 2< num_dscr < 10 = %d\n", g_mac_stats.num_lis_10_count);   
        trout_rf_test_record("WiFi Rx Data 10< num_dscr < 20 = %d\n", g_mac_stats.num_lis_20_count);       
        trout_rf_test_record("WiFi Rx Data 20< num_dscr < 30 = %d\n", g_mac_stats.num_lis_30_count);       
        trout_rf_test_record("WiFi Rx Data 30< num_dscr < 40 = %d\n", g_mac_stats.num_lis_40_count);       
      
    	trout_rf_test_record("scaned AP count = %d\n", g_mac_stats.scan_ap_count);         
    	trout_rf_test_record("rcv prob rsp count = %d\n", g_mac_stats.rcv_prob_rsp_count);    
        trout_rf_test_record("Received Frame Filter = 0x%x\n",convert_to_le(host_read_trout_reg((UWORD32)rMAC_RX_FRAME_FILTER)));

        trout_rf_test_record("\n");

        trout_rf_test_record("\n====  RF-test-tool Rx result ===\n");
        
        trout_rf_test_record("Rx Data Frames = %d\n", mget_ReceivedFragmentCount());//dumy add for test 0726
        if(get_machw_rx_end_count())
        {
            error_rate = ((get_machw_rx_end_count() - mget_ReceivedFragmentCount())*100)/get_machw_rx_end_count();
            trout_rf_test_record("Rx Data Frames Error Rate = %d\%\n", error_rate);
        }
        else
        {
            trout_rf_test_record("Rx Data Frames Error Rate cannot compute!\n");
        }
        trout_rf_test_record("Rx Multicast Data Frames = %d\n", mget_MulticastReceivedFrameCount());//dumy add for test 0726

        trout_rf_test_record("\n-----All test count-----\n");

        trout_rf_test_record("ALL RX-End Count = %d\n", get_machw_rx_end_count()); // phy recv, unfilter
        trout_rf_test_record("ALL RX-End Error Count = %d\n", get_machw_rx_error_end_count());
        trout_rf_test_record("ALL RX FCS Error Count = %d\n", get_fcs_count());
        {
            UWORD32 tmp_rxend_count = get_machw_rx_end_count();
            UWORD32 tmp_rxerr_count = get_machw_rx_error_end_count();
            UWORD32 tmp_rx_fcs_err_count = get_fcs_count();
            if((tmp_rxend_count+(tmp_rxerr_count)))
            {
                error_rate = ((tmp_rxerr_count+tmp_rx_fcs_err_count)*100)/(tmp_rxend_count+(tmp_rxerr_count));
                trout_rf_test_record("ALL Rx-End Packet's Error Rate = %d\%\n", error_rate);
            }
            else
            {
                trout_rf_test_record("ALL Rx-End Packet's Error Rate cannot compute!\n", error_rate);
            }
        }
        
        trout_rf_test_record("\n-----Current test count-----\n");

        {
            UWORD32 tmp2_rx_error_count = get_machw_rx_error_end_count() - rf_rx_error_count;
            UWORD32 tmp2_rx_fcs_err_count = get_fcs_count() - rf_rx_fcs_err_count;
            UWORD32 tmp2_rx_end_count = get_machw_rx_end_count() - rf_rx_end_count;
            trout_rf_test_record("(%d - %d = )RX-End Error Count = %d\n",
                get_machw_rx_error_end_count(),rf_rx_error_count,tmp2_rx_error_count);
            trout_rf_test_record("(%d - %d = )RX FCS Error Count = %d\n",
                get_fcs_count(),rf_rx_fcs_err_count,tmp2_rx_fcs_err_count);
            trout_rf_test_record("(%d - %d = )RX-End Count = %d\n",
                get_machw_rx_end_count(),rf_rx_end_count,tmp2_rx_end_count); // phy recv, unfilter
            if(0!=(tmp2_rx_end_count+(tmp2_rx_error_count)))
            {
                error_rate = ((tmp2_rx_error_count+tmp2_rx_fcs_err_count)*100)/(tmp2_rx_end_count+(tmp2_rx_error_count));
                trout_rf_test_record("error_rate = ((rf_rx_error_count+rf_rx_fcs_err_count)*100)/(rf_rx_end_count+(rf_rx_error_count));\n", error_rate);
                trout_rf_test_record("Rx-End Packet Error Rate = %d\%\n", error_rate);
            }
            else
            {
                trout_rf_test_record("Rx-End Packet Error Rate cannot compute!\n");
            }
        }

        trout_rf_test_record("----------\n");

    }

    
	mutex_lock(p_rf_lock);
    
    {
        len = (strlen(g_rf_test_log)+1) < buf_len ? (strlen(g_rf_test_log)+1) : buf_len;
        if(copy_to_user(buf, g_rf_test_log, len)) 
        {
            printk("%s: copy_to_user fail. buf_len = %d, all=%d\n",
                __FUNCTION__,buf_len,strlen(g_rf_test_log)+1);
            len = -1;
        }
        printk("len:%d, log:%s\n",len,g_rf_test_log);
    }
    
	mutex_unlock(p_rf_lock);

    // clean 
    if(len > 0)
    {
        //if(!g_reset_mac_in_progress)
    	//    restart_mac(&g_mac,0);
        g_rf_test_rsp = 0;
    }
    memset(g_rf_test_log, '\0', max_rf_test_log_len);
    g_rf_test_log_len = 1;
    
    printk("trout_rf_test_get_ret done.\n");
    return len;
}
EXPORT_SYMBOL(trout_rf_test_get_ret);

void do_rf_test_work(struct work_struct *work)
{
    struct trout_private *tp = netdev_priv(g_mac_dev);
    struct rf_test_param *p_param = &tp->rf_param;
    int tx_rate = get_curr_tx_rate();
    int i = 0, ret = 0;
    int err_count = 0;
    int max_tx_pkt_size = 4096;//1536; // for n: 4067?
#ifdef RF_TEST_MODE_TX_PROBE_REQ
    UWORD8 *probe_frame = NULL;
    UWORD8 *probe_frame_bak = NULL;
    UWORD16 probe_frame_len = 0;
#endif

    if(p_param == NULL)
    {
        printk("trout_rf_test_work p_param NULL!\n");
        trout_rf_test_record("ERROR ! trout_rf_test_work p_param NULL!\n");
        goto unexpected_out;
    }

    //reset
    //if(g_reset_mac_in_progress != BTRUE)
	//    restart_mac(&g_mac,0);

    // prepare current setting for send back to application
    trout_rf_test_print_param(p_param);

    // set config
    if(p_param->type == 1) //tx
    {
        // get tx rate
        //set_curr_tx_rate(p_param->rate);
        tx_rate = p_param->rate;

        switch(tx_rate)
        {
            // N rate
            case 7: // 6.5M
                tx_rate = 0x80;
                break;
            case 13: // 13M
                tx_rate = 0x81;
                break;
            case 19: // 19.5M
                tx_rate = 0x82;
                break;
            case 26: // 26M
                tx_rate = 0x83;
                break;
            case 39: // 39M
                tx_rate = 0x84;
                break;
            case 52: // 52M
                tx_rate = 0x85;
                break;
            case 58: // 58.5M
                tx_rate = 0x86;
                break;
            case 65: // 65M
                tx_rate = 0x87;
                break;
            
            default:
                break;
        }
        if(0x80 <= tx_rate && tx_rate <= 0x87)
        {
            p_param->phy_mode = 2;
            max_tx_pkt_size = 4096;
        }
        printk("set rate:%d, current tx_rate: %d\n",p_param->rate,tx_rate);
        p_param->rate = tx_rate;

        if(p_param->tx_pkt_size > max_tx_pkt_size)
        {
            printk("trout_rf_test error! tx_pkt_size > max_tx_pkt_size %d!\n",max_tx_pkt_size);
            trout_rf_test_record("trout_rf_test error! tx_pkt_size > max_tx_pkt_size %d!\n",max_tx_pkt_size);
            goto unexpected_out;
        }
        
        // set channel and frequence-offset
        set_prim_chnl_num(p_param->channel);
        printk("current channel: %d (%d) offset: 0x%x\n",get_prim_chnl_num(),get_mac_chnl_num(),p_param->freq_offset);
        
        select_channel_rf(p_param->channel-1, p_param->freq_offset);

        // set power
        if(p_param->power != 0)
        {
            UWORD8 phy_rate = get_phy_rate(tx_rate);
			
			set_user_control_enabled(BTRUE);
            if(IS_OFDM_RATE(phy_rate) == BFALSE)
            { // 802.11 b
                set_tx_power_level_11b(p_param->power);
            }
            else
            {
                if(IS_RATE_MCS(phy_rate) == BTRUE)
                { // n
                    set_tx_power_level_11n(p_param->power);
                }
                else
                { // a, g
                    set_tx_power_level_11a(p_param->power);
                }
            }
        }
        
#if 0
    // for test TX
    user_tx_rate = tx_rate;

    do
    {
        UWORD8 *asoc_frame=(UWORD8 *)mem_alloc(g_shared_pkt_mem_handle,MANAGEMENT_FRAME_LEN);
        if(asoc_frame == NULL)
        {
            printk("mem_alloc asoc_frame NULL ERROR!\n");
            break;
        }
        UWORD16 asoc_frame_len = prepare_asoc_req(asoc_frame);
        UWORD8 mac2[6] = {0x05,0x06,0x06,0x06,0x06,0x04};
        UWORD8 mac3[6] = {0x06,0x06,0x06,0x06,0x06,0x06};
        
        memcpy(asoc_frame+4,p_param->dmac,6);
        memcpy(asoc_frame+10,mac2,6);
        memcpy(asoc_frame+16,mac3,6);

        hex_dump("asoc frame", asoc_frame, asoc_frame_len);
        tx_mgmt_frame(asoc_frame, asoc_frame_len, NORMAL_PRI_Q, 0);
        mem_free(g_shared_pkt_mem_handle, asoc_frame);
        asoc_frame = 0;
    }while(0);
#endif
#ifdef RF_TEST_MODE_TX_PROBE_REQ
		start_mac_and_phy(&g_mac);
		mset_DesiredSSID("aplink");

		// for test TX
		user_tx_rate = tx_rate;

		probe_frame=(UWORD8 *)mem_alloc(g_shared_pkt_mem_handle,MANAGEMENT_FRAME_LEN);
		if(probe_frame == NULL)
		{
			TROUT_DBG4("mem_alloc probe_frame NULL ERROR!");
            goto unexpected_out;
		}
		probe_frame_bak=(UWORD8 *)mem_alloc(g_shared_pkt_mem_handle,MANAGEMENT_FRAME_LEN);
		if(probe_frame_bak == NULL)
		{
			TROUT_DBG4("mem_alloc probe_frame_bak NULL ERROR!");
            mem_free(g_shared_pkt_mem_handle, probe_frame);
            goto unexpected_out;
		}
		probe_frame_len = prepare_probe_req(probe_frame);
		UWORD8 mac2[6] = {0x05,0x06,0x06,0x06,0x06,0x04};
		UWORD8 mac3[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

		memcpy(probe_frame+4,mac3,6);
		//memcpy(probe_frame+4,p_param->dmac,6);
		memcpy(probe_frame+10,mget_StationID(),6);
		//memcpy(probe_frame+10,mget_StationID(),6);
		memcpy(probe_frame+16,mac3,6);

		memcpy(probe_frame_bak,probe_frame,probe_frame_len);
		hex_dump("probe_frame", probe_frame, probe_frame_len);

        /* probe_frame will be freed in tx_mgmt_frame() */
		tx_mgmt_frame(probe_frame, probe_frame_len, HIGH_PRI_Q, 0);
		//mem_free(g_shared_pkt_mem_handle, probe_frame);
		probe_frame = 0;
#endif

        // build packages
        if(0 == rf_test_build_package(p_param))
        {
            printk("trout_rf_test build_package error!\n");
            goto unexpected_out;
        }
        
        if(p_param->nonstop_work == 1) // non-stop TX/RX
        {
            do
            {
                //WORD16 retry = 1;
                //do {
                    #ifndef RF_TEST_MODE_TX_PROBE_REQ
                    ret = trout_rf_test_send_pkt(g_rf_test_pkt, g_rf_test_pkt_len, tx_rate);
                    #else
    				probe_frame=(UWORD8 *)mem_alloc(g_shared_pkt_mem_handle,MANAGEMENT_FRAME_LEN);
    				if(probe_frame == NULL)
    				{
    					TROUT_DBG4("mem_alloc probe_frame NULL ERROR!");
                        goto unexpected_out;
    				}
    				memcpy(probe_frame,probe_frame_bak,probe_frame_len);
    				ret = tx_mgmt_frame(probe_frame, probe_frame_len, HIGH_PRI_Q, 0);
                    #endif
                    //printk("%s: send NO.%d pkt, ret=%d\n",__FUNCTION__, i, ret);
                    if(ret < 0)
                    {
                        err_count++;
                        printk("%s: send NO.%d pkt error, ret=%d, err count=%d\n", 
                            __FUNCTION__, i, ret, err_count);

                        //#ifdef RF_TEST_SCHED_TO
                        set_current_state(TASK_INTERRUPTIBLE);
                        schedule_timeout(msecs_to_jiffies(20));//  ms
                        //#endif
                    }
                //} while((ret < 0) && (--retry > 0));
#if 0
                if((ret < 0) && (err_count>10) && (err_count > i/3))
                {
                    trout_rf_test_record("%s: bad memory, abort TX!  err_count=%d, current count=%d!\n",
                        __FUNCTION__,err_count,i);
                    break;
                }
#endif
                i++;
                
                #ifdef RF_TEST_SCHED_TO
                set_current_state(TASK_INTERRUPTIBLE);
                schedule_timeout(msecs_to_jiffies(10));// 10 ms
                #endif
            }while(p_param->nonstop_work != 2);
            
        }
        else
        {
            printk("start tx. pkt num:%d\n",p_param->tx_pkt_num);
            for(i=0; i<p_param->tx_pkt_num; i++)
            {
                //WORD16 retry = 1;
                //do {
					#ifndef RF_TEST_MODE_TX_PROBE_REQ
                        #if 0
                            if(p_param->power == 1)
                                ret = trout_rf_test_send_pkt_hugh(g_rf_test_pkt, g_rf_test_pkt_len, tx_rate);
                            else
                                ret = trout_rf_test_send_pkt(g_rf_test_pkt, g_rf_test_pkt_len, tx_rate);
                        #else
                            ret = trout_rf_test_send_pkt(g_rf_test_pkt, g_rf_test_pkt_len, tx_rate);
                        #endif
					#else
					probe_frame=(UWORD8 *)mem_alloc(g_shared_pkt_mem_handle,MANAGEMENT_FRAME_LEN);
					if(probe_frame == NULL)
					{
						TROUT_DBG4("mem_alloc probe_frame NULL ERROR!");
                        goto unexpected_out;
					}
					memcpy(probe_frame,probe_frame_bak,probe_frame_len);
					ret = tx_mgmt_frame(probe_frame, probe_frame_len, HIGH_PRI_Q, 0);					
					#endif
                    
                    //printk("%s: send NO.%d pkt, ret=%d\n",__FUNCTION__, i, ret);
                    if(ret < 0)
                    {
                        err_count++;
                        printk("%s: send NO.%d pkt error, ret=%d, err count=%d\n", __FUNCTION__, i, ret, err_count);

                    //#ifdef RF_TEST_SCHED_TO
                        set_current_state(TASK_INTERRUPTIBLE);
                        schedule_timeout(msecs_to_jiffies(20));//  ms
                    //#endif
                    }

                    #ifdef RF_TEST_SCHED_TO
                    set_current_state(TASK_INTERRUPTIBLE);
                    schedule_timeout(msecs_to_jiffies(10));// 10 ms
                    #endif
                //} while((ret < 0) && (--retry > 0));
                
            }
        }
        printk("%s: TX end, error_count=%d\n",__FUNCTION__,err_count);
    	trout_rf_test_record("TX count = %d (%d)\n",i, err_count);

        #ifdef RF_TEST_MODE_TX_PROBE_REQ
    		if(probe_frame_bak != NULL)
    			mem_free(g_shared_pkt_mem_handle, probe_frame_bak);
            probe_frame_bak = NULL;
		    probe_frame = 0;
        #endif
        
        if(g_rf_test_pkt != NULL)
        {
            #if 0
            mem_free(g_shared_pkt_mem_handle,g_rf_test_pkt);
            #else
	        kfree(g_rf_test_pkt);
            #endif
            g_rf_test_pkt = NULL;
        }
        
        trout_rf_work_flag = 0;
        
    }
    else if(p_param->type == 2) //rx
    {
        rf_rx_error_count = get_machw_rx_error_end_count();
        rf_rx_fcs_err_count = get_fcs_count();
        rf_rx_end_count = get_machw_rx_end_count();

        //unmask_machw_rx_comp_int();
        
        // set channel
        select_channel_rf(p_param->channel-1, p_param->freq_offset);

    }
    else
    {
        printk("%s: something wrong?\n",__FUNCTION__);
        trout_rf_work_flag = 0;
    }

    return;
    
unexpected_out:
	trout_rf_test_record("ERROR! GOTO unexpected_out!\n");

    if(g_rf_test_pkt != NULL)
    {
        #if 0
        mem_free(g_shared_pkt_mem_handle,g_rf_test_pkt);
        #else
        kfree(g_rf_test_pkt);
        #endif
        g_rf_test_pkt = NULL;
    }

    #ifdef RF_TEST_MODE_TX_PROBE_REQ
		if(probe_frame_bak != NULL)
			mem_free(g_shared_pkt_mem_handle, probe_frame_bak);
        probe_frame_bak = NULL;
	    probe_frame = 0;
    #endif
    
    trout_rf_work_flag = 0;
}

// need response
int trout_rf_test_set_param(const __user char *buf, int buf_len, int response)
{
    struct trout_private *tp = netdev_priv(g_mac_dev);
    struct rf_test_param *p_param = &tp->rf_param;
    struct mutex *p_rf_lock = &tp->rf_lock;
    int len = 0;
    int ret = 0;

    if(trout_rf_work_flag == 1)
    { // change some state while rf-test working
        if(buf_len == (strlen(rf_stop_magic)+1))
        {
            UWORD8 tmp_str[24];
            
            mutex_lock(p_rf_lock);
            
        	if(copy_from_user(tmp_str, (char *)buf, buf_len))
        	{
                printk("trout_rf_test copy_from_user(tmp_str) failed: len=%d\n.", buf_len);
                return -EFAULT;
            }
            else
                len = buf_len;
            
        	mutex_unlock(p_rf_lock);
            
            printk("get rf cmd:%s\n", tmp_str);
            if(memcmp(tmp_str, rf_stop_magic, buf_len)==0)
            {
                p_param->nonstop_work = 2;
                if(p_param->type == 2)
                    trout_rf_work_flag = 0;
            }
            
            return len;
        }
    
        printk("trout_rf_test trout_rf_work_flag working! skip this request!\n");
        return -EFAULT;
    }


    if(buf_len != sizeof(struct rf_test_param))
    {
        printk("trout_rf_test buf buf_len=%d error! \n",buf_len);
        return -EFAULT;
    }

    mutex_lock(p_rf_lock);

    // copy params from user space
	if(copy_from_user(p_param, (char *)buf, buf_len))
	{
        printk("trout_rf_test copy_from_user(buf) failed: len=%d\n.", buf_len);
        len = -EFAULT;
    }
    else
        len = buf_len;
    
	mutex_unlock(p_rf_lock);

if(1)
{
    int i;
    printk("t %d\n",p_param->type);
    printk("c %d\n",p_param->channel);
    printk("p %d\n",p_param->phy_mode);
    printk("s %d\n",p_param->tx_pkt_size);
    printk("n %d\n",p_param->tx_pkt_num);
    printk("r %d\n",p_param->rate);
    printk("m %d\n",p_param->power);
    printk("f %d\n",p_param->freq_offset);
    printk("nonstop %d\n",p_param->nonstop_work);
    for(i=0; i<6; i++)
        printk("%02x:",p_param->dmac[i]);
    printk("\n");
}

    if(len<0) 
        return len;
    
    if(p_param->magic != rf_param_st_magic)
    {
        printk("trout_rf_test magic error!\n");
        return -EFAULT;
    }
    g_rf_test_rsp = response;

    trout_rf_work_flag = 1;

    //schedule_rf_test_work();
    ret = queue_delayed_work(tp->dwork_wq, &tp->dwork, msecs_to_jiffies(50));  // 50 ms
    printk(KERN_INFO "queue_delayed_work ret=%d!\n", ret);  

    printk("trout_rf_test_set_param done.\n");
    return len;
}
EXPORT_SYMBOL(trout_rf_test_set_param);

int trout_rf_test_init(struct net_device *dev)
{
    struct trout_private *tp = netdev_priv(dev);
    struct rf_test_param *p_param = &tp->rf_param;
    struct mutex *p_rf_lock = &tp->rf_lock;
    static int rf_test_mode_init_flag = 0;

    if(rf_test_mode_init_flag == 0)
    {
        //tp->dwork_wq = create_workqueue("delaywork_wq");
        tp->dwork_wq = create_singlethread_workqueue("delaywork_wq");
        if (!tp->dwork_wq) {  
            printk(KERN_ERR "error! No memory for tp->dwork_wq workqueue!\n");  
            return -1;   
        }
    	INIT_DELAYED_WORK(&(tp->dwork), do_rf_test_work);
        
        rf_test_mode_init_flag++;
    }
    
    memset(p_param, 0, sizeof(struct rf_test_param));
    mutex_init(p_rf_lock);
    
    g_rf_test_pkt_len = 0;
    g_rf_test_rsp = 0;
    
    g_rf_test_log = (char *)kmalloc(max_rf_test_log_len, GFP_KERNEL);
	if(g_rf_test_log == NULL)
	{
		printk(KERN_ERR "mem_alloc for g_rf_test_log failed!\n");
		return -1;
    }
    memset(g_rf_test_log, '\0', max_rf_test_log_len);
    g_rf_test_log_len = 1;
    
    printk("trout_rf_test_init done.\n");
    return 0;
}

void trout_rf_test_release(struct net_device *dev)
{
    struct trout_private *tp = netdev_priv(dev);
    struct rf_test_param *p_param = &tp->rf_param;

    {
        int ret = cancel_delayed_work(&tp->dwork);  
        printk(KERN_INFO "cancel_delayed_work! ret=%d\n", ret); 
    }
    flush_workqueue(tp->dwork_wq);  
    destroy_workqueue(tp->dwork_wq);  
    
    memset(p_param, 0, sizeof(struct rf_test_param));

	if(g_rf_test_pkt != NULL)
	{
	    kfree(g_rf_test_pkt);
        g_rf_test_pkt = NULL;
        printk("trout_rf_test_release bad g_rf_test_pkt!\n");
	}
    
    g_rf_test_pkt_len = 0;
    g_rf_test_rsp = 0;
    
	if(g_rf_test_log != NULL)
	{
	    kfree(g_rf_test_log);
        g_rf_test_log = NULL;
	}
    g_rf_test_log_len = 0;

    printk("trout_rf_test_release done.\n");
}


#endif /* #ifdef TROUT_WIFI_NPI */
