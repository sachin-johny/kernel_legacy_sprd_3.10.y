/*******************************************************************************
* Copyright (c) 2011-2012,Spreadtrum Corporation
* All rights reserved.
* Filename: mac_reg_dbg.c
* Abstract: 
* 
* Version: 1.0
* Revison Log: 
* 	2012/8/08, Chengwg: Create this file.
* CVS Log: 
* 	$Id$
*******************************************************************************/

#ifdef DEBUG_MODE

#include "common.h"
#include "mh.h"
#include "spi_interface.h"

#define MAC_REG_READ(addr)	host_read_trout_reg(addr)

INLINE void reg_dbg(UWORD8 *str, UWORD32 reg, UWORD32 val)
{
	PRINTK("%*s(0x%04X) = 0x%X\n", -42, str, reg, val);
}

void display_general_register(void)
{
	PRINTD("\n===============general register===============\n");
	reg_dbg("PA Control", rMAC_PA_CON, MAC_REG_READ(rMAC_PA_CON));
	reg_dbg("PA Status", rMAC_PA_STAT, MAC_REG_READ(rMAC_PA_STAT));
	reg_dbg("PRBS Seed Value", rMAC_PRBS_SEED_VAL, MAC_REG_READ(rMAC_PRBS_SEED_VAL));
	reg_dbg("DMA Burst Size", rMAC_PA_DMA_BURST_SIZE, MAC_REG_READ(rMAC_PA_DMA_BURST_SIZE));
	reg_dbg("TX-RX Complete Count", rMAC_TX_RX_COMPLETE_CNT, MAC_REG_READ(rMAC_TX_RX_COMPLETE_CNT));
	reg_dbg("PRBS Read Control", rMAC_PRBS_READ_CTRL, MAC_REG_READ(rMAC_PRBS_READ_CTRL));
	reg_dbg("Null frame rate", rMAC_NULL_FRAME_RATE, MAC_REG_READ(rMAC_NULL_FRAME_RATE));
	reg_dbg("NULL Frame PHY TX Mode", rMAC_NULL_FRAME_PHY_TX_MODE, MAC_REG_READ(rMAC_NULL_FRAME_PHY_TX_MODE));
	reg_dbg("Test Mode", rMAC_TEST_MODE, MAC_REG_READ(rMAC_TEST_MODE));
	reg_dbg("MAC H/W ID", rMAC_HW_ID, MAC_REG_READ(rMAC_HW_ID));
	reg_dbg("MAC H/W Reset control", rMAC_RESET_CTRL, MAC_REG_READ(rMAC_RESET_CTRL));
	reg_dbg("TX abort frame duration - timeout value", rMAC_TX_ABORT_FRM_DUR_TIMEOUT, MAC_REG_READ(rMAC_TX_ABORT_FRM_DUR_TIMEOUT));
	reg_dbg("TX abort frame rate", rMAC_TX_ABORT_FRM_RATE, MAC_REG_READ(rMAC_TX_ABORT_FRM_RATE));
	reg_dbg("TX abort Frame PHY TX Mode", rMAC_TX_ABORT_FRM_PHY_TX_MODE, MAC_REG_READ(rMAC_TX_ABORT_FRM_PHY_TX_MODE));
	reg_dbg("Extended PA control", rMAC_EXTENDED_PA_CON, MAC_REG_READ(rMAC_EXTENDED_PA_CON));
}


void display_reception_register(void)
{
	PRINTD("\n===============reception register===============\n");
	reg_dbg("Received Frame Filter", rMAC_RX_FRAME_FILTER, MAC_REG_READ(rMAC_RX_FRAME_FILTER));
	reg_dbg("Frame Control", rMAC_FRAME_CON, MAC_REG_READ(rMAC_FRAME_CON));
	reg_dbg("RX Buffer Address", rMAC_RX_BUFF_ADDR, MAC_REG_READ(rMAC_RX_BUFF_ADDR));
	reg_dbg("FCS Fail Count", rMAC_FCS_FAIL_COUNT, MAC_REG_READ(rMAC_FCS_FAIL_COUNT));
	reg_dbg("RX Maximum Frame Length Filter", rMAC_RXMAXLEN_FILT, MAC_REG_READ(rMAC_RXMAXLEN_FILT));
	reg_dbg("Duplicate Frame Count", rMAC_DUP_DET_COUNT, MAC_REG_READ(rMAC_DUP_DET_COUNT));
	reg_dbg("RX END count register", rMAC_RX_END_COUNT, MAC_REG_READ(rMAC_RX_END_COUNT));
	reg_dbg("RX END error count register", rMAC_RX_ERROR_END_COUNT, MAC_REG_READ(rMAC_RX_ERROR_END_COUNT));
	reg_dbg("AMPDU received count", rMAC_AMPDU_RXD_COUNT, MAC_REG_READ(rMAC_AMPDU_RXD_COUNT));
	reg_dbg("MPDU in received AMPDU Count", rMAC_RX_MPDUS_IN_AMPDU_COUNT, MAC_REG_READ(rMAC_RX_MPDUS_IN_AMPDU_COUNT));
	reg_dbg("Received octets in AMPDU Count", rMAC_RX_BYTES_IN_AMPDU_COUNT, MAC_REG_READ(rMAC_RX_BYTES_IN_AMPDU_COUNT));
	reg_dbg("AMPDU delimiter CRC error Count", rMAC_AMPDU_DLMT_ERROR_COUNT, MAC_REG_READ(rMAC_AMPDU_DLMT_ERROR_COUNT));
	reg_dbg("Defragmentation Lifetime limit", rMAC_RX_LIFETIME_LIMIT, MAC_REG_READ(rMAC_RX_LIFETIME_LIMIT));
	reg_dbg("Hi priority RX Queue buffer address", rMAC_HIP_RX_BUFF_ADDR, MAC_REG_READ(rMAC_HIP_RX_BUFF_ADDR));
	reg_dbg("Hi priority RX Queue control", rMAC_HIP_RXQ_CON, MAC_REG_READ(rMAC_HIP_RXQ_CON));
	reg_dbg("Sub-MSDU gap", rMAC_SUB_MSDU_GAP, MAC_REG_READ(rMAC_SUB_MSDU_GAP));
	reg_dbg("Max Rx Buffer Length", rMAC_MAX_RX_BUFFER_LEN, MAC_REG_READ(rMAC_MAX_RX_BUFFER_LEN));
}


void display_EDCA_register(void)
{
	PRINTD("\n===============EDCA register===============\n");
	reg_dbg("AIFSN", rMAC_AIFSN, MAC_REG_READ(rMAC_AIFSN));
	reg_dbg("CWMinMax AC BK", rMAC_CW_MIN_MAX_AC_BK, MAC_REG_READ(rMAC_CW_MIN_MAX_AC_BK));
	reg_dbg("CWMinMax AC BE", rMAC_CW_MIN_MAX_AC_BE, MAC_REG_READ(rMAC_CW_MIN_MAX_AC_BE));
	reg_dbg("CWMinMax AC VI", rMAC_CW_MIN_MAX_AC_VI, MAC_REG_READ(rMAC_CW_MIN_MAX_AC_VI));
	reg_dbg("CWMinMax AC VO", rMAC_CW_MIN_MAX_AC_VO, MAC_REG_READ(rMAC_CW_MIN_MAX_AC_VO));
	reg_dbg("EDCA TXOP Limit AC BK BE", rMAC_EDCA_TXOP_LIMIT_AC_BKBE, MAC_REG_READ(rMAC_EDCA_TXOP_LIMIT_AC_BKBE));
	reg_dbg("EDCA TXOP Limit AC VI VO", rMAC_EDCA_TXOP_LIMIT_AC_VIVO, MAC_REG_READ(rMAC_EDCA_TXOP_LIMIT_AC_VIVO));
	reg_dbg("AC_BK First Frame Pointer", rMAC_EDCA_PRI_BK_Q_PTR, MAC_REG_READ(rMAC_EDCA_PRI_BK_Q_PTR));
	reg_dbg("AC_BK Q Retry Counts", rMAC_EDCA_PRI_BK_RETRY_CTR, MAC_REG_READ(rMAC_EDCA_PRI_BK_RETRY_CTR));
	reg_dbg("AC_BE First Frame Pointer", rMAC_EDCA_PRI_BE_Q_PTR, MAC_REG_READ(rMAC_EDCA_PRI_BE_Q_PTR));
	reg_dbg("AC_BE Q Retry Counts", rMAC_EDCA_PRI_BE_RETRY_CTR, MAC_REG_READ(rMAC_EDCA_PRI_BE_RETRY_CTR));
	reg_dbg("AC_VI First Frame Pointer", rMAC_EDCA_PRI_VI_Q_PTR, MAC_REG_READ(rMAC_EDCA_PRI_VI_Q_PTR));
	reg_dbg("AC_VI Q Retry Counts", rMAC_EDCA_PRI_VI_RETRY_CTR, MAC_REG_READ(rMAC_EDCA_PRI_VI_RETRY_CTR));
	reg_dbg("AC_VO First Frame Pointer", rMAC_EDCA_PRI_VO_Q_PTR, MAC_REG_READ(rMAC_EDCA_PRI_VO_Q_PTR));
	reg_dbg("AC_VO Q Retry Counts", rMAC_EDCA_PRI_VO_RETRY_CTR, MAC_REG_READ(rMAC_EDCA_PRI_VO_RETRY_CTR));
	reg_dbg("High Priority Q First Frame Pointer", rMAC_EDCA_PRI_HP_Q_PTR, MAC_REG_READ(rMAC_EDCA_PRI_HP_Q_PTR));
	reg_dbg("MSDU MAX TX Lifetime limit", rMAC_TX_MSDU_LIFETIME, MAC_REG_READ(rMAC_TX_MSDU_LIFETIME));
	reg_dbg("AC BK BE EDCA Lifetime limit", rMAC_EDCA_BK_BE_LIFETIME, MAC_REG_READ(rMAC_EDCA_BK_BE_LIFETIME));
	reg_dbg("AC VI VO EDCA Lifetime limit", rMAC_EDCA_VI_VO_LIFETIME, MAC_REG_READ(rMAC_EDCA_VI_VO_LIFETIME));
}


void display_HCCA_STA_register(void)
{
	PRINTD("\n===============HCCA_STA register===============\n");
	reg_dbg("HCCA STA Priority 0 First frame address", rMAC_HC_STA_PRI0_Q_PTR, MAC_REG_READ(rMAC_HC_STA_PRI0_Q_PTR));
	reg_dbg("HCCA STA Priority 1 First frame address", rMAC_HC_STA_PRI1_Q_PTR, MAC_REG_READ(rMAC_HC_STA_PRI1_Q_PTR));
	reg_dbg("HCCA STA Priority 2 First frame address", rMAC_HC_STA_PRI2_Q_PTR, MAC_REG_READ(rMAC_HC_STA_PRI2_Q_PTR));
	reg_dbg("HCCA STA Priority 3 First frame address", rMAC_HC_STA_PRI3_Q_PTR, MAC_REG_READ(rMAC_HC_STA_PRI3_Q_PTR));
	reg_dbg("HCCA STA Priority 4 First frame address", rMAC_HC_STA_PRI4_Q_PTR, MAC_REG_READ(rMAC_HC_STA_PRI4_Q_PTR));
	reg_dbg("HCCA STA Priority 5 First frame address", rMAC_HC_STA_PRI5_Q_PTR, MAC_REG_READ(rMAC_HC_STA_PRI5_Q_PTR));
	reg_dbg("HCCA STA Priority 6 First frame address", rMAC_HC_STA_PRI6_Q_PTR, MAC_REG_READ(rMAC_HC_STA_PRI6_Q_PTR));
	reg_dbg("HCCA STA Priority 7 First frame address", rMAC_HC_STA_PRI7_Q_PTR, MAC_REG_READ(rMAC_HC_STA_PRI7_Q_PTR));
}


void display_TSF_register(void)
{
	PRINTD("\n===============TSF register===============\n");
	reg_dbg("TSF Control", rMAC_TSF_CON, MAC_REG_READ(rMAC_TSF_CON));
	reg_dbg("TSF Timer Read Value High", rMAC_TSF_TIMER_HI, MAC_REG_READ(rMAC_TSF_TIMER_HI));
	reg_dbg("TSF Timer Read Value Low", rMAC_TSF_TIMER_LO, MAC_REG_READ(rMAC_TSF_TIMER_LO));
	reg_dbg("Beacon Period", rMAC_BEACON_PERIOD, MAC_REG_READ(rMAC_BEACON_PERIOD));
	reg_dbg("DTIM Period", rMAC_DTIM_PERIOD, MAC_REG_READ(rMAC_DTIM_PERIOD));
	reg_dbg("Beacon Packet Pointer", rMAC_BEACON_POINTER, MAC_REG_READ(rMAC_BEACON_POINTER));
	reg_dbg("Beacon Transmit Parameters", rMAC_BEACON_TX_PARAMS, MAC_REG_READ(rMAC_BEACON_TX_PARAMS));
	reg_dbg("DTIM Count Status", rMAC_DTIM_COUNT, MAC_REG_READ(rMAC_DTIM_COUNT));
	reg_dbg("DTIM Count update control", rMAC_AP_DTIM_COUNT, MAC_REG_READ(rMAC_AP_DTIM_COUNT));
	reg_dbg("Beacon PHY TX Mode", rMAC_BEACON_PHY_TX_MODE, MAC_REG_READ(rMAC_BEACON_PHY_TX_MODE));
}


void display_protection_SIFS_response_register(void)
{
	PRINTD("\n===============Prot, Frag, Aggr and SIFS Resp registers===============\n");
	reg_dbg("Protection Control", rMAC_PROT_CON, MAC_REG_READ(rMAC_PROT_CON));
	reg_dbg("RTS Threshold", rMAC_RTS_THRESH, MAC_REG_READ(rMAC_RTS_THRESH));
	reg_dbg("Protection Data Rate", rMAC_PROT_RATE, MAC_REG_READ(rMAC_PROT_RATE));
	reg_dbg("TXOP Holder Address Low", rMAC_TXOP_HOLDER_ADDR_LO, MAC_REG_READ(rMAC_TXOP_HOLDER_ADDR_LO));
	reg_dbg("TXOP Holder Address High", rMAC_TXOP_HOLDER_ADDR_HI, MAC_REG_READ(rMAC_TXOP_HOLDER_ADDR_HI));
	reg_dbg("Fragmentation Control", rMAC_FRAG_THRESH, MAC_REG_READ(rMAC_FRAG_THRESH));
	reg_dbg("Protection TX Mode", rMAC_PROT_TX_MODE, MAC_REG_READ(rMAC_PROT_TX_MODE));
	reg_dbg("HT Control register", rMAC_HT_CTRL, MAC_REG_READ(rMAC_HT_CTRL));
	reg_dbg("RA ¨C AMPDU specific LUT control register", rMAC_AMPDU_LUT_CTRL, MAC_REG_READ(rMAC_AMPDU_LUT_CTRL));
	reg_dbg("Transmitted AMPDU Count", rMAC_AMPDU_TXD_COUNT, MAC_REG_READ(rMAC_AMPDU_TXD_COUNT));
	reg_dbg("Transmitted MPDUs in AMPDU Count", rMAC_TX_MPDUS_IN_AMPDU_COUNT, MAC_REG_READ(rMAC_TX_MPDUS_IN_AMPDU_COUNT));
	reg_dbg("Transmitted Octets in AMPDU Count", rMAC_TX_BYTES_IN_AMPDU_COUNT, MAC_REG_READ(rMAC_TX_BYTES_IN_AMPDU_COUNT));
}

void display_channel_access_mgmt_register(void)
{
	PRINTD("\n===============channel access timer mgmt registers===============\n");
	reg_dbg("Slot Time", rMAC_SLOT_TIME, MAC_REG_READ(rMAC_SLOT_TIME));
	reg_dbg("SIFS Time", rMAC_SIFS_TIME, MAC_REG_READ(rMAC_SIFS_TIME));
	reg_dbg("EIFS Time", rMAC_EIFS_TIME, MAC_REG_READ(rMAC_EIFS_TIME));
	reg_dbg("PPDU MAX Time", rMAC_PPDU_MAX_TIME, MAC_REG_READ(rMAC_PPDU_MAX_TIME));
	reg_dbg("Secondary channel slot count", rMAC_SEC_CHAN_SLOT_COUNT, MAC_REG_READ(rMAC_SEC_CHAN_SLOT_COUNT));
	reg_dbg("SIFS Time2", rMAC_SIFS_TIME2, MAC_REG_READ(rMAC_SIFS_TIME2));
	reg_dbg("RIFS Time control register", rMAC_RIFS_TIME_CONTROL_REG, MAC_REG_READ(rMAC_RIFS_TIME_CONTROL_REG));
}

void display_retry_register(void)
{
	PRINTD("\n===============retry registers===============\n");
	reg_dbg("Long Retry Limit", rMAC_LONG_RETRY_LIMIT, MAC_REG_READ(rMAC_LONG_RETRY_LIMIT));
	reg_dbg("Short Retry Limit", rMAC_SHORT_RETRY_LIMIT, MAC_REG_READ(rMAC_SHORT_RETRY_LIMIT));
}


void display_sequence_number_register(void)
{
	PRINTD("\n===============sequence number registers===============\n");
	reg_dbg("Sequence Number Control", rMAC_SEQ_NUM_CON, MAC_REG_READ(rMAC_SEQ_NUM_CON));
	reg_dbg("STA Address High", rMAC_STA_ADDR_HI, MAC_REG_READ(rMAC_STA_ADDR_HI));
	reg_dbg("STA Address Low", rMAC_STA_ADDR_LO, MAC_REG_READ(rMAC_STA_ADDR_LO));
	reg_dbg("Transmit Sequence Number", rMAC_TX_SEQ_NUM, MAC_REG_READ(rMAC_TX_SEQ_NUM));
}

void display_PCF_register(void)
{
	PRINTD("\n===============PCF registers===============\n");
	reg_dbg("PCF Control", rMAC_PCF_CON, MAC_REG_READ(rMAC_PCF_CON));
	reg_dbg("CFP Max Duration", rMAC_CFP_MAX_DUR, MAC_REG_READ(rMAC_CFP_MAX_DUR));
	reg_dbg("CFP Interval", rMAC_CFP_INTERVAL, MAC_REG_READ(rMAC_CFP_INTERVAL));
	reg_dbg("CFP Parameter Set Element Byte Number", rMAC_CFP_PARAM_SET_BYTE_NUM, MAC_REG_READ(rMAC_CFP_PARAM_SET_BYTE_NUM));
	reg_dbg("Medium Occupancy Limit", rMAC_MEDIUM_OCCUPANCY, MAC_REG_READ(rMAC_MEDIUM_OCCUPANCY));
	reg_dbg("PCF Queue Pointer", rMAC_PCF_Q_PTR, MAC_REG_READ(rMAC_PCF_Q_PTR));
	reg_dbg("CFP Count", rMAC_CFP_COUNT, MAC_REG_READ(rMAC_CFP_COUNT));
	reg_dbg("CFP Unused Duration", rMAC_UNUSED_CFP_DUR, MAC_REG_READ(rMAC_UNUSED_CFP_DUR));
}


void display_power_mgmt_register(void)
{
	PRINTD("\n===============power mgmt registers===============\n");
	reg_dbg("Power Management Control", rMAC_PM_CON, MAC_REG_READ(rMAC_PM_CON));
	reg_dbg("ATIM Window", rMAC_ATIM_WINDOW, MAC_REG_READ(rMAC_ATIM_WINDOW));
	reg_dbg("Listen Interval", rMAC_LISTEN_INTERVAL, MAC_REG_READ(rMAC_LISTEN_INTERVAL));
	reg_dbg("Offset Interval", rMAC_OFFSET_INTERVAL, MAC_REG_READ(rMAC_OFFSET_INTERVAL));
	reg_dbg("S-APSD SSP", rMAC_S_APSD_SSP, MAC_REG_READ(rMAC_S_APSD_SSP));
	reg_dbg("S-APSD SI", rMAC_S_APSD_SI, MAC_REG_READ(rMAC_S_APSD_SI));
	reg_dbg("SMPS Control", rMAC_SMPS_CONTROL, MAC_REG_READ(rMAC_SMPS_CONTROL));
}

void display_interrupt_register(void)
{
	PRINTD("\n===============interrupt registers===============\n");
	reg_dbg("Interrupt Status", rMAC_INT_STAT, MAC_REG_READ(rMAC_INT_STAT));
	reg_dbg("Interrupt Mask", rMAC_INT_MASK, MAC_REG_READ(rMAC_INT_MASK));
	reg_dbg("TX Frame Pointer", rMAC_TX_FRAME_POINTER, MAC_REG_READ(rMAC_TX_FRAME_POINTER));
	reg_dbg("RX Frame Pointer", rMAC_RX_FRAME_POINTER, MAC_REG_READ(rMAC_RX_FRAME_POINTER));
	reg_dbg("Error Code", rMAC_ERROR_CODE, MAC_REG_READ(rMAC_ERROR_CODE));
	reg_dbg("TX MPDU Count register", rMAC_TX_MPDU_COUNT, MAC_REG_READ(rMAC_TX_MPDU_COUNT));
	reg_dbg("RX MPDU Count register", rMAC_RX_MPDU_COUNT, MAC_REG_READ(rMAC_RX_MPDU_COUNT));
	reg_dbg("Hi Priority RX Frame Pointer", rMAC_HIP_RX_FRAME_POINTER, MAC_REG_READ(rMAC_HIP_RX_FRAME_POINTER));
	reg_dbg("De-authentication reason code", rMAC_DEAUTH_REASON_CODE, MAC_REG_READ(rMAC_DEAUTH_REASON_CODE));
	reg_dbg("Error Interrupt Status", rMAC_ERROR_STAT, MAC_REG_READ(rMAC_ERROR_STAT));
	reg_dbg("Error Interrupt Mask", rMAC_ERROR_MASK, MAC_REG_READ(rMAC_ERROR_MASK));
}


void display_phy_interface_register(void)
{
	PRINTD("\n===============phy interface and param registers===============\n");
	reg_dbg("PHY Register Access Control", rMAC_PHY_REG_ACCESS_CON, MAC_REG_READ(rMAC_PHY_REG_ACCESS_CON));
	reg_dbg("PHY Register Read/Write data", rMAC_PHY_REG_RW_DATA, MAC_REG_READ(rMAC_PHY_REG_RW_DATA));
	reg_dbg("PHY RF Register Base Address", rMAC_PHY_RF_REG_BASE_ADDR, MAC_REG_READ(rMAC_PHY_RF_REG_BASE_ADDR));
	reg_dbg("TxPLCP Delay", rMAC_TXPLCP_DELAY, MAC_REG_READ(rMAC_TXPLCP_DELAY));
	reg_dbg("RxPLCP Delay", rMAC_RXPLCP_DELAY, MAC_REG_READ(rMAC_RXPLCP_DELAY));
	reg_dbg("RxTxTurnaround Time", rMAC_RXTXTURNAROUND_TIME, MAC_REG_READ(rMAC_RXTXTURNAROUND_TIME));
	reg_dbg("PHY Timeout Adjust Value", rMAC_PHY_TIMEOUT_ADJUST, MAC_REG_READ(rMAC_PHY_TIMEOUT_ADJUST));
	reg_dbg("PHY TX Service field", rMAC_PHY_SERVICE_FIELD, MAC_REG_READ(rMAC_PHY_SERVICE_FIELD));
	reg_dbg("TX Power Level", rMAC_PHY_TX_PWR_SET_REG, MAC_REG_READ(rMAC_PHY_TX_PWR_SET_REG));
	reg_dbg("PHY CCA Delay", rMAC_PHY_CCA_DELAY, MAC_REG_READ(rMAC_PHY_CCA_DELAY));
	reg_dbg("TX PLCP Adjust value", rMAC_TXPLCP_ADJUST_VAL, MAC_REG_READ(rMAC_TXPLCP_ADJUST_VAL));
	reg_dbg("Rx PLCP Delay 2", rMAC_RXPLCP_DELAY2, MAC_REG_READ(rMAC_RXPLCP_DELAY2));
	reg_dbg("Rx Start Delay Register", rMAC_RXSTART_DELAY_REG, MAC_REG_READ(rMAC_RXSTART_DELAY_REG));
	reg_dbg("Antenna Set Register 1", rMAC_ANTENNA_SET, MAC_REG_READ(rMAC_ANTENNA_SET));
}

void display_block_ack_register(void)
{
	PRINTD("\n===============Block-Ack registers===============\n");
	reg_dbg("BA Control Register", rMAC_BA_CTRL, MAC_REG_READ(rMAC_BA_CTRL));
	reg_dbg("Peer STA Address MSB", rMAC_BA_PEER_STA_ADDR_MSB, MAC_REG_READ(rMAC_BA_PEER_STA_ADDR_MSB));
	reg_dbg("Peer STA Address LSB", rMAC_BA_PEER_STA_ADDR_LSB, MAC_REG_READ(rMAC_BA_PEER_STA_ADDR_LSB));
	reg_dbg("BA Parameters Register", rMAC_BA_PARAMS, MAC_REG_READ(rMAC_BA_PARAMS));
	reg_dbg("BA Compressed Bitmap MSW", rMAC_BA_CBMAP_MSW, MAC_REG_READ(rMAC_BA_CBMAP_MSW));
	reg_dbg("BA Compressed Bitmap LSW", rMAC_BA_CBMAP_LSW, MAC_REG_READ(rMAC_BA_CBMAP_LSW));
}

// add by Ke.Li at 2013-04-11 for itm_config read ram
void display_trout_ram(WORD32 start_addr, WORD32 read_len)
{
    UWORD32 len = 0;
    WORD32 i = 0;
    UWORD32 addr_num = 0;
    UWORD8 data[1024];
    WORD32 display_addr = 0;
    UWORD8 *pdata = NULL;
    WORD32 ret = 0;

    // made address be align for 16 bytes
    addr_num = start_addr % 16;
    start_addr -= addr_num;
    len = read_len + addr_num;
    // made read length be align for 16 bytes
    addr_num = 16 - (len % 16);
    if(addr_num == 16)
    {
        addr_num = 0;
    }
    len += addr_num;
    // made max length be 1024 bytes
    if(len > 1024)
    {
        len = 1024;
    }

    // read ram data to array
    ret = host_read_trout_ram(data, (UWORD8 *)start_addr, len);
    
    // print data
    PRINTD("\n ============================  DUMP RAM DATA  ===========================\n\n");
    if(ret == 0)
    {
        for(i = 0; i < len; i+=16)
        {
            display_addr = start_addr + i;
            pdata = data + i;
            PRINTD("  0x%08x:   %02x %02x %02x %02x   %02x %02x %02x %02x   %02x %02x %02x %02x   %02x %02x %02x %02x\n",
                display_addr, *pdata, *(pdata+1), *(pdata+2), *(pdata+3), *(pdata+4), *(pdata+5), *(pdata+6), *(pdata+7),
                *(pdata+8), *(pdata+9), *(pdata+10), *(pdata+11), *(pdata+12), *(pdata+13), *(pdata+14), *(pdata+15));
        }
    }
    else
    {
        PRINTD("  trout ram address or length is out of range!\n");
    }
    PRINTD("\n ========================================================================\n\n");
}
// end add by Ke.Li at 2013-04-11 for itm_config read ram
#endif	/* DEBUG_MODE */

