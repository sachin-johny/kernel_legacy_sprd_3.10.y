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
/*  File Name         : mh.h                                                 */
/*                                                                           */
/*  Description       : This file contains the definitions and function      */
/*                      prototypes required for MAC hardware interface.      */
/*                                                                           */
/*  List of Functions : Access functions for all MAC Hardware registers.     */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef MH_H
#define MH_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include <linux/time.h>
#include "common.h"
#include "csl_if.h"
#include "mib.h"
#include "phy_hw_if.h"
#include "phy_prot_if.h"
#include "rf_if.h"
#include "spi_interface.h"
#include "mac_init.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define MAX_BA_LUT_SIZE              16
#define MAX_AMPDU_LUT_SIZE           255

/* 11N TBD - currently the default power level value is used to set power    */
/* level for beacon and protection frames. Can be varied in the future.      */
#define DEFAULT_POWER_LEVEL          52

/* The default Minimum TXOP Fragment Length */
#define DEFAULT_MIN_TXOP_FRAG_LENGTH         256

/* Timeout for PHY Register Update in units of 10us */
#define PHY_REG_RW_TIMEOUT  1000 /* 10ms */

/*****************************************************************************/
/* Macros                                                                    */
/*****************************************************************************/

#ifdef LITTLE_ENDIAN

#define MASK_INVERSE(len, offset) ((UWORD32)(~(((1 << (len)) - 1) << (offset))))
#define MASK(len, offset)         ((UWORD32)(((1 << (len)) - 1) << (offset)))

#endif /* LITTLE_ENDIAN */

#ifdef BIG_ENDIAN

#define MASK_INVERSE(len, offset) ((UWORD32)(SWAP_BYTE_ORDER_WORD(~(((1 << (len)) - 1) << (offset)))))
#define MASK(len, offset)         ((UWORD32)(SWAP_BYTE_ORDER_WORD(((1 << (len)) - 1) << (offset))))

#endif /* BIG_ENDIAN */

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
/* POWER SLEEP INFO MAGIC */
#define PS_MSG_HOST_SEND_MAGIC          0xf501 // host send msg
#define PS_MSG_HOST_EARLY_AWAKE_MAGIC   0xf531 // host early suspend awake notice
#define PS_MSG_ARM7_SEND_MAGIC          0xf502 // arm7 send msg
#define PS_MSG_ARM7_HANDLE_TBTT_MAGIC   0xf541 // host want arm7 to handle TBTT INTR
#define PS_MSG_HOST_HANDLE_TBTT_MAGIC   0xf542 // host want to handle TBTT INTR himself
#define PS_MSG_ARM7_SBEA_KC_MAGIC       0xf543 // ARM7 start to keep connection
#define PS_MSG_ARM7_EBEA_KC_MAGIC       0xf544 // ARM7 finish to keep connection
#define PS_MSG_WIFI_SUSPEND_MAGIC	0xf545 // host make Wi-Fi sleep
#define PS_MSG_WIFI_RESUME_MAGIC	0xf546 // host wake up Wi-Fi
#define PS_MSG_HOST_GET_READY_FOR_TBTT_MAGIC   0xf551 // arm7 notify host to get ready for TBTT
#define PS_MSG_ARM7_SIMULATE_HIGH_RX_INT_MAGIC 0xF552 //arm7 simulate a mac high rx int and want to mac rcv mgmt frame by itself.

#endif
/*****************************************************************************/
/* Protocol Accelerator Register Addresses                                   */
/*****************************************************************************/

/*****************************************************************************/
/* General Registers                                                         */
/*****************************************************************************/

#define rMAC_PA_VER                   (PA_BASE + 0x0000)
#define rMAC_PA_CON                   (PA_BASE + 0x0004)
#define rMAC_PA_STAT                  (PA_BASE + 0x0008)
#define rMAC_ADDR_HI                  (PA_BASE + 0x000C)
#define rMAC_ADDR_LO                  (PA_BASE + 0x0010)
#define rMAC_BSSID_HI                 (PA_BASE + 0x0014)
#define rMAC_BSSID_LO                 (PA_BASE + 0x0018)
#define rMAC_PRBS_SEED_VAL            (PA_BASE + 0x001C)
#define rMAC_PA_DMA_BURST_SIZE        (PA_BASE + 0x0020)
#define rMAC_TX_RX_COMPLETE_CNT       (PA_BASE + 0x0024)
#define rMAC_PRBS_READ_CTRL           (PA_BASE + 0x0028)
#define rMAC_NULL_FRAME_RATE          (PA_BASE + 0x002C)
#define rMAC_NULL_FRAME_PHY_TX_MODE   (PA_BASE + 0x0030)
#define rMAC_TEST_MODE                (PA_BASE + 0x0034)
#define rMAC_HW_ID                    (PA_BASE + 0x0038)
#define rMAC_RESET_CTRL               (PA_BASE + 0x003C)
#define rMAC_TX_ABORT_FRM_DUR_TIMEOUT (PA_BASE + 0x0040)
#define rMAC_TX_ABORT_FRM_RATE        (PA_BASE + 0x0044)
#define rMAC_TX_ABORT_FRM_PHY_TX_MODE (PA_BASE + 0x0048)
#define rMAC_EXTENDED_PA_CON          (PA_BASE + 0x004C)

/*dumy add for Rx Counter Registers*/

//ACK/CTS/RTS/FCS fail/etc(detail in Received Frame Filter Register-0x80) Filter Counter when RX Frame Done
#define rMAC_RX_FRAME_FILTER_COUNTER        (PA_BASE + 0x0050)

//Address etc filter counter(detail in Received Frame Filter Register) after RX MAC Header done
#define rMAC_RX_MAC_HEADER_FILTER_COUNTER   (PA_BASE + 0x0054)

//the counter for the losted frames when the Rx Q is Full.
#define rMAC_RXQ_FULL_COUNTER          (PA_BASE + 0x0058)

//the counter for the rx frames into the rx q.
#define rMAC_RX_RAM_PACKET_COUNTER          (PA_BASE + 0x00C4)

/*****************************************************************************/
/* Reception Registers                                                       */
/*****************************************************************************/

#define rMAC_RX_FRAME_FILTER          (PA_BASE + 0x0080)
#define rMAC_FRAME_CON                (PA_BASE + 0x0084)
#define rMAC_RX_BUFF_ADDR             (PA_BASE + 0x0088)
#define rMAC_FCS_FAIL_COUNT           (PA_BASE + 0x008C)
#define rMAC_RXMAXLEN_FILT            (PA_BASE + 0x0090)
#define rMAC_DUP_DET_COUNT            (PA_BASE + 0x0094)
#define rMAC_RX_END_COUNT             (PA_BASE + 0x0098)
#define rMAC_RX_ERROR_END_COUNT       (PA_BASE + 0x009C)
#define rMAC_AMPDU_RXD_COUNT          (PA_BASE + 0x00A0)
#define rMAC_RX_MPDUS_IN_AMPDU_COUNT  (PA_BASE + 0x00A4)
#define rMAC_RX_BYTES_IN_AMPDU_COUNT  (PA_BASE + 0x00A8)
#define rMAC_AMPDU_DLMT_ERROR_COUNT   (PA_BASE + 0x00AC)
#define rMAC_RX_LIFETIME_LIMIT        (PA_BASE + 0x00B0)
#define rMAC_HIP_RX_BUFF_ADDR         (PA_BASE + 0x00B4)
#define rMAC_HIP_RXQ_CON              (PA_BASE + 0x00B8)
#define rMAC_SUB_MSDU_GAP             (PA_BASE + 0x00BC)
#define rMAC_MAX_RX_BUFFER_LEN        (PA_BASE + 0x00C0)

/*****************************************************************************/
/* EDCA Registers                                                            */
/*****************************************************************************/

#define rMAC_AIFSN                     (PA_BASE + 0x0100)
#define rMAC_CW_MIN_MAX_AC_BK          (PA_BASE + 0x0104)
#define rMAC_CW_MIN_MAX_AC_BE          (PA_BASE + 0x0108)
#define rMAC_CW_MIN_MAX_AC_VI          (PA_BASE + 0x010C)
#define rMAC_CW_MIN_MAX_AC_VO          (PA_BASE + 0x0110)
#define rMAC_EDCA_TXOP_LIMIT_AC_BKBE   (PA_BASE + 0x0114)
#define rMAC_EDCA_TXOP_LIMIT_AC_VIVO   (PA_BASE + 0x0118)
#define rMAC_EDCA_PRI_BK_Q_PTR         (PA_BASE + 0x011C)
#define rMAC_EDCA_PRI_BK_RETRY_CTR     (PA_BASE + 0x0120)
#define rMAC_EDCA_PRI_BE_Q_PTR         (PA_BASE + 0x0124)
#define rMAC_EDCA_PRI_BE_RETRY_CTR     (PA_BASE + 0x0128)
#define rMAC_EDCA_PRI_VI_Q_PTR         (PA_BASE + 0x012C)
#define rMAC_EDCA_PRI_VI_RETRY_CTR     (PA_BASE + 0x0130)
#define rMAC_EDCA_PRI_VO_Q_PTR         (PA_BASE + 0x0134)
#define rMAC_EDCA_PRI_VO_RETRY_CTR     (PA_BASE + 0x0138)
#define rMAC_EDCA_PRI_HP_Q_PTR         (PA_BASE + 0x013C)
#define rMAC_TX_MSDU_LIFETIME          (PA_BASE + 0x0140)
#define rMAC_EDCA_BK_BE_LIFETIME       (PA_BASE + 0x0144)
#define rMAC_EDCA_VI_VO_LIFETIME       (PA_BASE + 0x0148)

/*****************************************************************************/
/* HCCA STA Registers                                                        */
/*****************************************************************************/

#define rMAC_HC_STA_PRI0_Q_PTR        (PA_BASE + 0x0180)
#define rMAC_HC_STA_PRI1_Q_PTR        (PA_BASE + 0x0184)
#define rMAC_HC_STA_PRI2_Q_PTR        (PA_BASE + 0x0188)
#define rMAC_HC_STA_PRI3_Q_PTR        (PA_BASE + 0x018C)
#define rMAC_HC_STA_PRI4_Q_PTR        (PA_BASE + 0x0190)
#define rMAC_HC_STA_PRI5_Q_PTR        (PA_BASE + 0x0194)
#define rMAC_HC_STA_PRI6_Q_PTR        (PA_BASE + 0x0198)
#define rMAC_HC_STA_PRI7_Q_PTR        (PA_BASE + 0x019C)

/*****************************************************************************/
/* TSF Registers                                                             */
/*****************************************************************************/

#define rMAC_TSF_CON                  (PA_BASE + 0x0200)
#define rMAC_TSF_TIMER_HI             (PA_BASE + 0x0204)
#define rMAC_TSF_TIMER_LO             (PA_BASE + 0x0208)
#define rMAC_BEACON_PERIOD            (PA_BASE + 0x020C)
#define rMAC_DTIM_PERIOD              (PA_BASE + 0x0210)
#define rMAC_BEACON_POINTER           (PA_BASE + 0x0214)
#define rMAC_BEACON_TX_PARAMS         (PA_BASE + 0x0218)
#define rMAC_DTIM_COUNT               (PA_BASE + 0x021C)
#define rMAC_AP_DTIM_COUNT            (PA_BASE + 0x0220)
#define rMAC_BEACON_PHY_TX_MODE       (PA_BASE + 0x0224)
/*****************************************************************************/
/* Protection And SIFS Response Registers                                    */
/*****************************************************************************/

#define rMAC_PROT_CON                 (PA_BASE + 0x0280)
#define rMAC_RTS_THRESH               (PA_BASE + 0x0284)
#define rMAC_PROT_RATE                (PA_BASE + 0x0288)
#define rMAC_TXOP_HOLDER_ADDR_HI      (PA_BASE + 0x028C)
#define rMAC_TXOP_HOLDER_ADDR_LO      (PA_BASE + 0x0290)
#define rMAC_FRAG_THRESH              (PA_BASE + 0x029C)
#define rMAC_PROT_TX_MODE             (PA_BASE + 0x02A0)
#define rMAC_HT_CTRL                  (PA_BASE + 0x02A4)
#define rMAC_AMPDU_LUT_CTRL           (PA_BASE + 0x02A8)
#define rMAC_AMPDU_TXD_COUNT          (PA_BASE + 0x02AC)
#define rMAC_TX_MPDUS_IN_AMPDU_COUNT  (PA_BASE + 0x02B0)
#define rMAC_TX_BYTES_IN_AMPDU_COUNT  (PA_BASE + 0x02B4)

#define rTX_NUM_20MHZ_TXOP               (PA_BASE + 0x02BC)
#define rTX_NUM_40MHZ_TXOP               (PA_BASE + 0x02C0)
#define rTX_NUM_20MHZ_MPDU_IN_40MHZ_TXOP (PA_BASE + 0x02C4)
#define rTX_NUM_PROMOTED_MPDU            (PA_BASE + 0x02C8)
#define rTX_NUM_MPDU_DEMOTED             (PA_BASE + 0x02CC)
#define rTX_NUM_PROMOTED_PROT            (PA_BASE + 0x02D0)
#define rTX_NUM_PROT_DUE_TO_FC           (PA_BASE + 0x02D4)
#define rTX_NUM_TXOP_ABORT_ON_SEC_BUSY   (PA_BASE + 0x02D8)

/*****************************************************************************/
/* Channel Access Timer Management Registers                                 */
/*****************************************************************************/

#define rMAC_SLOT_TIME                (PA_BASE + 0x0300)
#define rMAC_SIFS_TIME                (PA_BASE + 0x0304)
#define rMAC_EIFS_TIME                (PA_BASE + 0x0308)
#define rMAC_PPDU_MAX_TIME            (PA_BASE + 0x030C)
#define rMAC_SEC_CHAN_SLOT_COUNT      (PA_BASE + 0x0310)
#define rMAC_SIFS_TIME2               (PA_BASE + 0x0314)
#define rMAC_RIFS_TIME_CONTROL_REG    (PA_BASE + 0x0318)

/*****************************************************************************/
/* Retry Registers                                                           */
/*****************************************************************************/

#define rMAC_LONG_RETRY_LIMIT         (PA_BASE + 0x0380)
#define rMAC_SHORT_RETRY_LIMIT        (PA_BASE + 0x0384)

/*****************************************************************************/
/* Sequence Number and Duplicate Detection Registers                         */
/*****************************************************************************/

#define rMAC_SEQ_NUM_CON              (PA_BASE + 0x0400)
#define rMAC_STA_ADDR_HI              (PA_BASE + 0x0404)
#define rMAC_STA_ADDR_LO              (PA_BASE + 0x0408)
#define rMAC_TX_SEQ_NUM               (PA_BASE + 0x040C)

/*****************************************************************************/
/* PCF Registers                                                             */
/*****************************************************************************/

#define rMAC_PCF_CON                  (PA_BASE + 0x0480)
#define rMAC_CFP_MAX_DUR              (PA_BASE + 0x0484)
#define rMAC_CFP_INTERVAL             (PA_BASE + 0x0488)
#define rMAC_CFP_PARAM_SET_BYTE_NUM   (PA_BASE + 0x048C)
#define rMAC_MEDIUM_OCCUPANCY         (PA_BASE + 0x0490)
#define rMAC_PCF_Q_PTR                (PA_BASE + 0x0494)
#define rMAC_CFP_COUNT                (PA_BASE + 0x0498)
#define rMAC_UNUSED_CFP_DUR           (PA_BASE + 0x049C)

/*****************************************************************************/
/* Power Management Registers                                                */
/*****************************************************************************/

#define rMAC_PM_CON                   (PA_BASE + 0x0500)
#define rMAC_ATIM_WINDOW              (PA_BASE + 0x0504)
#define rMAC_LISTEN_INTERVAL          (PA_BASE + 0x0508)
#define rMAC_OFFSET_INTERVAL          (PA_BASE + 0x050C)
#define rMAC_S_APSD_SSP               (PA_BASE + 0x0510)
#define rMAC_S_APSD_SI                (PA_BASE + 0x0514)
#define rMAC_SMPS_CONTROL             (PA_BASE + 0x0518)

/*****************************************************************************/
/* Interrupt Registers                                                       */
/*****************************************************************************/

#define rMAC_INT_STAT                 (PA_BASE + 0x0580)
#define rMAC_INT_MASK                 (PA_BASE + 0x0584)
#define rMAC_TX_FRAME_POINTER         (PA_BASE + 0x0588)
#define rMAC_RX_FRAME_POINTER         (PA_BASE + 0x058C)
#define rMAC_ERROR_CODE               (PA_BASE + 0x0590)
#define rMAC_TX_MPDU_COUNT            (PA_BASE + 0x0594)
#define rMAC_RX_MPDU_COUNT            (PA_BASE + 0x0598)
#define rMAC_HIP_RX_FRAME_POINTER     (PA_BASE + 0x059C)
#define rMAC_DEAUTH_REASON_CODE       (PA_BASE + 0x05A0)
#define rMAC_ERROR_STAT               (PA_BASE + 0x05A4)
#define rMAC_ERROR_MASK               (PA_BASE + 0x05A8)

/*****************************************************************************/
/* PHY Interface and Parameters Register                                     */
/*****************************************************************************/

#define rMAC_PHY_REG_ACCESS_CON       (PA_BASE + 0x0638)
#define rMAC_PHY_REG_RW_DATA          (PA_BASE + 0x063C)
#define rMAC_PHY_RF_REG_BASE_ADDR     (PA_BASE + 0x0624)
#define rMAC_TXPLCP_DELAY             (PA_BASE + 0x0628)
#define rMAC_RXPLCP_DELAY             (PA_BASE + 0x062C)
#define rMAC_RXTXTURNAROUND_TIME      (PA_BASE + 0x0630)
#define rMAC_PHY_TIMEOUT_ADJUST       (PA_BASE + 0x0634)
#define rMAC_PHY_SERVICE_FIELD        (PA_BASE + 0x0640)
#define rMAC_PHY_TX_PWR_SET_REG       (PA_BASE + 0x0644)
#define rMAC_PHY_CCA_DELAY            (PA_BASE + 0x0648)
#define rMAC_TXPLCP_ADJUST_VAL        (PA_BASE + 0x064C)
#define rMAC_RXPLCP_DELAY2            (PA_BASE + 0x0650)
#define rMAC_RXSTART_DELAY_REG        (PA_BASE + 0x0654)
#define rMAC_ANTENNA_SET              (PA_BASE + 0x0658)

// 20120709 caisf add, merged ittiam mac v1.2 code
#ifdef MWLAN
#define rMAC_ODDR_CTRL                (PA_BASE + 0x065C)
#endif /* MWLAN */
/*****************************************************************************/
/* Block Ack register address                                                */
/*****************************************************************************/

#define rMAC_BA_CTRL                 (PA_BASE + 0x0698)
#define rMAC_BA_PEER_STA_ADDR_MSB    (PA_BASE + 0x069C)
#define rMAC_BA_PEER_STA_ADDR_LSB    (PA_BASE + 0x06A0)
#define rMAC_BA_PARAMS               (PA_BASE + 0x06A4)
#define rMAC_BA_CBMAP_MSW            (PA_BASE + 0x06A8)
#define rMAC_BA_CBMAP_LSW            (PA_BASE + 0x06AC)

/*****************************************************************************/
/* HCCA AP Registers                                                         */
/*****************************************************************************/

#define rMAC_SCHEDULE_LINK_ADDR       (PA_BASE + 0x0700)
#define rMAC_CAP_START_TIME           (PA_BASE + 0x0704)

/*****************************************************************************/
/* Queue pointer addresses                                                   */
/*****************************************************************************/

#define MAC_EDCA_PRI_BK_Q_PTR         (PA_BASE + 0x011C)
#define MAC_EDCA_PRI_BE_Q_PTR         (PA_BASE + 0x0124)
#define MAC_EDCA_PRI_VI_Q_PTR         (PA_BASE + 0x012C)
#define MAC_EDCA_PRI_VO_Q_PTR         (PA_BASE + 0x0134)
#define MAC_EDCA_PRI_HP_Q_PTR         (PA_BASE + 0x013C)
#define MAC_EDCA_PRI_CF_Q_PTR         (PA_BASE + 0x0494)
#define MAC_HC_STA_PRI0_Q_PTR         (PA_BASE + 0x0180)
#define MAC_HC_STA_PRI1_Q_PTR         (PA_BASE + 0x0184)
#define MAC_HC_STA_PRI2_Q_PTR         (PA_BASE + 0x0188)
#define MAC_HC_STA_PRI3_Q_PTR         (PA_BASE + 0x018C)
#define MAC_HC_STA_PRI4_Q_PTR         (PA_BASE + 0x0190)
#define MAC_HC_STA_PRI5_Q_PTR         (PA_BASE + 0x0194)
#define MAC_HC_STA_PRI6_Q_PTR         (PA_BASE + 0x0198)
#define MAC_HC_STA_PRI7_Q_PTR         (PA_BASE + 0x019C)

/*****************************************************************************/
/* Protocol Accelerator Register Initialization Values                       */
/*****************************************************************************/

/*****************************************************************************/
/* General Registers                                                         */
/*****************************************************************************/

#define MAC_PA_DMA_BURST_SIZE_INIT_VALUE   0x00000004
#define MAC_HW_ID_INIT_VALUE               0x4E4D4143 /* ASCII NMAC */

/*****************************************************************************/
/* Reception Registers                                                       */
/*****************************************************************************/
#ifdef NON_FC_MACHW_SUPPORT
/* 7          6            5          4        3       2       1       0     */
/* NonDir     QCF_POLL     CF_END     ATIM     BCN     RTS     CTS     ACK   */
/* 31-14     13          12              11      10        9       8         */
/* Reserved  ExpectedBA  UnexpectedBA    Deauth  FCSFail   Dup     OtherBSS  */
#ifdef BSS_ACCESS_POINT_MODE
#define MAC_RX_FRAME_FILTER_INIT_VALUE  0x000036FF
#else /* BSS_ACCESS_POINT_MODE */
#define MAC_RX_FRAME_FILTER_INIT_VALUE  0x00003EFF
#endif /* BSS_ACCESS_POINT_MODE */
#else /* NON_FC_MACHW_SUPPORT */

/* 7          6            5          4        3       2       1       0     */
/* NonDirMgmt QCF_POLL     CF_END     ATIM     BCN     RTS     CTS     ACK   */
/* 14       13          12              11      10       9    8              */
/* SecChan  ExpectedBA  UnexpectedBA    Deauth  FCSFail  Dup  BcMcMgmtOBSS   */
/*        31-19     18                17          16          15             */
/*        Reserved  DiscardedIBSSBcn  NonDirCtrl  NonDirData  BcstDataOBSS   */

#ifdef BSS_ACCESS_POINT_MODE
#define MAC_RX_FRAME_FILTER_INIT_VALUE  0x0003F6FF
#else /* BSS_ACCESS_POINT_MODE */
#define MAC_RX_FRAME_FILTER_INIT_VALUE  0x0003FEFF
#endif /* BSS_ACCESS_POINT_MODE */

#endif /* NON_FC_MACHW_SUPPORT */

#define MAC_RX_FRAME_FILTER_ALL_VALUE   0xFFFFFFFF
#define MAC_FRAME_CON_INIT_VALUE        0x00000000
#define MAC_RXMAXLENFILT_INIT_VALUE     RX_BUFFER_SIZE /* Same as Rx buffer */

#define MAC_RX_BUFF_ADDR_INIT_VALUE     ((UWORD32)\
        (g_q_handle.rx_handle.rx_header[NORMAL_PRI_RXQ].element_head))

#define MAC_HIP_RX_BUFF_ADDR_INIT_VALUE ((UWORD32)\
        (g_q_handle.rx_handle.rx_header[HIGH_PRI_RXQ].element_head))

/* 31 - 5     4     3           2          1       0                         */
/* Reserved  ATIM   Probe Rsp   Probe Req  Beacon  HighPriorityQEnable       */
#define MAC_HIP_RXQ_CON_INIT_VALUE  0x0000000F

/*****************************************************************************/
/* Interrupt Registers                                                       */
/*****************************************************************************/

/* 9           8     7      6     5      4    3         2    1       0       */
/* RFVCOUnlock CFEnd CAPEnd Error WakeUp ATIM HCCA TXOP TBTT TX Comp RX Comp */
/* 31 - 16   15     14      13     12           11             10            */
/* Reserved Deauth PATxSus RadDet HwTxAbReqEnd HwTxAbReqStart HIPQRxComp     */
#ifdef BSS_ACCESS_POINT_MODE
#define MAC_INT_MASK_INIT_VALUE 0xFFFFFFFF	//0x0000FFFF  //modified by Hugh
#else /* BSS_ACCESS_POINT_MODE */
#define MAC_INT_MASK_INIT_VALUE 0xFFFFFFFF //0x0000FFFF  //modified by Hugh
#endif /* BSS_ACCESS_POINT_MODE */

/* enable error mask except RX/TX FIFO over flow by zhao 6-25 2013 */
#ifdef ERROR_INT_ENABLE
#define MAC_ERROR_MASK_INIT_VALUE  0x0000E000
#else
#define MAC_ERROR_MASK_INIT_VALUE  0xFFFFFFFF
#endif
/*****************************************************************************/
/* PHY Interface and Parameters Register                                     */
/*****************************************************************************/

#define MAC_PHY_RF_REG_BASE_ADDR_INIT_VALUE  0x00000000
#define MAC_PHY_TIMEOUT_ADJUST_INIT_VALUE    0x0000001F
#define MAC_PHY_SERVICE_FIELD_INIT_VALUE     0x00000000
#define MAC_RX_WATCHDOG_TIMER_INIT_VALUE     0x0000001E
#if 0
#define MAC_RXSTART_DELAY_REG_INIT_VALUE     0x221B6FCF //0x22196FCF - Changed since ACK reception at Non-HT rates was failing
#else
/*junbin.wang, modify for cr 229369, 11b mode, tx rate slow to 1Mbps*/
#define MAC_RXSTART_DELAY_REG_INIT_VALUE     0x221BFFFF  //mengyuan.du mod 2013-04-24,fix 11b ack
//#define MAC_RXSTART_DELAY_REG_INIT_VALUE     0x221B79D9  //zhongli wang modify for ccmc test version 20130906
#endif
// 20120830 caisf masked, merged ittiam mac v1.3 code
//#define MAC_ANTENNA_SET_INIT_VALUE           0x0F070301

/*****************************************************************************/
/* Channel Access Timer Management Registers                                 */
/*****************************************************************************/

#define MAC_SIFS_TIME_INIT_VALUE             0x0000100A
#define MAC_SIFS_TIME2_INIT_VALUE            0x0000A064
#define MAC_RIFS_TIME_CONTROL_REG_INIT_VALUE 0x00000050

#define MAC_SEC_CHAN_SLOT_COUNT_INIT_VAL_FREQ_5     0x00000031
#define MAC_SEC_CHAN_SLOT_COUNT_INIT_VAL_FREQ_2     0x00000032

/*****************************************************************************/
/* Power Management Registers                                                */
/*****************************************************************************/

#define MAC_OFFSET_INTERVAL_INIT_VALUE      0x0000000C

/*****************************************************************************/
/* CE Register Addresses                                                     */
/*****************************************************************************/

#define rMAC_CE_KEY_FIRST             (CE_BASE + 0x0000)
#define rMAC_CE_KEY_SECOND            (CE_BASE + 0x0004)
#define rMAC_CE_KEY_THIRD             (CE_BASE + 0x0008)
#define rMAC_CE_KEY_FOURTH            (CE_BASE + 0x000C)
#define rMAC_CE_MAC_ADDR_MSB          (CE_BASE + 0x0010)
#define rMAC_CE_MAC_ADDR_LSB          (CE_BASE + 0x0014)
#define rMAC_CE_STA_ADDR_MSB          (CE_BASE + 0x0018)
#define rMAC_CE_STA_ADDR_LSB          (CE_BASE + 0x001C)
#define rMAC_CE_LUT_OPERN             (CE_BASE + 0x0020)
#define rMAC_CE_LUT_STATUS            (CE_BASE + 0x0024)
#define rMAC_CE_GTK_PN_MSB            (CE_BASE + 0x0028)
#define rMAC_CE_GTK_PN_LSB            (CE_BASE + 0x002C)
#define rMAC_CE_CONFIG                (CE_BASE + 0x0030)
#define rMAC_CE_RX_GRP_CIPHER_TYPE    (CE_BASE + 0x0034)
#define rMAC_CE_CONTROL               (CE_BASE + 0x0038)
#define rMAC_CE_TKIP_MIC_KEY_Q1       (CE_BASE + 0x003C)
#define rMAC_CE_TKIP_MIC_KEY_Q2       (CE_BASE + 0x0040)
#define rMAC_CE_TKIP_MIC_KEY_Q3       (CE_BASE + 0x0044)
#define rMAC_CE_TKIP_MIC_KEY_Q4       (CE_BASE + 0x0048)
#define rMAC_CE_TKIP_REPLAY_FAIL_CNT  (CE_BASE + 0x004C)
#define rMAC_CE_CCMP_REPLAY_FAIL_CNT  (CE_BASE + 0x0050)
#define rMAC_CE_RX_BC_PN_MSB          (CE_BASE + 0x0054)
#define rMAC_CE_RX_BC_PN_LSB          (CE_BASE + 0x0058)

/*****************************************************************************/
/* Miscellaneous Register Addresses (Arbiter, DMA, Host select etc)          */
/*****************************************************************************/


/*****************************************************************************/
/* Reset Control                                                             */
/*****************************************************************************/

//#define rMACPHYRESCNRTL               (PA_BASE + 0x40004)  //Hugh: FIXME when RTL add PHY reset logic.

#ifndef GENERIC_PLATFORM
/*****************************************************************************/
/* Arbiter Register Addresses                                                */
/*****************************************************************************/

#if 0
//masked by hugh
#define rMAC_PROG_ARBIT_POLICY       (ARB_BASE + 0x0000)
#define rMAC_PROG_WEIGHT_RR          (ARB_BASE + 0x0004)
#define rMAC_PROG_PREEMPT            (ARB_BASE + 0x0008)
#define rMAC_TRIG_ARBITER            (ARB_BASE + 0x000C)
#endif

/*****************************************************************************/
/* Arbiter Register Initialization Values                                    */
/*****************************************************************************/

#define MAC_TRIG_ARBITER_CPU_PROG_INIT_VALUE 0x00000002
#define MAC_PROG_ARBIT_POLICY_INIT_VALUE     0x00000000
#define MAC_PROG_WEIGHT_RR_INIT_VALUE        0x00000000
#define MAC_PROG_PREEMPT_INIT_VALUE          0x00000080
#define MAC_TRIG_ARBITER_INIT_VALUE          0x00000003

#endif /* GENERIC_PLATFORM */

#ifdef MWLAN
/*****************************************************************************/
/* DMA Register Addresses                                                    */
/*****************************************************************************/

//#define rMAC_DMA_BURST_SIZE          (DMA_BASE + 0x0040)
#endif /* MWLAN */

/*****************************************************************************/
/* DMA Register Initialization Values                                        */
/*****************************************************************************/

#define MAC_DMA_BURST_SIZE_INIT_VALUE MAC_PA_DMA_BURST_SIZE_INIT_VALUE

/*****************************************************************************/
/* Host Select Register                                                      */
/*****************************************************************************/

#define rHOST_SEL_REG     (PLD0_ADDR + 0x00040020)


#ifdef MWLAN

#if 0
//Hugh
#define rSW_HANG_DBG_REG     (PLD0_ADDR + 0x00040024)  

#define FC_ASSERT_COUNT_REG_BASE  (PLD0_ADDR + 0x00040040)
#define NUM_FC_ASSERT_COUNT_REG   12

#define rFC_ASSERT1_CNT      (PLD0_ADDR + 0x00040040)
#define rFC_ASSERT2_CNT      (PLD0_ADDR + 0x00040044)
#define rFC_ASSERT3_CNT      (PLD0_ADDR + 0x00040048)
#define rFC_ASSERT4_CNT      (PLD0_ADDR + 0x0004004C)
#define rFC_ASSERT5_CNT      (PLD0_ADDR + 0x00040050)
#define rFC_ASSERT6_CNT      (PLD0_ADDR + 0x00040054)
#define rFC_ASSERT7_CNT      (PLD0_ADDR + 0x00040058)
#define rFC_ASSERT8_CNT      (PLD0_ADDR + 0x0004005C)
#define rFC_ASSERT9_CNT      (PLD0_ADDR + 0x00040060)
#define rFC_ASSERT10_CNT     (PLD0_ADDR + 0x00040064)
#define rFC_ASSERT11_CNT     (PLD0_ADDR + 0x00040068)
#define rFC_ASSERT12_CNT     (PLD0_ADDR + 0x0004006C)
#endif
#endif /* MWLAN */

/*****************************************************************************/
/* Customer Platform Specific Definitions                                    */
/*****************************************************************************/


/*****************************************************************************/
/* P2P Registers                                                             */
/*****************************************************************************/

#ifdef MAC_P2P
#define  r_P2P_CNTRL_REG            (PA_BASE + 0x0800)
#define  r_P2P_NOA_CNT_STATUS_REG   (PA_BASE + 0x0804)
#define  r_P2P_NOA1_DURATION_REG    (PA_BASE + 0x0808)
#define  r_P2P_NOA1_INTERVAL_REG    (PA_BASE + 0x080C)
#define  r_P2P_NOA1_START_TIME_REG  (PA_BASE + 0x0810)
#define  r_P2P_NOA2_DURATION_REG    (PA_BASE + 0x0814)
#define  r_P2P_NOA2_INTERVAL_REG    (PA_BASE + 0x0818)
#define  r_P2P_NOA2_START_TIME_REG  (PA_BASE + 0x081C)
// 20120709 caisf add, merged ittiam mac v1.2 code
#define  r_P2P_EOA_OFFSET           (PA_BASE + 0x0820)
#define  r_P2P_STATUS_REG           (PA_BASE + 0x0824)
#endif /* MAC_P2P */

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

/* Receive Queue Identifier */
typedef enum {NORMAL_PRI_RXQ  = 0,
              HIGH_PRI_RXQ    = 1,
			  INVALID_PRI_RXQ = 2,
} RX_QID_T;

/* Error interrupt codes */
typedef enum {UNEXPECTED_MAC_TX_HANG        = 1,
              UNEXPECTED_TX_Q_EMPTY         = 2,
              UNEXPECTED_TX_STATUS3         = 3,
              BUS_ERROR                     = 4,
              BUS_WRAP_SIG_ERROR            = 5,
              DMA_NO_ERROR                  = 6,
              TX_CE_NOT_READY               = 7,
              RX_CE_NOT_READY               = 8,
              SEQNUM_GEN_ADDR_FAIL          = 9,
              UNEXPECTED_NON_PENDING_MSDU   = 10,
              UNEXPECTED_MSDU_ADDR          = 11,
              UNEXPECTED_RX_Q_EMPTY         = 12,
              TX_INTR_FIFO_OVERRUN          = 13,
              RX_INTR_FIFO_OVERRUN          = 14,
              HIRX_INTR_FIFO_OVERRUN        = 15,
              UNEXPECTED_HIRX_Q_EMPTY       = 16,
              RX_PATH_WDT_TO_FCS_FAILED     = 17,
              RX_PATH_WDT_TO_FCS_PASSED     = 18,
              UNEXPECTED_PHY_TX_HANG        = 19,
              MAX_MAC_HW_ERROR_CODES
} ERROR_INT_CODE_T;

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/
#ifdef IBSS_BSS_STATION_MODE
extern volatile UWORD8 g_cmcc_test_mode;
#endif

extern void   reset_phy_machw(void);
extern UWORD8 get_random_byte(void);
extern UWORD8 get_random_seed(void);
extern void initialize_machw(void);
extern void update_edca_machw(void);
extern void set_machw_tsf_disable(void);
extern void set_machw_tsf_start(void);
extern void set_machw_tsf_join(void);

#ifdef BURST_TX_MODE
extern void update_burst_mode_tx_dscr(UWORD8  *tx_dscr);
#endif /* BURST_TX_MODE */

extern void write_dot11_phy_reg(UWORD8 ra, UWORD32 rd);
extern void read_dot11_phy_reg(UWORD8 ra, UWORD32 *rd);

extern void update_trout_int_mask(UWORD32 mask);
extern UWORD32 get_trout_int_mask(void);
/*zhq add for powersave*/
extern unsigned int root_host_write_trout_reg(unsigned int val, unsigned int reg_addr);
extern unsigned int root_host_read_trout_reg(unsigned int reg_addr);
extern unsigned int root_host_write_trout_ram(void *dst, void *src, unsigned int len);
extern unsigned int root_host_read_trout_ram(void *dst, void *src, unsigned int len);
extern int tx_shareram_slot_busy(UWORD8 slot);
extern void tx_pkt_process_new(UWORD8 slot, int call_flag);
extern UWORD8  tx_pkt_process(UWORD8 slot, int call_flag,UWORD32 **dscr_base);
static inline void force_clear_interrupts(void);
static inline void root_force_clear_interrupts(void);
/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/* Inline functions to get/set general registers in the PA register bank     */

/* The PA version register contains the version of the protocol accelerator  */
/* implementation.                                                           */
INLINE UWORD32 get_machw_pa_ver(void)
{
    //chenq mod
    //return (convert_to_le(rMAC_PA_VER ));
    return (convert_to_le(host_read_trout_reg( (UWORD32)rMAC_PA_VER )));
}

//add by chengwg.
INLINE UWORD32 get_machw_pa_reg_value(void)
{
    return (convert_to_le(host_read_trout_reg((UWORD32)rMAC_PA_CON)));
}

/* The PA control register has various subfields described below.            */
/*                                                                           */
/*          7       6    5       4       3          2       1         0      */
/*       RxFIFCtrl QACK PCFEn HCCAEn  EDCAEnable AP~/STA BSS~/IBSS PA Mode   */
/*                                                                           */
/* 10                          9                      8                      */
/* MSDU Life time check enable RX Link Update Control TX Head Update Control */
/*                                                                           */
/*          14                      13              12             11        */
/*          PHY Register Addr Width Dup Det Disable PA TX Suspend  CE Enable */
/*                                                                           */
/* 20            19       18       17         16             15              */
/* PrChnlCCABPEn DisCTSTx DisAckTx CCATimerEn RxErrorEIFSEn  FCSFailEIFSEn   */
/*                                                                           */
/*      25   24       23               22              21                    */
/*  MACClkEn HTSTAEn  SecChnlCCABPVal  SecChnlCCABPEn  PrChnlCCABPVal        */
/*                                                                           */
/* 31      30           29                 28       27        26             */
/* HwAREn  TxAbortMode  Self-CTSonTxAbort  TxAbort  De-aggrEn DefragEn       */
/*                                                                           */
/* PA Mode     0 - Power down                                                */
/*             1 - Power up                                                  */
/* BSS~/IBSS   1 - This station is part of Independent BSS.                  */
/*             0 - This station is part of Infrastructure BSS.               */
/* AP~/STA     0 - In Infrastructure BSS this is an access point.            */
/*             1 - In Infrastructure BSS this is a station                   */
/* EDCA Enable 0 - EDCA feature disabled. DCF procedure is followed.         */
/*             1 - EDCA feature is enabled.                                  */
/* HCCA Enable 0 - HCCA feature disabled.                                    */
/*             1 - HCCA feature enabled.                                     */
/* PCF         0 - PCF feature is disabled.                                  */
/*             1 - PCF feature is enabled.                                   */
/* QACK        0 - CF-ACK bit in non-directed frame will not be interpreted. */
/*             1 - CF-ACK bit in non-directed frame will be interpreted.     */
/*             CF-Ack bit set in received directed frame will be interpreted */
/*             regardless of this setting.                                   */
/* RX FIFO     0 - RX buffer is always needed by H/w otherwise it will hang  */
/* control     1 - If RX buffer is not available, H/w will not hang          */
/* TX Head     0 - TX head pointer register (points to first frame in queue) */
/* Update Con      with the next pointer read from current TX descriptor at  */
/*                 completion of transmission.                               */
/*             1 - TX head pointer register is always updated with NULL at   */
/*                 completion of transmission.                               */
/* RX Link     0 - RX link pointer register (pointer to next RX link element */
/* Update Con      in queue) with the next pointer read from current RX link */
/*                 element at completion of reception.                       */
/*             1 - RX link pointer register is always updated with NULL at   */
/*                 completion of reception.                                  */
/* MSDU Life   1 - Lifetime of the MSDU is validated                         */
/* time check  0 - Lifetime of the MSDU is not validated                     */
/* CE Enable   1 - Cipher engine is enabled for encryption/decryption        */
/*             0 - Cipher engine is disabled.                                */
/* TX Suspend  1 - Suspend request. This value triggers the suspension of    */
/*                 Protocol accelerator transmission after the current frame */
/*                 exchange. SIFS response and Beacon transmission however   */
/*                 are not stopped.                                          */
/*             0 - Protocol accelerator is not suspended.                    */
/* Disable Dup 1 - Disables duplicate detection in hardware.                 */
/* Detection   0 - Default.  Duplicate detection in hardware is enabled.     */
/* PHY Reg     0 - Lower 7 bits of PHY Register Address only are valid.      */
/* Addr Width  1 - All 8 bits of PHY Register Address are valid and are      */
/*                 shifted out of SPI.                                       */
/* HT STA En   0 - Non HT STA                                                */
/*             1 - HT STA                                                    */
/* MACClkEn    1 - Clock is enabled for MAC H/W blocks                       */
/*             0 - Clock is disabled (gated) for MAC H/W blocks              */
/* DefragEn    1 - Defragmentation will be done by H/W                       */
/*             0 - Defragmentation will not be done in H/W                   */
/* De-aggrEn   1 - De-aggregation will be done by H/W                        */
/*             0 - De-aggregation will not be done by H/W                    */
/* TxAbort     1 - Transmission will be aborted                              */
/*             0 - PA will work normally based on channel access function    */
/* Self-CTS    1 - PA will transmit self-CTS with in PIFS after a TX abort   */
/* onTxAbort   0 - PA will not transmit self-CTS after a TX abort            */
/* TxAbortMode 1 - TX abort can be initiated only by H/W                     */
/*             0 - TX abort can be initiated only by S/W                     */
/* HwAREn      0 - Auto rate for retransmissions will not be done by H/w     */
/*             1 - Auto rate for retransmissions will be done by H/w         */

INLINE void enable_8bit_phy_addr(void)
{
    //chenq mod
    //rMAC_PA_CON |= REGBIT14;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PA_CON ) | REGBIT14,
        (UWORD32)rMAC_PA_CON );
}

INLINE void bypass_cca(void)
{
    //chenq mod
    //rMAC_PA_CON |= (REGBIT20 + REGBIT22);
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PA_CON ) | (REGBIT20 + REGBIT22),
        (UWORD32)rMAC_PA_CON );
    

#ifdef DEBUG_MODE
    PRINTD2("PHY CCA is bypassed!!\n\r");
#endif /* DEBUG_MODE */
}

INLINE void cca_based_prot(UWORD8 enable)
{
    //chenq  mod
    if(enable == BTRUE)
        //rMAC_PA_CON |= REGBIT17;
        host_write_trout_reg(
            host_read_trout_reg( (UWORD32)rMAC_PA_CON ) | REGBIT17,
            (UWORD32)rMAC_PA_CON );
    else
        //rMAC_PA_CON |= (REGBIT15 + REGBIT16);
        host_write_trout_reg(
            host_read_trout_reg( (UWORD32)rMAC_PA_CON ) | (REGBIT15 + REGBIT16),
            (UWORD32)rMAC_PA_CON );

}

INLINE void disable_machw_phy_and_pa(void)
{
    //chenq mod
    //rMAC_PA_CON &= ~REGBIT0;
    host_write_trout_reg(
            host_read_trout_reg( (UWORD32)rMAC_PA_CON ) & (~REGBIT0),
            (UWORD32)rMAC_PA_CON );
}
/*zhq add for powersave*/
INLINE void root_disable_machw_phy_and_pa(void)
{
    //chenq mod
    //rMAC_PA_CON &= ~REGBIT0;
    root_host_write_trout_reg(
            root_host_read_trout_reg( (UWORD32)rMAC_PA_CON ) & (~REGBIT0),
            (UWORD32)rMAC_PA_CON );
}

INLINE void enable_machw_phy_and_pa(void)
{
	unsigned int v;
    /* Enable PHY Synchronization */
    wait_for_sync();

	v = convert_to_le(host_read_trout_reg( (UWORD32)rMAC_PA_CON));
	if(v & BIT0)
		return;
	/* handle unexcepted interrupts lost */
	//force_clear_interrupts();
    //chenq mod
    //rMAC_PA_CON |= REGBIT0;
    host_write_trout_reg(
            host_read_trout_reg( (UWORD32)rMAC_PA_CON ) | REGBIT0,
            (UWORD32)rMAC_PA_CON ); 
}

INLINE void root_enable_machw_phy_and_pa(void)
{
	unsigned long v;
    /* Enable PHY Synchronization */
    wait_for_sync();
	/*jiangtao.yi changed host_read_trout_reg to root_host_read_trout_reg for bug244373*/
	v = convert_to_le(root_host_read_trout_reg( (UWORD32)rMAC_PA_CON));
	if(v & BIT0)
		return;
	/* handle unexcepted interrupts lost by zhao 07-05 2013 */
	//root_force_clear_interrupts();

    //chenq mod
    //rMAC_PA_CON |= REGBIT0;
    root_host_write_trout_reg(
            root_host_read_trout_reg( (UWORD32)rMAC_PA_CON ) | REGBIT0,
            (UWORD32)rMAC_PA_CON );
}

INLINE BOOL_T is_machw_enabled(void)
{
    //chenq mod
    //if( rMAC_PA_CON  &  REGBIT0 )
    if(host_read_trout_reg( (UWORD32)rMAC_PA_CON ) & REGBIT0 )
    {
        return BTRUE;
    }

    return BFALSE;
}
/*zhq add for powersave*/
INLINE BOOL_T root_is_machw_enabled(void)
{
    //chenq mod
    //if( rMAC_PA_CON  &  REGBIT0 )
    if(root_host_read_trout_reg( (UWORD32)rMAC_PA_CON ) & REGBIT0 )
    {
        return BTRUE;
    }

    return BFALSE;
}

INLINE void set_machw_op_mode(UWORD8 mode)
{
    //chenq mod
    //rMAC_PA_CON &= ~REGBIT1;
    //rMAC_PA_CON &= ~REGBIT2;
    host_write_trout_reg(
            host_read_trout_reg( (UWORD32)rMAC_PA_CON ) & (~REGBIT1),
            (UWORD32)rMAC_PA_CON );
    host_write_trout_reg(
            host_read_trout_reg( (UWORD32)rMAC_PA_CON ) & (~REGBIT2),
            (UWORD32)rMAC_PA_CON );
    

    if(mode == 2)
    {
        //chenq mod
        //rMAC_PA_CON |= REGBIT1;
        //rMAC_PA_CON |= REGBIT2;
        host_write_trout_reg(
            host_read_trout_reg( (UWORD32)rMAC_PA_CON ) | (REGBIT1) ,
            (UWORD32)rMAC_PA_CON );
        host_write_trout_reg(
            host_read_trout_reg( (UWORD32)rMAC_PA_CON ) | (REGBIT2) ,
            (UWORD32)rMAC_PA_CON );
    }
    else if(mode == 1)
    {
#ifndef BSS_ACCESS_POINT_MODE
        //chenq mod
        //rMAC_PA_CON |= REGBIT2;
        host_write_trout_reg(
            host_read_trout_reg( (UWORD32)rMAC_PA_CON ) | (REGBIT2) ,
            (UWORD32)rMAC_PA_CON );
#endif /* BSS_ACCESS_POINT_MODE */
    }
}

INLINE void enable_machw_edca(void)
{
    //chenq mod
    //rMAC_PA_CON |= REGBIT3;
    host_write_trout_reg(
            host_read_trout_reg( (UWORD32)rMAC_PA_CON ) | (REGBIT3) ,
            (UWORD32)rMAC_PA_CON );
}

INLINE void disable_machw_edca(void)
{
    //chenq mod
    //rMAC_PA_CON &= ~REGBIT3;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PA_CON ) & (~REGBIT3) ,
        (UWORD32)rMAC_PA_CON );
}

INLINE void enable_machw_hcca(void)
{
    //chenq mod
    //rMAC_PA_CON |= REGBIT4;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PA_CON ) | (REGBIT4) ,
        (UWORD32)rMAC_PA_CON );
}

INLINE void enable_machw_pcf(void)
{
    //chenq mod
    //rMAC_PA_CON |= REGBIT5;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PA_CON ) | (REGBIT5) ,
        (UWORD32)rMAC_PA_CON );
}

INLINE void enable_machw_process_cfack(void)
{
    //chenq mod
    //rMAC_PA_CON |= REGBIT6;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PA_CON ) | (REGBIT6) ,
        (UWORD32)rMAC_PA_CON );
}

INLINE void disable_rxbuf_dependency(void)
{
    //chenq mod
    //rMAC_PA_CON |= REGBIT7;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PA_CON ) | (REGBIT7) ,
        (UWORD32)rMAC_PA_CON );
}

INLINE void disable_txptr_update(void)
{
    //chenq mod
    //rMAC_PA_CON |= REGBIT8;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PA_CON ) | (REGBIT8) ,
        (UWORD32)rMAC_PA_CON );
}

INLINE void enable_txptr_update(void)	//add by chengwg.
{
    host_write_trout_reg(
        host_read_trout_reg((UWORD32)rMAC_PA_CON) & (~REGBIT8),
        (UWORD32)rMAC_PA_CON);
}


INLINE void disable_rxptr_update(void)
{
    //chenq mod
    //rMAC_PA_CON |= REGBIT9;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PA_CON ) | (REGBIT9) ,
        (UWORD32)rMAC_PA_CON );
}

INLINE void enable_msdu_lifetime_check(void)
{
    //chenq mod
    //rMAC_PA_CON |= REGBIT10;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PA_CON ) | (REGBIT10) ,
        (UWORD32)rMAC_PA_CON );
}

INLINE void disable_msdu_lifetime_check(void)
{
    //chenq mod
    //rMAC_PA_CON &= ~REGBIT10;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PA_CON ) & (~REGBIT10) ,
        (UWORD32)rMAC_PA_CON );
}

extern UWORD32 hw_txq_busy;
INLINE void set_machw_tx_suspend(void)
{
	UWORD32 cnt = 0, f, s = 0;
	struct trout_private *tp;
	int reset_lock_flag=0;

	/* tx suspend & resume need a mutex by zhao */
	tp = netdev_priv(g_mac_dev);
	mutex_lock(&tp->txsr_mutex);

	if(is_machw_enabled() == BFALSE)
		goto out;
	reset_lock_flag = reset_mac_trylock();
	/*leon liu removed processing in reset_mac*/
#if 0
	if(unlikely(reset_lock_flag == 0)){
		while(hw_txq_busy){
			f = convert_to_le(host_read_trout_reg( (UWORD32)rMAC_PA_CON));
			if((f & 0x1) == 0){
				printk("@@@:PA disabled\n");
				goto out;
			}
			s++;
			if(s >= 1000){
				printk("@@@:WTTX-TIMEOUT\n");
				break;
			}
		}
	}
#endif
	/* if already in suspend state, do nothing by zhao */
	f = convert_to_le(host_read_trout_reg( (UWORD32)rMAC_PA_CON));
	//s = convert_to_le(host_read_trout_reg( (UWORD32)rMAC_PA_STAT));
	if((f & BIT12)){
		/* just clear the TX suspend INT, just do it BY zhao */
		host_write_trout_reg( REGBIT14, (UWORD32)rMAC_INT_STAT);
		host_write_trout_reg( REGBIT14 << 2, (UWORD32)rCOMM_INT_CLEAR);
		goto out;
	}
	/* arrive here, means TX HW must in active mode */
    //chenq mod
    //rMAC_PA_CON |= REGBIT12;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PA_CON ) | (REGBIT12) ,
        (UWORD32)rMAC_PA_CON );

    /* Wait till MAC H/w enters in suspend state */
    //chenq mod
    //while( (rMAC_PA_STAT & REGBIT6) == 0)
    while((host_read_trout_reg( (UWORD32)rMAC_PA_STAT) & REGBIT6) == 0)
    {
        if(++cnt > 1000)
            break;
    }
	if(cnt >= 1000)
		printk("@@@@@@@@ %s NOT SUSPEND, TIMEOUT\n", __func__);	
	/* although later the int will be ASSERT, but just do it for safty by zhao */
	host_write_trout_reg( REGBIT14, (UWORD32)rMAC_INT_STAT);
	host_write_trout_reg( REGBIT14 << 2, (UWORD32)rCOMM_INT_CLEAR);
out:
	if(reset_lock_flag){
		reset_mac_unlock();
	}
	mutex_unlock(&tp->txsr_mutex);
	return;
}
/*zhq add for powersave*/
INLINE void  root_set_machw_tx_suspend(void)
{
	UWORD32 cnt = 0, f;

    if(root_is_machw_enabled() == BFALSE)
        return;
	f = convert_to_le(root_host_read_trout_reg( (UWORD32)rMAC_PA_CON));
	//s = convert_to_le(root_host_read_trout_reg( (UWORD32)rMAC_PA_STAT));
	if((f & BIT12)){
		/* just clear the TX suspend INT */
		root_host_write_trout_reg( REGBIT14, (UWORD32)rMAC_INT_STAT);
		root_host_write_trout_reg( REGBIT14 << 2, (UWORD32)rCOMM_INT_CLEAR);
		return;
	}

    //chenq mod
    //rMAC_PA_CON |= REGBIT12;
    root_host_write_trout_reg(
        root_host_read_trout_reg( (UWORD32)rMAC_PA_CON ) | (REGBIT12) ,
        (UWORD32)rMAC_PA_CON );

    /* Wait till MAC H/w enters in suspend state */
    //chenq mod
    //while( (rMAC_PA_STAT & REGBIT6) == 0)
    while((root_host_read_trout_reg( (UWORD32)rMAC_PA_STAT) & REGBIT6) == 0)
    {
        if(++cnt > 1000)
            break;
    }
	if(cnt >= 1000)
		printk("@@@@@@@@ %s NOT SUSPEND, TIMEOUT\n", __func__);	
	/* although later the int will be ASSERT, but just do it for safty by zhao */
	root_host_write_trout_reg( REGBIT14, (UWORD32)rMAC_INT_STAT);
	root_host_write_trout_reg( REGBIT14 << 2, (UWORD32)rCOMM_INT_CLEAR);
}

INLINE void set_machw_tx_resume(void)
{
	unsigned long v, cnt;
	struct trout_private *tp;

	/* tx suspend & resume need a mutex by zhao */
	tp = netdev_priv(g_mac_dev);
	mutex_lock(&tp->txsr_mutex);	

	v = convert_to_le(host_read_trout_reg( (UWORD32)rMAC_PA_CON));
	v = v & (~BIT12);
	host_write_trout_reg(convert_to_le(v), (UWORD32)rMAC_PA_CON);

	while((host_read_trout_reg( (UWORD32)rMAC_PA_STAT) & REGBIT6) != 0){
		if(++cnt > 1000)
			break;
	}
	if(cnt > 1000)
		printk("%s ..wait PA status BIT6 to 0 timeout!\n", __func__);

	mutex_unlock(&tp->txsr_mutex);
	/* just clear the TX suspend INT */
	host_write_trout_reg( REGBIT14, (UWORD32)rMAC_INT_STAT);
	host_write_trout_reg( REGBIT14 << 2, (UWORD32)rCOMM_INT_CLEAR);
/*
    //chenq mod
    //rMAC_PA_CON &= ~REGBIT12;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PA_CON ) & (~REGBIT12) ,
        (UWORD32)rMAC_PA_CON );
*/
}
/*zhq add for powersave*/
INLINE void root_set_machw_tx_resume(void)
{
	unsigned long v, cnt;

	v = convert_to_le(root_host_read_trout_reg( (UWORD32)rMAC_PA_CON));
	v = v & (~BIT12);
	root_host_write_trout_reg(convert_to_le(v), (UWORD32)rMAC_PA_CON);

	while((root_host_read_trout_reg( (UWORD32)rMAC_PA_STAT) & REGBIT6) != 0){
		if(++cnt > 1000)
			break;
	}
	if(cnt > 1000)
		printk("%s ..wait PA status BIT6 to 0 timeout!\n", __func__);

	/* just clear the TX suspend INT */
	root_host_write_trout_reg( REGBIT14, (UWORD32)rMAC_INT_STAT);
	root_host_write_trout_reg( REGBIT14 << 2, (UWORD32)rCOMM_INT_CLEAR);
/*
    //chenq mod
    //rMAC_PA_CON &= ~REGBIT12;
    root_host_write_trout_reg(
        root_host_read_trout_reg( (UWORD32)rMAC_PA_CON ) & (~REGBIT12) ,
        (UWORD32)rMAC_PA_CON );
*/
}

INLINE BOOL_T is_machw_tx_suspended(void)
{
    //chenq mod
    //if(rMAC_PA_CON & REGBIT12)
    if( host_read_trout_reg( (UWORD32)rMAC_PA_CON ) & REGBIT12)
    {
        return BTRUE;
    }

    return BFALSE;
}

INLINE void disable_machw_duplicate_detection(void) //dumy add 0814
{
    //chenq mod
    //rMAC_PA_CON |= REGBIT13;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PA_CON ) | (REGBIT13) ,
        (UWORD32)rMAC_PA_CON );
}

INLINE void disable_machw_ack_trans(void)
{
    //chenq mod
    //rMAC_PA_CON |= REGBIT18;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PA_CON ) | (REGBIT18) ,
        (UWORD32)rMAC_PA_CON );
}

INLINE void enable_machw_ack_trans(void)
{
    //chenq mod
    //rMAC_PA_CON &= ~REGBIT18;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PA_CON ) & (~REGBIT18) ,
        (UWORD32)rMAC_PA_CON );
}

INLINE void disable_machw_cts_trans(void)
{
    //chenq mod
    //rMAC_PA_CON |=  REGBIT19;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PA_CON ) | (REGBIT19) ,
        (UWORD32)rMAC_PA_CON );
}

INLINE void enable_machw_cts_trans(void)
{
    //chenq mod
    //rMAC_PA_CON &=  ~REGBIT19;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PA_CON ) & (~REGBIT19) ,
        (UWORD32)rMAC_PA_CON );
}

INLINE void enable_machw_ht(void)
{
    //chenq mod
    //rMAC_PA_CON |= REGBIT24;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PA_CON ) | (REGBIT24) ,
        (UWORD32)rMAC_PA_CON );
}

INLINE void disable_machw_ht(void)
{
    //chenq mod
    //rMAC_PA_CON &= ~REGBIT24;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PA_CON ) & (~REGBIT24) ,
        (UWORD32)rMAC_PA_CON );
}

#ifndef TROUT_WIFI_POWER_SLEEP_ENABLE
INLINE void enable_machw_clock(void)
{
	static int enable_flag = 0 ;

	if(enable_flag)
		return;

    //chenq mod
    //rMAC_PA_CON |= REGBIT25;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PA_CON ) | (REGBIT25) ,
        (UWORD32)rMAC_PA_CON );

	enable_flag = 1;
}

INLINE void disable_machw_clock(void)
{
    //chenq mod
    //rMAC_PA_CON &= ~REGBIT25;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PA_CON ) & (~REGBIT25) ,
        (UWORD32)rMAC_PA_CON );
}

#else /* TROUT_WIFI_POWER_SLEEP_ENABLE */
extern void enable_machw_clock(void);
extern void disable_machw_clock(void);

#endif /* TROUT_WIFI_POWER_SLEEP_ENABLE */

INLINE void enable_machw_defrag(void)
{
    //chenq mod
    //rMAC_PA_CON |= REGBIT26;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PA_CON ) | (REGBIT26) ,
        (UWORD32)rMAC_PA_CON );
}

INLINE void disable_machw_defrag(void)
{
    //chenq mod
    //rMAC_PA_CON &= ~REGBIT26;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PA_CON ) & (~REGBIT26) ,
        (UWORD32)rMAC_PA_CON );
}

INLINE void enable_machw_deaggr(void)
{
    //chenq mod
    //rMAC_PA_CON |= REGBIT27;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PA_CON ) | (REGBIT27) ,
        (UWORD32)rMAC_PA_CON );
}

INLINE void disable_machw_deaggr(void)
{
    //chenq mod
    //rMAC_PA_CON &= ~REGBIT27;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PA_CON ) & (~REGBIT27) ,
        (UWORD32)rMAC_PA_CON );
}

INLINE void enable_machw_tx_abort(void)
{
    //chenq mod
    //rMAC_PA_CON |= REGBIT28;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PA_CON ) | (REGBIT28) ,
        (UWORD32)rMAC_PA_CON );
}

INLINE void disable_machw_tx_abort(void)
{
    //chenq mod
    //rMAC_PA_CON &= ~REGBIT28;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PA_CON ) & (~REGBIT28) ,
        (UWORD32)rMAC_PA_CON );
}
#ifdef TX_ABORT_FEATURE
INLINE BOOL_T is_machw_tx_aborted(void)
{
    if(g_machw_tx_aborted == BTRUE)
    {
        return BTRUE;
    }

    //chenq mod
    //if((rMAC_PA_CON & REGBIT28) == REGBIT28)
    if(( host_read_trout_reg( (UWORD32)rMAC_PA_CON) & REGBIT28) == REGBIT28)
        return BTRUE;

    return BFALSE;
}
#endif /* TX_ABORT_FEATURE */
INLINE void enable_machw_selfcts_on_txab(void)
{
    //chenq mod
    //rMAC_PA_CON |= REGBIT29;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PA_CON ) | (REGBIT29) ,
        (UWORD32)rMAC_PA_CON );
}

INLINE void disable_machw_selfcts_on_txab(void)
{
    //chenq mod
    //rMAC_PA_CON &= ~REGBIT29;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PA_CON ) & (~REGBIT29) ,
        (UWORD32)rMAC_PA_CON );
}

INLINE void enable_machw_txabort_hw_trig(void)
{
    //chenq mod
    //rMAC_PA_CON |= REGBIT30;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PA_CON ) | (REGBIT30) ,
        (UWORD32)rMAC_PA_CON );
}

INLINE void enable_machw_txabort_sw_trig(void)
{
    //chenq mod
    //rMAC_PA_CON &= ~REGBIT30;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PA_CON ) & (~REGBIT30) ,
        (UWORD32)rMAC_PA_CON );
}

INLINE void enable_machw_autorate(void)
{
    //chenq mod
    //rMAC_PA_CON |= REGBIT31;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PA_CON ) | (REGBIT31) ,
        (UWORD32)rMAC_PA_CON );
}

INLINE void disable_machw_autorate(void)
{
    //chenq mod
    //rMAC_PA_CON &= ~REGBIT31;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PA_CON ) & (~REGBIT31) ,
        (UWORD32)rMAC_PA_CON );
}

/* This function stops all transmissions from MAC HW */
INLINE void disable_machw_tx(void)
{
    set_machw_tx_suspend();
    disable_machw_ack_trans();
    disable_machw_cts_trans();
}

/* This function resumes transmissions from MAC HW */
INLINE void enable_machw_tx(void)
{
    set_machw_tx_resume();
    enable_machw_ack_trans();
    enable_machw_cts_trans();
}

/* The PA status register has various subfields described below.             */
/*                                                                           */
/* 5         4                   3                   2           1     0     */
/* Slot Used Beacon Transmitted  Duplicate Detected  PHY Rx Mode CCA   FCS   */
/*                                                                           */
/*  20     19 - 12           11 - 8            7         6                   */
/* HCCA-AP HCCA Q 0 - 7 Lock EDCA Q 0 - 3 Lock HP Q Lock PA in suspend mode  */
/*                                                                           */
/*    24                         23              22                21        */
/*    Self-CTS done on TxAbort   PA in Tx Abort  Prev Beacon Tx    PCF Reg   */
/*                                                                           */
/* FCS                 0 - FCS check failed.                                 */
/*                     1 - FCS check passed.                                 */
/* CCA                 0 - Idle.                                             */
/*                     1 - Busy.                                             */
/* PHY RX Mode         0 - Received frame is of 802.11b type.                */
/*                     1 - Received frame is of OFDM type.                   */
/* Duplicate Detected  0 - Received frame was not a duplicate.               */
/*                     1 - Received frame was detected as duplicate.         */
/* Beacon Transmitted  0 - Beacon was not transmitted this beacon interval.  */
/*                     1 - Beacon was transmitted in this beacon interval.   */
/* Slot in use         0 - Short slot is being used                          */
/*                     1 - Long slot is being used                           */
/* PA in suspend mode  0 - Protocol accelerator is active and can initiate   */
/*                         frame exchanges.                                  */
/*                     1 - Protocol accelerator is in suspension mode. It    */
/*                         will not initiate any frame exchange. However     */
/*                         SIFS response, Beacon Tx are not stopped.         */
/* xxx Q access lock   0 - The registers storing the pointer to the first    */
/*                         frame in xxx Q can be accessed.                   */
/*                     1 - The registers storing the pointer to the first    */
/*                         frame in xxx Q should not be accessed             */

INLINE BOOL_T get_machw_fcs_stat(void)
{
    //chenq mod
    //if((rMAC_PA_STAT & REGBIT0) == 0)
    if( (host_read_trout_reg( (UWORD32)rMAC_PA_STAT  ) & REGBIT0) == 0 )
        return BFALSE;

    return BTRUE;
}

INLINE BOOL_T get_machw_pri_cca_busy_stat(void)
{
    //chenq mod
    //if((rMAC_PA_STAT & REGBIT1) == 0)
    if( (host_read_trout_reg( (UWORD32)rMAC_PA_STAT  ) & REGBIT1) == 0 )
        return BFALSE;

    return BTRUE;
}

INLINE UWORD8 get_machw_sec_cca_busy_stat(void)
{
    //chenq mod
    //if((rMAC_PA_STAT & REGBIT2) == 0)
    if( (host_read_trout_reg( (UWORD32)rMAC_PA_STAT  ) & REGBIT2) == 0 )
        return BFALSE;

    return BTRUE;
}

INLINE BOOL_T get_machw_dup_det_stat(void)
{
    //chenq mod
    //if((rMAC_PA_STAT & REGBIT3) == 0)
    if( (host_read_trout_reg( (UWORD32)rMAC_PA_STAT  ) & REGBIT3) == 0 )
        return BFALSE;

    return BTRUE;
}

INLINE BOOL_T get_machw_bcn_tx_stat(void)
{
    //chenq mod
    //if((rMAC_PA_STAT & REGBIT4) == 0)
    if( (host_read_trout_reg( (UWORD32)rMAC_PA_STAT  ) & REGBIT4) == 0 )
        return BFALSE;

    return BTRUE;
}


INLINE BOOL_T get_machw_spnd_tx_stat(void)
{
    //chenq mod
    //if((rMAC_PA_STAT & REGBIT6) == 0)
    if( (host_read_trout_reg( (UWORD32)rMAC_PA_STAT  ) & REGBIT6) == 0 )
        return BFALSE;

    return BTRUE;
}

INLINE BOOL_T get_machw_prev_bcn_tx_stat(void)
{
    //chenq mod
    //if((rMAC_PA_STAT & REGBIT22) == 0)
    if( (host_read_trout_reg( (UWORD32)rMAC_PA_STAT  ) & REGBIT22) == 0 )
        return BFALSE;

    return BTRUE;
}

INLINE BOOL_T get_machw_selfcts_tx_stat(void)
{
    //chenq mod
    //if((rMAC_PA_STAT & REGBIT24) == 0)
    if( (host_read_trout_reg( (UWORD32)rMAC_PA_STAT  ) & REGBIT24) == 0 )
        return BFALSE;

    return BTRUE;
}

INLINE void wait_for_machw_tx_abort_start(void)
{
    //chenq mod
    //while((rMAC_PA_STAT & REGBIT23) == 0);
    while((host_read_trout_reg( (UWORD32)rMAC_PA_STAT  ) & REGBIT23) == 0);
}

INLINE void wait_for_machw_tx_abort_end(void)
{
    //chenq mod
    //while((rMAC_PA_STAT & REGBIT23) != 0);
    while((host_read_trout_reg( (UWORD32)rMAC_PA_STAT  ) & REGBIT23) != 0);
}

/* The MAC address is set in the 2 registers with the lower 32 bits set in   */
/* rMAC_ADDR_LO and higher 16 bits in rMAC_ADDR_HI                           */
INLINE void set_machw_macaddr(UWORD8* mac_addr)
{
    UWORD32 mac_h = 0;
    UWORD32 mac_l = 0;

    mac_h = mac_addr[0];
    mac_h = (mac_h << 8) | mac_addr[1];

    mac_l = mac_addr[2];
    mac_l = (mac_l << 8) | mac_addr[3];
    mac_l = (mac_l << 8) | mac_addr[4];
    mac_l = (mac_l << 8) | mac_addr[5];

    //chenq mod
    //rMAC_ADDR_HI = convert_to_le(mac_h);
    host_write_trout_reg( convert_to_le(mac_h), (UWORD32)rMAC_ADDR_HI );
    //rMAC_ADDR_LO = convert_to_le(mac_l);
    host_write_trout_reg( convert_to_le(mac_l), (UWORD32)rMAC_ADDR_LO );
}

/* The BSSID is set in 2 registers with lower 32 bits set in rMAC_BSSID_LO   */
/* and higher 16 bits in rMAC_BSSID_HI.                                      */
INLINE void set_machw_bssid(UWORD8* bssid)
{
    UWORD32 bssid_h = 0;
    UWORD32 bssid_l = 0;

    bssid_h = bssid[0];
    bssid_h = (bssid_h << 8) | bssid[1];

    bssid_l = bssid[2];
    bssid_l = (bssid_l << 8) | bssid[3];
    bssid_l = (bssid_l << 8) | bssid[4];
    bssid_l = (bssid_l << 8) | bssid[5];

    //chenq mod
    //rMAC_BSSID_HI = convert_to_le(bssid_h);
    //rMAC_BSSID_LO = convert_to_le(bssid_l);
    host_write_trout_reg( convert_to_le(bssid_h), (UWORD32)rMAC_BSSID_HI );
    host_write_trout_reg( convert_to_le(bssid_l), (UWORD32)rMAC_BSSID_LO );
}


/* The seed value for the PRBS generator in the hardware is set to the given */
/* input seed in the register rMAC_PRBS_SEED_VAL.                            */
INLINE void set_machw_prng_seed_val(UWORD16 seed_val)
{
    //chenq mod
    //rMAC_PRBS_SEED_VAL = convert_to_le((UWORD32)seed_val);
    host_write_trout_reg( convert_to_le((UWORD32)seed_val), 
                            (UWORD32)rMAC_PRBS_SEED_VAL );
}

INLINE void set_machw_prng_seed_val_all_ac(void)
{
    UWORD32 temp = 0;

    temp = get_random_seed();
    temp = (temp << 8) | get_random_seed();
    temp = (temp << 8) | get_random_seed();
    temp = (temp << 8) | get_random_seed();

    //chenq mod
    //rMAC_PRBS_SEED_VAL = convert_to_le(temp);
    host_write_trout_reg( convert_to_le(temp), 
                            (UWORD32)rMAC_PRBS_SEED_VAL );
}

/* The PRBS read control register has various subfields described below.     */
/*                                                                           */
/* 31 - 10                    9 - 1                    0                     */
/* Reserved                   Value                    Control               */
/*                                                                           */
/* The control bit should be set when S/w needs to read a random number. On  */
/* loading a random number to the value subfield the bit will be reset.      */

INLINE UWORD8 get_random_number(void)
{
// caisf temp mod for the reg rMAC_PRBS_READ_CTRL error!
//chenq add a macro 2012-12-20
#ifdef TROUT_WIFI_EVB
    return get_random_seed();
#endif
    //chenq mod
    //rMAC_PRBS_READ_CTRL |= REGBIT0;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PRBS_READ_CTRL ) | (REGBIT0) ,
        (UWORD32)rMAC_PRBS_READ_CTRL);

    //while( (rMAC_PRBS_READ_CTRL & REGBIT0) != 0)
    while((host_read_trout_reg( (UWORD32)rMAC_PRBS_READ_CTRL )
            & REGBIT0) != 0)
    {
        /* Wait till the control bit is reset so that the value is loaded */
    }

	//return (UWORD8)(convert_to_le(rMAC_PRBS_READ_CTRL) >> 1);
    return (UWORD8)(convert_to_le(
        host_read_trout_reg( (UWORD32)rMAC_PRBS_READ_CTRL )) >> 1);
}

#ifdef NON_FC_MACHW_SUPPORT
/* Inline functions to get/set reception registers in the PA register bank   */
/* 7          6            5          4        3       2       1       0     */
/* NonDir     QCF_POLL     CF_END     ATIM     BCN     RTS     CTS     ACK   */
/* 31-14     13          12              11      10        9       8         */
/* Reserved  ExpectedBA  UnexpectedBA    Deauth  FCSFail   Dup     OtherBSS  */
#else /* NON_FC_MACHW_SUPPORT */
/* The Received Frame Filter register has various subfields described below. */
/*         8            7         6        5       4      3     2    1    0  */
/*   BcstMgmtOBSS  NonDirMgmt  QCF_Poll  CF-End  ATIM  Beacon  RTS  CTS  ACK */
/*                                                                           */
/*            15         14         13        12        11      10      9    */
/*        BcstDataOBSS  SecChan  SIFS-BA  NonSIFS-BA  Deauth  FCSFail  Dup   */
/*                                                                           */
/*                        31-19         18              17          16       */
/*                      Reserved  DiscardedIBSSBcn  NonDirCtrl  NonDirData   */
#endif /* NON_FC_MACHW_SUPPORT */

INLINE void set_machw_rx_frame_filter(UWORD32 frame_filter)
{
    //chenq mod
    //rMAC_RX_FRAME_FILTER = convert_to_le(frame_filter);
    host_write_trout_reg( convert_to_le(frame_filter), 
                          (UWORD32)rMAC_RX_FRAME_FILTER );
}

/* This function checks if Beacon filter is enabled */
INLINE BOOL_T is_bcn_filter_on(void)
{
    /* Check for BIT 3 set or not */

    //chenq mod
    //if((rMAC_RX_FRAME_FILTER & REGBIT3) == REGBIT3)
    if(( host_read_trout_reg( (UWORD32)rMAC_RX_FRAME_FILTER) 
            & REGBIT3) == REGBIT3)
        return BTRUE;

    return BFALSE;
}

/* This function enables Beacon filtering in hardware */
INLINE void enable_machw_beacon_filter(void)
{
    //chenq mod
    //rMAC_RX_FRAME_FILTER |= REGBIT3;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_RX_FRAME_FILTER ) | (REGBIT3) ,
        (UWORD32)rMAC_RX_FRAME_FILTER );
}

/* This function disables beacon filtering in hardware */
INLINE void disable_machw_beacon_filter(void)
{
    //chenq mod
    //rMAC_RX_FRAME_FILTER &= ~REGBIT3;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_RX_FRAME_FILTER ) & (~REGBIT3) ,
        (UWORD32)rMAC_RX_FRAME_FILTER );
}
/* This function enables filtering of non directed Mgmt frames in hardware */
INLINE void enable_machw_non_directed_mgmt_filter(void)
{
    //chenq mod
    //rMAC_RX_FRAME_FILTER |= REGBIT7;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_RX_FRAME_FILTER ) | (REGBIT7) ,
        (UWORD32)rMAC_RX_FRAME_FILTER );
}

/* This function disables filtering of non directed Mgmt frames in hardware */
INLINE void disable_machw_non_directed_mgmt_filter(void)
{
    //chenq mod
    //rMAC_RX_FRAME_FILTER &= ~REGBIT7;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_RX_FRAME_FILTER ) & (~REGBIT7) ,
        (UWORD32)rMAC_RX_FRAME_FILTER );
}

/* This function enables filtering of non directed Data frames in hardware */
INLINE void enable_machw_non_directed_data_filter(void)
{
#ifndef NON_FC_MACHW_SUPPORT
    //chenq mod
    //rMAC_RX_FRAME_FILTER |= REGBIT16;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_RX_FRAME_FILTER ) | (REGBIT16) ,
        (UWORD32)rMAC_RX_FRAME_FILTER );
#endif /* NON_FC_MACHW_SUPPORT */
}

/* This function disables filtering of non directed Data frames in hardware */
INLINE void disable_machw_non_directed_data_filter(void)
{
#ifndef NON_FC_MACHW_SUPPORT
    //chenq mod
    //rMAC_RX_FRAME_FILTER &= ~REGBIT16;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_RX_FRAME_FILTER ) & (~REGBIT16) ,
        (UWORD32)rMAC_RX_FRAME_FILTER );
#endif /* NON_FC_MACHW_SUPPORT */
}

/* This function enables filtering of non directed Ctrl frames in hardware */
INLINE void enable_machw_non_directed_ctrl_filter(void)
{
#ifndef NON_FC_MACHW_SUPPORT
    //chenq mod
    //rMAC_RX_FRAME_FILTER |= REGBIT17;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_RX_FRAME_FILTER ) | (REGBIT17) ,
        (UWORD32)rMAC_RX_FRAME_FILTER );
#endif /* NON_FC_MACHW_SUPPORT */
}

/* This function disables filtering of non directed Ctrl frames in hardware */
INLINE void disable_machw_non_directed_ctrl_filter(void)
{
#ifndef NON_FC_MACHW_SUPPORT
    //chenq mod
    //rMAC_RX_FRAME_FILTER &= ~REGBIT17;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_RX_FRAME_FILTER ) & (~REGBIT17) ,
        (UWORD32)rMAC_RX_FRAME_FILTER );
#endif /* NON_FC_MACHW_SUPPORT */
}

/* This function enables Other BSS Data frame filtering in hardware */
INLINE void enable_machw_obss_mgmt_filter(void)
{
    //chenq mod
    //rMAC_RX_FRAME_FILTER |= REGBIT8;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_RX_FRAME_FILTER ) | (REGBIT8) ,
        (UWORD32)rMAC_RX_FRAME_FILTER );
}

/* This function disables  Other BSS Data frame filtering in hardware */
INLINE void disable_machw_obss_mgmt_filter(void)
{
    //chenq mod
    //rMAC_RX_FRAME_FILTER &= ~REGBIT8;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_RX_FRAME_FILTER ) & (~REGBIT8) ,
        (UWORD32)rMAC_RX_FRAME_FILTER );
}

/* This function enables Other BSS Bcst Data frame filtering in hardware */
INLINE void enable_machw_obss_data_filter(void)
{
#ifndef NON_FC_MACHW_SUPPORT
    //chenq mod
    //rMAC_RX_FRAME_FILTER |= REGBIT15;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_RX_FRAME_FILTER ) | (REGBIT15) ,
        (UWORD32)rMAC_RX_FRAME_FILTER );
#endif /* NON_FC_MACHW_SUPPORT */
}

/* This function disables  Other BSS Bcst Data frame filtering in hardware */
INLINE void disable_machw_obss_data_filter(void)
{
#ifndef NON_FC_MACHW_SUPPORT
    //chenq mod
    //rMAC_RX_FRAME_FILTER &= ~REGBIT15;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_RX_FRAME_FILTER ) & (~REGBIT15) ,
        (UWORD32)rMAC_RX_FRAME_FILTER );
#endif /* NON_FC_MACHW_SUPPORT */
}

/* This function enables FCS failed frames filtering in hardware */
INLINE void enable_machw_fcs_fail_filter(void)
{
    //chenq mod
    //rMAC_RX_FRAME_FILTER |= REGBIT10;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_RX_FRAME_FILTER ) | (REGBIT10) ,
        (UWORD32)rMAC_RX_FRAME_FILTER );
}

/* This function disables FCS failed frames filtering in hardware */
INLINE void disable_machw_fcs_fail_filter(void)
{
    //chenq mod
    //rMAC_RX_FRAME_FILTER &= ~REGBIT10;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_RX_FRAME_FILTER ) & (~REGBIT10) ,
        (UWORD32)rMAC_RX_FRAME_FILTER );
}

/* This function checks if the FCS fail filter is enabled */
INLINE BOOL_T is_fcs_filter_on(void)
{
    /* Check for BIT 10 set or not */
    //chenq mod
    //if((rMAC_RX_FRAME_FILTER & REGBIT10) == REGBIT10)    
    if((host_read_trout_reg( (UWORD32)rMAC_RX_FRAME_FILTER  ) 
        & REGBIT10) == REGBIT10)
        return BTRUE;

    return BFALSE;
}

/* This function enables Deauth filtering in hardware */
INLINE void enable_machw_deauth_filter(void)
{
    //chenq mod
    //rMAC_RX_FRAME_FILTER |= REGBIT11;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_RX_FRAME_FILTER ) | (REGBIT11) ,
        (UWORD32)rMAC_RX_FRAME_FILTER );
}

/* This function disables Deauth filtering in hardware */
INLINE void disable_machw_deauth_filter(void)
{
    //chenq mod
    //rMAC_RX_FRAME_FILTER &= ~REGBIT11;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_RX_FRAME_FILTER ) & (~REGBIT11) ,
        (UWORD32)rMAC_RX_FRAME_FILTER );
}

/* This function enables filtering BA frames which are not received as SIFS  */
/*  response                                                                 */
INLINE void enable_machw_nonsifs_responseba_filter(void)
{
    //chenq mod
    //rMAC_RX_FRAME_FILTER |= REGBIT12;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_RX_FRAME_FILTER ) | (REGBIT12) ,
        (UWORD32)rMAC_RX_FRAME_FILTER );
}

/* This function enables reporting of BA frames which are  not received as   */
/* SIFS response                                                             */
INLINE void disable_machw_nonsifs_responseba_filter(void)
{
    //chenq mod
    //rMAC_RX_FRAME_FILTER &= ~REGBIT12;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_RX_FRAME_FILTER ) & (~REGBIT12) ,
        (UWORD32)rMAC_RX_FRAME_FILTER );
}

/* This function checks if the expected BA filter is enabled */
INLINE BOOL_T is_expba_filter_on(void)
{
    /* Check for BIT 13 set or not */
    //chenq mod
    //if((rMAC_RX_FRAME_FILTER & REGBIT13) == REGBIT13)
    if( (host_read_trout_reg( (UWORD32)rMAC_RX_FRAME_FILTER  ) 
         & REGBIT13 ) == REGBIT13 )
        return BTRUE;

    return BFALSE;
}

/* This function enables filtering BA frames which are received as SIFS      */
/* response                                                                  */
INLINE void enable_machw_sifs_responseba_filter(void)
{
    //chenq mod
    //rMAC_RX_FRAME_FILTER |= REGBIT13;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_RX_FRAME_FILTER ) | (REGBIT13) ,
        (UWORD32)rMAC_RX_FRAME_FILTER );
}

/* This function enables reporting of BA frames which are received as SIFS   */
/*  response                                                                 */
INLINE void disable_machw_sifs_responseba_filter(void)
{
    //chenq mod
    //rMAC_RX_FRAME_FILTER &= ~REGBIT13;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_RX_FRAME_FILTER ) & (~REGBIT13) ,
        (UWORD32)rMAC_RX_FRAME_FILTER );
}


/* The base address to store the received packet and its descriptor is set   */
/* in rMAC_RX_BUFF_ADDR. It is updated by PA on reception of a packet.       */
INLINE void set_machw_rx_buff_addr(UWORD32 addr, UWORD8 q_num)
{
	if(addr & 0x3){
		printk("Q Add is not Dword align\n");
		BUG_ON(1);
	}
    if(q_num == HIGH_PRI_RXQ)
    {
        //chenq mod
        //rMAC_HIP_RX_BUFF_ADDR = convert_to_le(virt_to_phy_addr(addr));
        host_write_trout_reg( convert_to_le(virt_to_phy_addr(addr)), 
                              (UWORD32)rMAC_HIP_RX_BUFF_ADDR );
    }
    else if(q_num == NORMAL_PRI_RXQ)
    {
        //chenq mod
        //rMAC_RX_BUFF_ADDR = convert_to_le(virt_to_phy_addr(addr));
        host_write_trout_reg( convert_to_le(virt_to_phy_addr(addr)), 
                              (UWORD32)rMAC_RX_BUFF_ADDR );
    }
}

/* The base address to store the received packet and its descriptor is set   */
/* in rMAC_RX_BUFF_ADDR. It is updated by PA on reception of a packet.       */
INLINE UWORD32 get_machw_rx_buff_addr(UWORD8 q_num)
{
    UWORD32 temp = 0;

    if(q_num == HIGH_PRI_RXQ)
    {
        //chenq mod
        //temp = convert_to_le(rMAC_HIP_RX_BUFF_ADDR);
        temp = convert_to_le(host_read_trout_reg( 
                                (UWORD32)rMAC_HIP_RX_BUFF_ADDR ));
    }
    else if(q_num == NORMAL_PRI_RXQ)
    {
        //chenq mod
        //temp = convert_to_le(rMAC_RX_BUFF_ADDR);
        temp = convert_to_le(host_read_trout_reg( 
                                (UWORD32)rMAC_RX_BUFF_ADDR ));
    }

    return (phy_to_virt_addr(temp));
}

/* This function checks whether the MAC H/w Rx-Queue is empty. */
INLINE BOOL_T is_machw_rx_buff_null(UWORD8 q_num)
{
    //chenq mod
    //if((q_num == HIGH_PRI_RXQ) && (rMAC_HIP_RX_BUFF_ADDR == 0))
    if((q_num == HIGH_PRI_RXQ) && (host_read_trout_reg( (UWORD32)rMAC_HIP_RX_BUFF_ADDR) == 0))
            return BTRUE;

    //chenq mod
    //if((q_num == NORMAL_PRI_RXQ) && ( rMAC_RX_BUFF_ADDR == 0))
    if((q_num == NORMAL_PRI_RXQ) && (host_read_trout_reg( (UWORD32)rMAC_RX_BUFF_ADDR) == 0))
            return BTRUE;

    return BFALSE;
}

/* This function reads the FCS Error Count from the register */
INLINE UWORD32 get_fcs_count(void)
{
    //chenq mod
    //return (convert_to_le(rMAC_FCS_FAIL_COUNT));
    return ( convert_to_le(host_read_trout_reg( (UWORD32)rMAC_FCS_FAIL_COUNT)) );
}

/* This function sets the Duplicate Detected Count register */
INLINE void set_fcs_count(UWORD32 inp)
{
    //chenq mod
    //rMAC_FCS_FAIL_COUNT = inp;
    host_write_trout_reg( inp, (UWORD32)rMAC_FCS_FAIL_COUNT );
}


/*dumy add for Rx Counter Registers*/
/*==========================================================================*/
INLINE UWORD32 get_rx_frame_filter_count(void)
{
    return (convert_to_le(host_read_trout_reg((UWORD32)rMAC_RX_FRAME_FILTER_COUNTER)));
}

INLINE void set_rx_frame_filter_count(UWORD32 inp)
{
    host_write_trout_reg(inp, (UWORD32)rMAC_RX_FRAME_FILTER_COUNTER);
}

INLINE UWORD32 get_rx_mac_header_filter_count(void)
{
    return (convert_to_le(host_read_trout_reg((UWORD32)rMAC_RX_MAC_HEADER_FILTER_COUNTER)));
}

INLINE void set_rx_mac_header_filter_count(UWORD32 inp)
{
    host_write_trout_reg(inp, (UWORD32)rMAC_RX_MAC_HEADER_FILTER_COUNTER);
}

INLINE UWORD32 get_rxq_full_filter_count(void)
{
    return (convert_to_le(host_read_trout_reg((UWORD32)rMAC_RXQ_FULL_COUNTER)));
}

INLINE void set_rxq_full_filter_count(UWORD32 inp)
{
    host_write_trout_reg(inp, (UWORD32)rMAC_RXQ_FULL_COUNTER);
}

INLINE UWORD32 get_rx_ram_packet_count(void)
{
    return (convert_to_le(host_read_trout_reg((UWORD32)rMAC_RX_RAM_PACKET_COUNTER)));
}

INLINE void set_rx_ram_packet_count(UWORD32 inp)
{
    host_write_trout_reg(inp, (UWORD32)rMAC_RX_RAM_PACKET_COUNTER);
}

INLINE UWORD16 get_tx_complete_count(void)
{
	return (convert_to_le(host_read_trout_reg((UWORD32)rMAC_TX_RX_COMPLETE_CNT) & 0xFFFF));
}

INLINE UWORD16 get_rx_complete_count(void)
{
	return ((convert_to_le(host_read_trout_reg((UWORD32)rMAC_TX_RX_COMPLETE_CNT) >> 16) & 0xFFFF));
}
/*==========================================================================*/


/* This function reads the Duplicate Detected Count from the register */
INLINE UWORD16 get_dup_count(void)
{
    //chenq mod
    //return (UWORD16)(convert_to_le(rMAC_DUP_DET_COUNT));
    return (UWORD16)(convert_to_le(host_read_trout_reg( 
                                        (UWORD32)rMAC_DUP_DET_COUNT)));
}

/* This function sets the Duplicate Detected Count register */
INLINE void set_dup_count(UWORD32 inp)
{
    //chenq mod
    //rMAC_DUP_DET_COUNT = inp;
    host_write_trout_reg( inp, (UWORD32)rMAC_DUP_DET_COUNT );
}

/* Inline functions to get/set/use EDCA registers in the PA register bank    */

/* The AIFSN register has various subfields described below.                 */
/*                                                                           */
/* 31 - 16     15 - 12        11 - 8           7 - 4          3 - 0          */
/* Reserved  AIFSN_AC_BK   AIFSN_AC_BE       AIFSN_AC_VI      AIFSN_AC_VO    */

#ifdef MAC_WMM

INLINE void set_machw_aifsn_all_ac(UWORD8 bk, UWORD8 be, UWORD8 vi, UWORD8 vo)
{
    UWORD32 temp = 0;

    temp = ((bk << 12) | (be << 8) | (vi << 4) | vo);

    //chenq mod
    //rMAC_AIFSN = convert_to_le(temp);
    host_write_trout_reg( convert_to_le(temp), (UWORD32)rMAC_AIFSN );
}

#endif /* MAC_WMM */

/* The AIFS value for 11a/b/g needs to be set in VO category only. */
INLINE void set_machw_aifsn(void)
{
    //chenq mod
    //rMAC_AIFSN = convert_to_le(0x02);
    host_write_trout_reg( convert_to_le(0x02), (UWORD32)rMAC_AIFSN );
}

#ifdef BURST_TX_MODE
/* The AIFS value for Burst transmission mode                      */
INLINE void set_machw_aifsn_burst_mode(void)
{
    //chenq mod
    //rMAC_AIFSN = convert_to_le(0x0A);
    host_write_trout_reg( convert_to_le(0x0A), (UWORD32)rMAC_AIFSN );
}
#endif /* BURST_TX_MODE */

/* The AIFSN register has various subfields described below.                 */
/*                                                                           */
/* 31 - 16     15 - 12        11 - 8           7 - 4           3 - 0         */
/* Reserved  AIFSN_AC_BK    AIFSN_AC_BE      AIFSN_AC_VI     AIFSN_AC_VO     */

#ifdef MAC_WMM

INLINE void set_machw_cw_vo(UWORD8 cmax, UWORD8 cmin)
{
    UWORD32 temp = (cmax << 4) | (cmin);
#if 0
#ifdef IBSS_BSS_STATION_MODE	
	if(g_cmcc_test_mode == 1)
		temp = 0x22;
#endif	
#endif
    //chenq mod
    //rMAC_CW_MIN_MAX_AC_VO = convert_to_le(temp);
    host_write_trout_reg( convert_to_le(temp),
                          (UWORD32)rMAC_CW_MIN_MAX_AC_VO );
    // host_write_trout_reg( 0x00000022,
     //                     (UWORD32)rMAC_CW_MIN_MAX_AC_VO );
}

INLINE void set_machw_cw_vi(UWORD8 cmax, UWORD8 cmin)
{
    UWORD32 temp = (cmax << 4) | (cmin);
#if 0
#ifdef IBSS_BSS_STATION_MODE	
	if(g_cmcc_test_mode == 1)
		temp = 0x22;
#endif
#endif

    //chenq mod
    //rMAC_CW_MIN_MAX_AC_VI = convert_to_le(temp);
    host_write_trout_reg( convert_to_le(temp),
                          (UWORD32)rMAC_CW_MIN_MAX_AC_VI );
    //host_write_trout_reg( 0x00000022,
     //                     (UWORD32)rMAC_CW_MIN_MAX_AC_VI );
}

INLINE void set_machw_cw_be(UWORD8 cmax, UWORD8 cmin)
{
    UWORD32 temp = (cmax << 4) | (cmin);
#if 0
#ifdef IBSS_BSS_STATION_MODE	
	if(g_cmcc_test_mode == 1)
		temp = 0x22;
#endif
#endif

    //chenq mod
    //rMAC_CW_MIN_MAX_AC_BE = convert_to_le(temp);
    host_write_trout_reg( convert_to_le(temp),
                          (UWORD32)rMAC_CW_MIN_MAX_AC_BE );
    //host_write_trout_reg( 0x00000022,
      //                    (UWORD32)rMAC_CW_MIN_MAX_AC_BE );
}

INLINE void set_machw_cw_bk(UWORD8 cmax, UWORD8 cmin)
{
    UWORD32 temp = (cmax << 4) | (cmin);
#if 0
#ifdef IBSS_BSS_STATION_MODE	
	if(g_cmcc_test_mode == 1)
		temp = 0x22;
#endif
#endif
	
	//chenq mod
    //rMAC_CW_MIN_MAX_AC_BK = convert_to_le(temp);
    host_write_trout_reg( convert_to_le(temp), 
                          (UWORD32)rMAC_CW_MIN_MAX_AC_BK );
}

INLINE void set_machw_txop_limit_bkbe(UWORD16 bk, UWORD16 be)
{
    UWORD32 temp = ((UWORD32)be << 16) | bk;

    //chenq mod
    //rMAC_EDCA_TXOP_LIMIT_AC_BKBE = convert_to_le(temp);
    host_write_trout_reg( convert_to_le(temp), 
                          (UWORD32)rMAC_EDCA_TXOP_LIMIT_AC_BKBE );
}

INLINE void set_machw_txop_limit_vovi(UWORD16 vo, UWORD16 vi)
{
    UWORD32 temp = ((UWORD32)vo << 16) | vi;

    //chenq mod
    //rMAC_EDCA_TXOP_LIMIT_AC_VIVO = convert_to_le(temp);
    host_write_trout_reg( convert_to_le(temp), 
                          (UWORD32)rMAC_EDCA_TXOP_LIMIT_AC_VIVO );
}

#endif /* MAC_WMM */

INLINE void set_machw_cw(UWORD8 cmax, UWORD8 cmin)
{
    //UWORD32 temp = (cmax << 4) | (cmin);

    //chenq mod
    //rMAC_CW_MIN_MAX_AC_VO = convert_to_le(temp);
    //host_write_trout_reg( convert_to_le(temp),
    //                      (UWORD32)rMAC_CW_MIN_MAX_AC_VO );

	host_write_trout_reg( 0x00000022,
                          (UWORD32)rMAC_CW_MIN_MAX_AC_VO );
}


/* The EDCA BK, BE lifetime limit register has 2 subfields described below.  */
/*                                                                           */
/* 31 - 16                        | 15 - 0                                   */
/* AC BE EDCA Lifetime limit      | AC BK EDCA Lifetime limit                */

INLINE void set_machw_edca_bkbe_lifetime(UWORD16 bk, UWORD16 be)
{
    UWORD32 temp = (be << 16) | bk;

    //chenq mod
    //rMAC_EDCA_BK_BE_LIFETIME = convert_to_le(temp);
    host_write_trout_reg( convert_to_le(temp), 
                          (UWORD32)rMAC_EDCA_BK_BE_LIFETIME );
}

/* The EDCA VI, VO lifetime limit register has 2 subfields described below.  */
/*                                                                           */
/* 31 - 16                        | 15 - 0                                   */
/* AC VO EDCA Lifetime limit      | AC VI EDCA Lifetime limit                */

INLINE void set_machw_edca_vivo_lifetime(UWORD16 vi, UWORD16 vo)
{
    UWORD32 temp = (vo << 16) | vi;

    //chenq mod
    //rMAC_EDCA_VI_VO_LIFETIME = convert_to_le(temp);
    host_write_trout_reg( convert_to_le(temp), 
                          (UWORD32)rMAC_EDCA_VI_VO_LIFETIME );
}

/* Inline functions to get/set/use TSF registers in the PA register bank     */

/* The TSF Control register has various subfields described below.           */
/*                                                                           */
/* 31 - 3    2              1              0                                 */
/* Reserved  Beacon Suspend Start/Join~    TSF Enable                        */
/*                                                                           */
/* TSF Enable   0 - This function is disabled. TBTT is not generated.        */
/*              1 - This function is enabled.                                */
/* Start/Join~  0 - After being enabled, the TSF waits to receive Beacon or  */
/*                  Probe Response frame with matching BSSID, adopts the     */
/*                  parameters and starts the TSF timer and TBTT generation. */
/*              1 - After being enabled, the TSF immediately starts the TSF  */
/*                  timer and TBTT generation.                               */
/*                  This bit is interpreted when TSF Enable is set to 1.     */

INLINE void set_machw_tsf_beacon_tx_suspend_enable(void)
{
    //chenq mod
    //rMAC_TSF_CON |= REGBIT2;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_TSF_CON ) | (REGBIT2) ,
        (UWORD32)rMAC_TSF_CON );
}

INLINE void set_machw_tsf_beacon_tx_suspend_disable(void)
{
    //chenq mod
    //rMAC_TSF_CON &= (~REGBIT2);
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_TSF_CON ) & (~REGBIT2) ,
        (UWORD32)rMAC_TSF_CON );
}

INLINE void set_machw_tsf_ctrl(UWORD32 val)
{
    //chenq mod
    //rMAC_TSF_CON = convert_to_le(val);
    host_write_trout_reg( convert_to_le(val),(UWORD32)rMAC_TSF_CON);
}

INLINE UWORD32 get_machw_tsf_ctrl(void)
{
    //chenq mod
    //return convert_to_le(rMAC_TSF_CON);
    return convert_to_le(host_read_trout_reg( (UWORD32)rMAC_TSF_CON ));
}

INLINE void get_machw_tsf_timer(UWORD32 *hi, UWORD32 *lo)
{
    //chenq mod
    //*lo = convert_to_le((UWORD32)rMAC_TSF_TIMER_LO);
    //*hi = convert_to_le((UWORD32)rMAC_TSF_TIMER_HI);
    *lo = convert_to_le(
            (UWORD32)host_read_trout_reg( (UWORD32)rMAC_TSF_TIMER_LO ));
    *hi = convert_to_le(
            (UWORD32)host_read_trout_reg( (UWORD32)rMAC_TSF_TIMER_HI ));
}

INLINE void set_machw_tsf_timer(UWORD32 hi, UWORD32 lo)
{
    //chenq mod
    //rMAC_TSF_TIMER_HI = convert_to_le(hi);
    //rMAC_TSF_TIMER_LO = convert_to_le(lo);
    host_write_trout_reg( convert_to_le(hi), (UWORD32)rMAC_TSF_TIMER_HI );
    host_write_trout_reg( convert_to_le(lo), (UWORD32)rMAC_TSF_TIMER_LO );
}

INLINE UWORD32 get_machw_tsf_timer_lo(void)
{
    //chenq mod
    //return convert_to_le((UWORD32)rMAC_TSF_TIMER_LO);
    return convert_to_le(
                (UWORD32)host_read_trout_reg( (UWORD32)rMAC_TSF_TIMER_LO ));
}

/* The beacon period register is set to the current beacon period. This will */
/* be set only at initialization. On change of beacon period reset will be   */
/* done.                                                                     */
INLINE void set_machw_beacon_period(UWORD16 beacon_period)
{
    //chenq mod
    //rMAC_BEACON_PERIOD = convert_to_le((UWORD32)beacon_period);
    host_write_trout_reg( convert_to_le((UWORD32)beacon_period), 
                          (UWORD32)rMAC_BEACON_PERIOD );
}

/* The DTIM period register is set to the current DTIM period. This will     */
/* be set only at initialization. On change of DTIM period reset will be     */
/* done.                                                                     */
INLINE void set_machw_dtim_period(UWORD8 dtim_period)
{
    //chenq mod
    //rMAC_DTIM_PERIOD = convert_to_le((UWORD32)dtim_period);
    host_write_trout_reg( convert_to_le((UWORD32)dtim_period), 
                          (UWORD32)rMAC_DTIM_PERIOD );
}

/* The beacon pointer register contains the address of the location of       */
/* beacon packet descriptor.                                                 */
INLINE void set_machw_beacon_pointer(UWORD32 beacon_pointer)
{
    UWORD32 temp = virt_to_phy_addr((UWORD32)beacon_pointer);

    //chenq mod
    //rMAC_BEACON_POINTER = convert_to_le(temp);
    host_write_trout_reg( convert_to_le(temp), 
                          (UWORD32)rMAC_BEACON_POINTER );
}

INLINE UWORD32 get_machw_beacon_pointer(void)
{
    //chenq mod
    //UWORD32 temp = convert_to_le(rMAC_BEACON_POINTER);
    UWORD32 temp = convert_to_le(
                        host_read_trout_reg( (UWORD32)rMAC_BEACON_POINTER));

    return (phy_to_virt_addr(temp));
}

/* The beacon transmit parameters register has various subfields described   */
/* below.                                                                    */
/*                                                                           */
/* 31 - 24                23 - 16               15 - 0                       */
/* Beacon Tx Power level  Beacon Tx Data Rate   Beacon Tx Length             */
INLINE void set_machw_beacon_tx_params(UWORD16 len, UWORD8 rate)
{
    UWORD32 temp = 0;
    UWORD8  tx_pwr = 0;

    tx_pwr = get_tx_pow(rate, 0);

    temp = (tx_pwr << 24) | (rate << 16) | len;

    //chenq mod
    //rMAC_BEACON_TX_PARAMS = convert_to_le(temp);
    host_write_trout_reg( convert_to_le(temp), 
                          (UWORD32)rMAC_BEACON_TX_PARAMS );
}

INLINE void set_machw_beacon_ptm(UWORD32 ptm)
{
    //chenq mod
    //rMAC_BEACON_PHY_TX_MODE = convert_to_le(ptm);
    host_write_trout_reg( convert_to_le(ptm), 
                          (UWORD32)rMAC_BEACON_PHY_TX_MODE );
}

INLINE UWORD8 get_machw_dtim_count(void)
{
    //chenq mod
    //return convert_to_le(rMAC_DTIM_COUNT);
    return convert_to_le(host_read_trout_reg( (UWORD32)rMAC_DTIM_COUNT));
}

/* This function extracts the DTIM count from the received beacon and        */
/* sets it in this register.                                                 */
INLINE void set_machw_ap_dtim_cnt(UWORD8 dtim_cnt)
{
    //chenq mod
    //rMAC_AP_DTIM_COUNT = convert_to_le((UWORD32)dtim_cnt);
    host_write_trout_reg(  convert_to_le((UWORD32)dtim_cnt), 
                           (UWORD32)rMAC_AP_DTIM_COUNT );
}

/* Inline functions to get/set Protection And SIFS Response registers in the */
/* PA register bank                                                          */

/* The Protection control register has various subfields described below.    */
/*                                                                           */
/* 31 -6    5             4        3             2                 1 - 0     */
/* Reserved ERP Prot Type ERP Prot NAV Prot Type Enhanced NAV Prot Mode      */
/*                                                                           */
/* Mode                 00 - 802.11b physical layer                          */
/*                      01 - 802.11a physical layer                          */
/*                      10 - 802.11g Compatibility                           */
/*                      11 - 802.11g High Rate                               */
/* Enhanced NAV Prot    0 - No enhanced protection                           */
/*                      1 - This overrides RTS threshold and mode bits.      */
/*                          Protection frame is transmitted for every burst  */
/*                      This is valid only for 11e                           */
/* NAV Prot Type        0 - RTS/CTS                                          */
/*                      1 - Self CTS                                         */
/*                      This is valid only if Enhanced NAV Protection is set */
/* ERP Prot             0 - No ERP protection                                */
/*                      1 - Protection required for OFDM frames if Mode = 10 */
/*                      This is valid only for Mode = 10                     */
/* ERP Prot Type        0 - RTS/CTS                                          */
/*                      1 - Self CTS                                         */
/*                      This is valid only if ERP Protection is set.         */
INLINE void set_machw_prot_control(void)
{
    UWORD32 temp = get_phy_prot_con();

    //chenq mod
    //rMAC_PROT_CON = convert_to_le(temp);
    host_write_trout_reg( convert_to_le(temp), (UWORD32)rMAC_PROT_CON );

    /* Based on the MAC protocol in use the protection control is modified */
    /* set_mac_prot_control();  -> enhanced prot needs to be set */
}

INLINE void enable_machw_self_cts_control(void)
{
	UWORD32 value = host_read_trout_reg((UWORD32)rMAC_PROT_CON) | 0x32;
	host_write_trout_reg(value, (UWORD32)rMAC_PROT_CON);
}

INLINE void disable_machw_self_cts_control(void)
{
	UWORD32 value = host_read_trout_reg((UWORD32)rMAC_PROT_CON) & (~0x32);
	host_write_trout_reg(value, (UWORD32)rMAC_PROT_CON);
}


/* The RTS threshold register contains the RTS threshold. This needs to be   */
/* updated whenever RTS threshold is changed.                                */
INLINE void set_machw_rts_thresh(UWORD16 rts_thresh)
{
    //chenq mod
    //rMAC_RTS_THRESH = convert_to_le((UWORD32)rts_thresh);
    host_write_trout_reg( convert_to_le((UWORD32)rts_thresh), 
                          (UWORD32)rMAC_RTS_THRESH );
}

/* The minimum TXOP fragmentation limit is set to the register. In normal    */
/* operation this is not set by S/w and default value of 256 is used in H/w. */
/* Used for H/w unit test.                                                   */
INLINE void set_machw_min_txop_frag(UWORD16 thresh)
{
    //chenq mod
    //rMAC_FRAG_THRESH &= MASK_INVERSE(16, 16);
    //rMAC_FRAG_THRESH |= convert_to_le((thresh << 16)) & MASK(16, 16);
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_FRAG_THRESH ) 
            & (MASK_INVERSE(16, 16)) ,
        (UWORD32)rMAC_FRAG_THRESH );
    
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_FRAG_THRESH ) 
            | (convert_to_le((thresh << 16)) & MASK(16, 16)) ,
        (UWORD32)rMAC_FRAG_THRESH );
}

/* The fragmentation threshold register contains the fragmentation threshold */
/* 31 - 28  27 - 16                       15 - 12   11 - 0                   */
/* Reserved Minimum TXOP fragment length  Reserved  Fragmentation Threshold  */

INLINE void set_machw_frag_thresh(UWORD16 frag_thresh)
{
    //chenq mod
    //rMAC_FRAG_THRESH &= MASK_INVERSE(16, 0);
    //rMAC_FRAG_THRESH |= convert_to_le(frag_thresh) & MASK(16, 0);
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_FRAG_THRESH ) 
            & (MASK_INVERSE(16, 0)) ,
        (UWORD32)rMAC_FRAG_THRESH );
    
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_FRAG_THRESH ) 
            | (convert_to_le(frag_thresh) & MASK(16, 0)) ,
        (UWORD32)rMAC_FRAG_THRESH );

    /* Disable TXOP Fragmentation */
    set_machw_min_txop_frag(frag_thresh);
}

/* The HT control register has various subfields described below.            */
/*                                                                           */
/*      8   7          6           5            4 - 3      2           1 - 0 */
/* EnableFC DisRTS-CTS RIFSBurstEn NonGFSTAPres HTProtType LongNAVSupp HTOp  */
/*                                                                           */
/*     12              11                  10                 9              */
/* FCProtType 40MHzDemAtTxopMiddle 40MHzDemAtTxopStart  40MHzTxopProm        */
/*                                                                           */
/*       31 - 16         15                  14                13            */
/*       Reserved LsigNavupdateEnable FirstframeTXOPLimit SecChanOffset      */
/*                                                                           */
/* HT Operation Mode    Indicates the HT Operation mode (0 - 4 as per MIB)   */
/* Long NAV Support     0 - Long NAV supported                               */
/*                      1 - Long NAV not supported                           */
/* HT Protection Type   0 - RTS-CTS Frame Exchange at non-HT-rate            */
/*                      1 - First Frame Exchange at non-HT-rate              */
/*                      2 - LSIG TXOP                                        */
/*                      3 - First Frame Exchange in Mixed Format             */
/* NonGF STA Present    0 - Non GF HT STA present                            */
/*                      1 - Non GF HT STA not present                        */
/*                      If HTOpMode is 0 or 2, protection will be done based */
/*                      on this bit. HTProtType will be ignored.             */
/* RIFSBurstEnable      0 - MAC HW will not perform RIFS burst               */
/*                      1 - MAC HW will perform RIFS burst                   */
/* DisableRTS-CTS       0 - RTS-CTS will be performed for frames with        */
/*                          NO_ACK or BA ACK policy when they are in the     */
/*                          start of TXOP                                    */
/*                      1 - RTS-CTS will not be performed for frames with    */
/*                          NO_ACK or BA ACK policy when they are in the     */
/*                          start of TXOP                                    */
/* EnableFC             0 - 20/40 MHz Co-existence mode of operation is      */
/*                          disabled                                         */
/*                      1 - 20/40 MHz Co-existence mode of operation is      */
/*                          enabled                                          */
/* 40MHzTxopProm        0 - TXOP promotions are not enabled. MAC H/W will    */
/*                          not try to promote a 20 MHz TXOP into a 40 MHz   */
/*                          TXOP                                             */
/*                      1 - MAC H/W tries to promote every TXOP into a 40MHz */
/*                          TXOP.                                            */
/* 40MHzDemAtTxopStart  0 - TXOP demotions are not enabled                   */
/*                      1 - MAC H/W will try to start a 20 MHz TXOP when     */
/*                          secondary channel is busy and the first frame in */
/*                          the TXOP is a 40MHz frame                        */
/* 40MHzDemAtTxopMiddle 0 - PPDU demotions are not performed.                */
/*                      1 - When a 20 MHz TXOP is in progress then MAC H/W   */
/*                          demotes all 40 MHz PPDU to 20 MHz PPDU and       */
/*                          continues the TXOP burst without relinquishing.  */
/* FCProtType           The protection type specified below will be used     */
/*                      when a protection frame exchange needs to be         */
/*                      performed to promote the TXOP to a 40MHz TXOP.       */
/*                      0 - RTS/CTS                                          */
/*                      1 - Self CTS                                         */
/* 40MHzDemAtTxopStart  0 - Secondary channel corresponds to Upper 20 MHz in */
/*                          40MHz Band                                       */
/*                      1 - Secondary channel corresponds to Lower 20 MHz in */
/*                          40MHz Band                                       */
/* First frame TXOP limit check                                              */
/*                      0 - TXOP limit check is not done for the first frame */
/*                          in the TXOP                                      */
/*                      1 - First frame in the TXOP is subjected to TXOP     */
/*                          limit                                            */
/* LsigNav UpdateEnable                                                      */
/*                      0 - MAC HW does not update NAV based on information  */
/*                          from LSIG field from HT_MF frames.               */
/*                      1 - MAC HW updates NAV based on information from LSIG*/
/*                          field from HT_MF frames                          */


INLINE void set_machw_ht_op_mode(UWORD8 val)
{
    //chenq mod
    //rMAC_HT_CTRL &= MASK_INVERSE(2, 0);
    //rMAC_HT_CTRL |= convert_to_le(val) & MASK(2, 0);

    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_HT_CTRL ) 
            & (MASK_INVERSE(2, 0)) ,
        (UWORD32)rMAC_HT_CTRL );

    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_HT_CTRL ) 
            | (convert_to_le(val) & MASK(2, 0)) ,
        (UWORD32)rMAC_HT_CTRL );
    
}

INLINE void enable_machw_long_nav_support(void)
{
    //chenq mod
    //rMAC_HT_CTRL &= ~REGBIT2;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_HT_CTRL ) & (~REGBIT2) ,
        (UWORD32)rMAC_HT_CTRL );
}

INLINE void disable_machw_long_nav_support(void)
{
    //chenq mod
    //rMAC_HT_CTRL |= REGBIT2;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_HT_CTRL ) | (REGBIT2) ,
        (UWORD32)rMAC_HT_CTRL );
}

INLINE void set_machw_ht_prot_type(UWORD8 val)
{
    //chenq mod
    //rMAC_HT_CTRL &= MASK_INVERSE(2, 3);
    //rMAC_HT_CTRL |= convert_to_le(val << 3) & MASK(2, 3);

    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_HT_CTRL ) 
            & (MASK_INVERSE(2, 3)) ,
        (UWORD32)rMAC_HT_CTRL );

    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_HT_CTRL ) 
            | (convert_to_le(val << 3) & MASK(2, 3)) ,
        (UWORD32)rMAC_HT_CTRL );    
}

INLINE void enable_machw_gf_rifs_prot(void)
{
    //chenq mod
    //rMAC_HT_CTRL &= ~REGBIT5;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_HT_CTRL ) & (~REGBIT5) ,
        (UWORD32)rMAC_HT_CTRL );
}

INLINE void disable_machw_gf_rifs_prot(void)
{
    //chenq mod
    //rMAC_HT_CTRL |= REGBIT5;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_HT_CTRL ) | (REGBIT5) ,
        (UWORD32)rMAC_HT_CTRL );
}

INLINE void enable_machw_rifs(void)
{
    //chenq mod
    //rMAC_HT_CTRL |= REGBIT6;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_HT_CTRL ) | (REGBIT6) ,
        (UWORD32)rMAC_HT_CTRL );
}

INLINE void disable_machw_rifs(void)
{
    //chenq mod
    //rMAC_HT_CTRL &= ~REGBIT6;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_HT_CTRL ) & (~REGBIT6) ,
        (UWORD32)rMAC_HT_CTRL );
}

INLINE void disable_machw_rtscts_norsp(void)
{
    //chenq mod
    //rMAC_HT_CTRL |= REGBIT7;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_HT_CTRL ) | (REGBIT7) ,
        (UWORD32)rMAC_HT_CTRL );
}

INLINE void enable_machw_rtscts_norsp(void)
{
    //chenq mod
    //rMAC_HT_CTRL &= ~REGBIT7;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_HT_CTRL ) & (~REGBIT7) ,
        (UWORD32)rMAC_HT_CTRL );
}

INLINE BOOL_T is_machw_rtscts_norsp_disabled(void)
{
    //chenq mod
    //if(rMAC_HT_CTRL & REGBIT7)
    if( host_read_trout_reg( (UWORD32)rMAC_HT_CTRL ) & REGBIT7 )
    {
        return BTRUE;
    }

    return BFALSE;
}

INLINE void enable_machw_fc(void)
{
    //chenq mod
    //rMAC_HT_CTRL |= REGBIT8;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_HT_CTRL ) | (REGBIT8) ,
        (UWORD32)rMAC_HT_CTRL );
}

INLINE void disable_machw_fc(void)
{
    //chenq mod
    //rMAC_HT_CTRL &= ~REGBIT8;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_HT_CTRL ) & (~REGBIT8) ,
        (UWORD32)rMAC_HT_CTRL );
}

INLINE void enable_machw_fmz_txop_prom(void)
{
    //chenq mod
    //rMAC_HT_CTRL |= REGBIT9;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_HT_CTRL ) | (REGBIT9) ,
        (UWORD32)rMAC_HT_CTRL );
}

INLINE void disable_machw_fmz_txop_prom(void)
{
    //chenq mod
    //rMAC_HT_CTRL &= ~REGBIT9;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_HT_CTRL ) & (~REGBIT9) ,
        (UWORD32)rMAC_HT_CTRL );
}

INLINE void enable_machw_fmz_txop_start_dem(void)
{
    //chenq mod
    //rMAC_HT_CTRL |= REGBIT10;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_HT_CTRL ) | (REGBIT10) ,
        (UWORD32)rMAC_HT_CTRL );
}

INLINE void disable_machw_fmz_txop_start_dem(void)
{
    //chenq mod
    //rMAC_HT_CTRL &= ~REGBIT10;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_HT_CTRL ) & (~REGBIT10) ,
        (UWORD32)rMAC_HT_CTRL );
}

INLINE void enable_machw_fmz_txop_mid_dem(void)
{
    //chenq mod
    //rMAC_HT_CTRL |= REGBIT11;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_HT_CTRL ) | (REGBIT11) ,
        (UWORD32)rMAC_HT_CTRL );
}

INLINE void disable_machw_fmz_txop_mid_dem(void)
{
    //chenq mod
    //rMAC_HT_CTRL &= ~REGBIT11;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_HT_CTRL ) & (~REGBIT11) ,
        (UWORD32)rMAC_HT_CTRL );
}

INLINE void set_machw_fmz_txop_prot_type(UWORD8 val)
{
    //chenq mod
    //rMAC_HT_CTRL &= MASK_INVERSE(1, 12);
    //rMAC_HT_CTRL |= convert_to_le((val << 12)) & MASK(1, 12);

    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_HT_CTRL ) 
            & (MASK_INVERSE(1, 12)) ,
        (UWORD32)rMAC_HT_CTRL );

    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_HT_CTRL ) 
            | (convert_to_le((val << 12)) & MASK(1, 12)) ,
        (UWORD32)rMAC_HT_CTRL );
}

INLINE void set_machw_fmz_sec_chan_offset(UWORD8 val)
{
    //chenq mod
    //rMAC_HT_CTRL &= MASK_INVERSE(1, 13);
    //rMAC_HT_CTRL |= convert_to_le((val << 13)) & MASK(1, 13);

    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_HT_CTRL ) 
            & (MASK_INVERSE(1, 13)) ,
        (UWORD32)rMAC_HT_CTRL );

    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_HT_CTRL ) 
            | (convert_to_le((val << 13)) & MASK(1, 13)) ,
        (UWORD32)rMAC_HT_CTRL );
}

INLINE void enable_machw_first_frm_txopcheck(void)
{
    //chenq mod
    //rMAC_HT_CTRL |= REGBIT14;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_HT_CTRL ) | (REGBIT14) ,
        (UWORD32)rMAC_HT_CTRL );
}

INLINE void disable_machw_first_frm_txopcheck(void)
{
    //chenq mod
    //rMAC_HT_CTRL &= ~REGBIT14;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_HT_CTRL ) & (~REGBIT14) ,
        (UWORD32)rMAC_HT_CTRL );
}

INLINE void enable_machw_lsignav_update(void)
{
    //chenq mod
    //rMAC_HT_CTRL |= REGBIT15;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_HT_CTRL ) | (REGBIT15) ,
        (UWORD32)rMAC_HT_CTRL );
}

INLINE void disable_machw_lsignav_update(void)
{
    //chenq mod
    //rMAC_HT_CTRL &= ~REGBIT15;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_HT_CTRL ) & (~REGBIT15) ,
        (UWORD32)rMAC_HT_CTRL );
}

/* This function sets the MAC H/w HT control register with all parameters */
INLINE void set_machw_ht_control(void)
{
#ifdef MAC_802_11N
    //chenq mod
    //rMAC_HT_CTRL = 0;
    host_write_trout_reg( 0 , (UWORD32)rMAC_HT_CTRL );


    set_machw_ht_op_mode(mget_HTOperatingMode());

    set_machw_ht_prot_type(get_11n_ht_prot_type());

    disable_machw_long_nav_support();

    if((mget_RIFSMode() == TV_TRUE) && (g_user_allow_rifs_tx == 1))
        enable_machw_rifs();
    else
        disable_machw_rifs();

    if(get_protection() == GF_PROT)
        enable_machw_gf_rifs_prot();
    else
        disable_machw_gf_rifs_prot();

    /* Disable RTS Txn before TXOP burst */
    if(g_disable_rtscts_norsp == 1)
        disable_machw_rtscts_norsp();

    /* Disable TXOP limit check for the first frame in the TXOP */
    disable_machw_first_frm_txopcheck();

    /* Enable MAC HW updates NAV based on information from LSIG field from */
    /* HT_MF frames                                                        */
    enable_machw_lsignav_update();
#endif /* MAC_802_11N */
}

/* The slot time register contains the short and long slot time and the slot */
/* select bit. This register has various subfields described below.          */
/*                                                                           */
/* 31 - 11          10 - 6          5 - 1                      0             */
/* Reserved         Long slot time  Short slot time            Slot select   */

INLINE void set_machw_slot_times(UWORD8 shortst, UWORD8 longst)
{
    UWORD32 temp = 0;

    temp = ((longst & 0x1F) << 6) | ((shortst & 0x1F) << 1);

    //chenq mod
    //rMAC_SLOT_TIME = convert_to_le(temp);
    host_write_trout_reg( convert_to_le(temp), (UWORD32)rMAC_SLOT_TIME );
}

INLINE void set_machw_short_slot_select(void)
{
    /* Reset BIT0 for selecting short slot */
    //chenq mod
    //rMAC_SLOT_TIME &= ~REGBIT0;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_SLOT_TIME ) & (~REGBIT0) ,
        (UWORD32)rMAC_SLOT_TIME );
}

INLINE void set_machw_long_slot_select(void)
{
    /* Set BIT0 for selecting long slot */
    //chenq mod
    //rMAC_SLOT_TIME |= REGBIT0;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_SLOT_TIME ) | (REGBIT0) ,
        (UWORD32)rMAC_SLOT_TIME );
}

/* The protection rate register contains the premable bit encoded. Reset the */
/* bit and set based on the preamble type given as input.                    */
INLINE void set_machw_prot_pream(UWORD8 pr)
{
    if(get_current_start_freq() == RC_START_FREQ_2)
    {
        //chenq mod
        //UWORD8  rate = convert_to_le(rMAC_PROT_RATE);
        UWORD8  rate = convert_to_le(
                            host_read_trout_reg( (UWORD32)rMAC_PROT_RATE));

        /* Preamble setting is valid only for 11b protection data rate */
        if(is_11b_rate(rate) == BTRUE)
        {
            /* Update the preamble bit */
            
            //rMAC_PROT_RATE &= ~REGBIT2;
            host_write_trout_reg(
                host_read_trout_reg( (UWORD32)rMAC_PROT_RATE ) & (~REGBIT2) ,
                (UWORD32)rMAC_PROT_RATE );
            if(pr)
                //rMAC_PROT_RATE |= REGBIT2;
                host_write_trout_reg(
                    host_read_trout_reg( (UWORD32)rMAC_PROT_RATE ) | (REGBIT2) ,
                    (UWORD32)rMAC_PROT_RATE );

            /* Update the PHY Tx Mode based on preamble value */

            //chenq mod
            //rMAC_PROT_TX_MODE = get_phy_prot_tx_mode(get_pr_to_ur(rate), pr);
            host_write_trout_reg( get_phy_prot_tx_mode(get_pr_to_ur(rate), pr),
                                  (UWORD32)rMAC_PROT_TX_MODE );
        }
        //chengwg test.
        //host_write_trout_reg((host_read_trout_reg((UWORD32)rMAC_PROT_RATE) & (~0xFF))|0x4, rMAC_PROT_RATE);
    }
}

/* Set the default parameters for Protection data rate, preamble, power and  */
/* PHY Tx mode in the required registers.                                    */
INLINE void set_default_machw_prot_params(void)
{
    UWORD32 temp = 0;
    UWORD8  dr   = 0;
    UWORD8  pr   = 0;
    UWORD8  tx_pwr = 0;

    /* The protection data rate depends on the PHY type */
    dr = get_prot_data_rate();
    pr = get_phy_rate(dr);
    tx_pwr = get_tx_pow(dr, 0);

    /* The protection rate register has various subfields described below.   */
    /* 31 - 16         15 - 8                       7 - 0                    */
    /* Reserved        Protection TX Power level    Protection Data Rate     */
    temp = (tx_pwr << 8) | pr;

    /* Set the power and data rate to the rate register */
    //chenq mod
    //rMAC_PROT_RATE = convert_to_le(temp);
    host_write_trout_reg(  convert_to_le(temp), (UWORD32)rMAC_PROT_RATE );

    /* The PHY Tx mode is based on the protection rate and the PHY type */
    temp = get_phy_prot_tx_mode(dr, 0);

    /* Set the PHY Tx mode to the protection PHY Tx mode register */

    //chenq mod
    //rMAC_PROT_TX_MODE = convert_to_le(temp);
    host_write_trout_reg( convert_to_le(temp), (UWORD32)rMAC_PROT_TX_MODE );

    //chengwg test.
    //host_write_trout_reg((host_read_trout_reg((UWORD32)rMAC_PROT_RATE) & (~0xFF))|0x4, rMAC_PROT_RATE);

}

/* Inline functions to get/set Retry registers in the PA register bank */

/* The Long Retry limit register contains limit on number of retransmissions */
/* of MPDU/MMPDU packets of length greater than RTS Threshold.               */
INLINE void set_machw_lrl(UWORD8 lrl)
{
    //chenq mod
    //rMAC_LONG_RETRY_LIMIT = convert_to_le((UWORD32)lrl);
    host_write_trout_reg( convert_to_le((UWORD32)lrl), 
                         (UWORD32)rMAC_LONG_RETRY_LIMIT );
}

/* Short Retry limit register contains limit on number of retransmissions of */
/* MPDU/MMPDU packets of length lesser than RTS Threshold. This applies to   */
/* retransmissions of RTS frame also.                                        */
INLINE void set_machw_srl(UWORD8 srl)
{
    //chenq mod
    //rMAC_SHORT_RETRY_LIMIT = convert_to_le((UWORD32)srl);
#ifdef TROUT_B2B_TEST_MODE
        srl = 1;
#endif

//#ifdef TROUT_WIFI_NPI
 //       srl = 1;
//#endif

    host_write_trout_reg( convert_to_le((UWORD32)srl), 
                          (UWORD32)rMAC_SHORT_RETRY_LIMIT );
}

/* Inline functions to get/set Sequence Number registers in PA register bank */

/* The Sequence Number and Duplicate Detection Control register has various  */
/* subfields described below.                                                */
/*                                                                           */
/* 31 - 13  12 - 8  7 - 4 3         2    1           0                       */
/* Reserved Index   TID   Reserved  QSTA Read~/Write Table Operation Enable  */
/*                                                                           */
/* Table Operation Enable   1 - A zero to one transition of this bit will    */
/*                              start the table update procedure.  After     */
/*                              table is updated this bit is reset to zero.  */
/*                          0 - No action.  Table update procedure over.     */
/* Read~/Write              0 - Read from table.                             */
/*                          1 - Write to table.This is valid only when Table */
/*                              Update Enable is set to one.                 */
/* QSTA                     0 - The entry is not a QSTA.  Only one sequence  */
/*                              control entry is maintained.                 */
/*                          1 - The entry is a QSTA.  Separate sequence      */
/*                              control entries are maintained for each TID  */
/* TID                      Range is 0 - 15. This is used while reading from */
/*                          table. Entry corresponding to the TID is read.   */
/* Index                    Range is 0 - 127. 128 STA entries are possible   */
/*                          in the table                                     */

INLINE void set_machw_seq_num_index_update(UWORD8 index, BOOL_T is_qos)
{
    UWORD32 temp = 0;

    /* Note that the station value must be set before this */
    temp  = (((UWORD32)index) << 8);
    temp |= (is_qos == BTRUE)? BIT2: 0;
    temp |= BIT1;
    temp |= BIT0;

    //chenq mod
    //rMAC_SEQ_NUM_CON = convert_to_le(temp);
    host_write_trout_reg( convert_to_le(temp), 
                         (UWORD32)rMAC_SEQ_NUM_CON );

}

INLINE UWORD32 get_machw_seq_num(UWORD8 index, UWORD8 tid)
{
    UWORD32 temp = 0;

    temp  = (((UWORD32)index) << 8);
    temp  = (((UWORD32)tid) << 4);
    temp &= ~BIT1;
    temp |= BIT0;

    //chenq mod
    //rMAC_SEQ_NUM_CON = convert_to_le(temp);
    host_write_trout_reg( convert_to_le(temp), 
                         (UWORD32)rMAC_SEQ_NUM_CON );

    while(host_read_trout_reg( (UWORD32)rMAC_SEQ_NUM_CON) & REGBIT0)
    {
        /* Wait for H/w to reset this bit */
    }

    //chenq mod
    //return convert_to_le(rMAC_TX_SEQ_NUM);
    return convert_to_le(host_read_trout_reg( (UWORD32)rMAC_TX_SEQ_NUM));
}

/* The MAC address of the STA entry to be added to the table is set in the 2 */
/* registers with the lower 32 bits set in rMAC_STA_ADDR_LO and higher 16    */
/* bits in rMAC_STA_ADDR_HI                                                  */
INLINE void set_machw_stamacaddr(UWORD8* mac_addr)
{
    UWORD32 mac_h = 0;
    UWORD32 mac_l = 0;

    mac_h = mac_addr[0];
    mac_h = (mac_h << 8) | mac_addr[1];

    mac_l = mac_addr[2];
    mac_l = (mac_l << 8) | mac_addr[3];
    mac_l = (mac_l << 8) | mac_addr[4];
    mac_l = (mac_l << 8) | mac_addr[5];

    //chenq mod
    //rMAC_STA_ADDR_HI = convert_to_le(mac_h);
    //rMAC_STA_ADDR_LO = convert_to_le(mac_l);
    host_write_trout_reg( convert_to_le(mac_h), 
                          (UWORD32)rMAC_STA_ADDR_HI );
    host_write_trout_reg( convert_to_le(mac_l), 
                          (UWORD32)rMAC_STA_ADDR_LO );
}

/* Inline functions to get/set Interrupt registers in PA register bank       */

/* The interrupt status register has various subfields described below.      */
/*                                                                           */
/* 9           8     7      6     5      4    3         2    1       0       */
/* RFVCOUnlock CFEnd CAPEnd Error WakeUp ATIM HCCA TXOP TBTT TX Comp RX Comp */
/* 31 - 16   15     14      13     12           11             10            */
/* Reserved Deauth PATxSus RadDet HwTxAbReqEnd HwTxAbReqStart HIPQRxComp     */
/*                                                                           */
/* 0 - Interrupt not raised                                                  */
/* 1 - Interrupt is raised                                                   */

INLINE BOOL_T get_machw_rx_comp_int(void)
{
    //chenq mod
    //UWORD32 int_mask = rMAC_INT_MASK;
    //UWORD32 int_stat = rMAC_INT_STAT;
    UWORD32 int_mask = host_read_trout_reg((UWORD32)rCOMM_INT_MASK) >> 2;
    UWORD32 int_stat = host_read_trout_reg((UWORD32)rCOMM_INT_STAT) >> 2;

	update_trout_int_mask(int_mask);
	
    int_stat &= ~int_mask;

    if((int_stat & REGBIT0) == 0)
        return BFALSE;

    return BTRUE;
}

INLINE BOOL_T get_machw_tx_comp_int(void)
{
    //chenq mod
    //UWORD32 int_mask = rMAC_INT_MASK;
    //UWORD32 int_stat = rMAC_INT_STAT;
    UWORD32 int_mask = host_read_trout_reg( (UWORD32)rCOMM_INT_MASK ) >> 2;
    UWORD32 int_stat = host_read_trout_reg( (UWORD32)rCOMM_INT_STAT ) >> 2;

	update_trout_int_mask(int_mask);
	
    int_stat &= ~int_mask;

    if((int_stat & REGBIT1) == 0)
        return BFALSE;

    return BTRUE;
}

INLINE BOOL_T get_machw_tbtt_int(void)
{
    //chenq mod
    //UWORD32 int_mask = rMAC_INT_MASK;
    //UWORD32 int_stat = rMAC_INT_STAT;
    UWORD32 int_mask = host_read_trout_reg( (UWORD32)rCOMM_INT_MASK ) >> 2;
    UWORD32 int_stat = host_read_trout_reg( (UWORD32)rCOMM_INT_STAT ) >> 2;

	update_trout_int_mask(int_mask);
	
    int_stat &= ~int_mask;

    if((int_stat & REGBIT2) == 0)
        return BFALSE;

    return BTRUE;
}

INLINE BOOL_T get_machw_hcca_txop_int(void)
{
    //chenq mod
    //UWORD32 int_mask = rMAC_INT_MASK;
    //UWORD32 int_stat = rMAC_INT_STAT;
    UWORD32 int_mask = host_read_trout_reg( (UWORD32)rCOMM_INT_MASK ) >> 2;
    UWORD32 int_stat = host_read_trout_reg( (UWORD32)rCOMM_INT_STAT ) >> 2;

	update_trout_int_mask(int_mask);
	
    int_stat &= ~int_mask;

    if((int_stat & REGBIT3) == 0)
        return BFALSE;

    return BTRUE;
}

INLINE BOOL_T get_machw_atim_int(void)
{
    //chenq mod
    //UWORD32 int_mask = rMAC_INT_MASK;
    //UWORD32 int_stat = rMAC_INT_STAT;
    UWORD32 int_mask = host_read_trout_reg( (UWORD32)rCOMM_INT_MASK ) >> 2;
    UWORD32 int_stat = host_read_trout_reg( (UWORD32)rCOMM_INT_STAT ) >> 2;

	update_trout_int_mask(int_mask);
	
    int_stat &= ~int_mask;

    if((int_stat & REGBIT4) == 0)
        return BFALSE;

    return BTRUE;
}

INLINE BOOL_T get_machw_wakeup_int(void)
{
    //chenq mod
    //UWORD32 int_mask = rMAC_INT_MASK;
    //UWORD32 int_stat = rMAC_INT_STAT;
    UWORD32 int_mask = host_read_trout_reg( (UWORD32)rCOMM_INT_MASK ) >> 2;
    UWORD32 int_stat = host_read_trout_reg( (UWORD32)rCOMM_INT_STAT ) >> 2;

	update_trout_int_mask(int_mask);
	
    int_stat &= ~int_mask;

    if((int_stat & REGBIT5) == 0)
        return BFALSE;

    return BTRUE;
}

INLINE BOOL_T get_machw_error_int(void)
{
    //chenq mod
    //UWORD32 int_mask = rMAC_INT_MASK;
    //UWORD32 int_stat = rMAC_INT_STAT;
    UWORD32 int_mask = host_read_trout_reg( (UWORD32)rCOMM_INT_MASK ) >> 2;
    UWORD32 int_stat = host_read_trout_reg( (UWORD32)rCOMM_INT_STAT ) >> 2;

	update_trout_int_mask(int_mask);
	
    int_stat &= ~int_mask;

    if((int_stat & REGBIT6) == 0)
        return BFALSE;

    return BTRUE;
}

INLINE BOOL_T get_machw_cap_end_int(void)
{
    //chenq mod
    //UWORD32 int_mask = rMAC_INT_MASK;
    //UWORD32 int_stat = rMAC_INT_STAT;
    UWORD32 int_mask = host_read_trout_reg( (UWORD32)rCOMM_INT_MASK ) >> 2;
    UWORD32 int_stat = host_read_trout_reg( (UWORD32)rCOMM_INT_STAT ) >> 2;

	update_trout_int_mask(int_mask);
	
    int_stat &= ~int_mask;

    if((int_stat & REGBIT7) == 0)
        return BFALSE;

    return BTRUE;
}

INLINE BOOL_T get_machw_cfend_int(void)
{
    //chenq mod
    //UWORD32 int_mask = rMAC_INT_MASK;
    //UWORD32 int_stat = rMAC_INT_STAT;
    UWORD32 int_mask = host_read_trout_reg( (UWORD32)rCOMM_INT_MASK ) >> 2;
    UWORD32 int_stat = host_read_trout_reg( (UWORD32)rCOMM_INT_STAT ) >> 2;

	update_trout_int_mask(int_mask);
	
    int_stat &= ~int_mask;

    if((int_stat & REGBIT8) == 0)
        return BFALSE;

    return BTRUE;
}

INLINE BOOL_T get_machw_hprx_comp_int(void)
{
    //chenq mod
    //UWORD32 int_mask = rMAC_INT_MASK;
    //UWORD32 int_stat = rMAC_INT_STAT;
    UWORD32 int_mask = host_read_trout_reg( (UWORD32)rCOMM_INT_MASK ) >> 2;
    UWORD32 int_stat = host_read_trout_reg( (UWORD32)rCOMM_INT_STAT ) >> 2;

	update_trout_int_mask(int_mask);
	
    int_stat &= ~int_mask;

    if((int_stat & REGBIT10) == 0)
        return BFALSE;

    return BTRUE;
}

INLINE BOOL_T get_machw_radar_det_int(void)
{
    //chenq mod
    //UWORD32 int_mask = rMAC_INT_MASK;
    //UWORD32 int_stat = rMAC_INT_STAT;
    UWORD32 int_mask = host_read_trout_reg( (UWORD32)rCOMM_INT_MASK ) >> 2;
    UWORD32 int_stat = host_read_trout_reg( (UWORD32)rCOMM_INT_STAT ) >> 2;

	update_trout_int_mask(int_mask);
	
    int_stat &= ~int_mask;

    if((int_stat & REGBIT13) == 0)
        return BFALSE;

    return BTRUE;
}

INLINE BOOL_T get_machw_txsus_int(void)
{
    //chenq mod
    //UWORD32 int_mask = rMAC_INT_MASK;
    //UWORD32 int_stat = rMAC_INT_STAT;
    UWORD32 int_mask = host_read_trout_reg( (UWORD32)rCOMM_INT_MASK ) >> 2;
    UWORD32 int_stat = host_read_trout_reg( (UWORD32)rCOMM_INT_STAT ) >> 2;

	update_trout_int_mask(int_mask);
	
    int_stat &= ~int_mask;

    if((int_stat & REGBIT14) == 0)
        return BFALSE;

    return BTRUE;
}

INLINE BOOL_T get_machw_deauth_int(void)
{
    //chenq mod
    //UWORD32 int_mask = rMAC_INT_MASK;
    //UWORD32 int_stat = rMAC_INT_STAT;
    UWORD32 int_mask = host_read_trout_reg( (UWORD32)rCOMM_INT_MASK ) >> 2;
    UWORD32 int_stat = host_read_trout_reg( (UWORD32)rCOMM_INT_STAT ) >> 2;

	update_trout_int_mask(int_mask);
	
    int_stat &= ~int_mask;

    if((int_stat & REGBIT15) == 0)
        return BFALSE;

    return BTRUE;
}

// arm7 to host interrupt
INLINE void reset_machw_arm2host_int(void)
{
    //chenq mod
    //rMAC_INT_STAT = REGBIT29;
    //host_write_trout_reg(REGBIT29, (UWORD32)rMAC_INT_STAT);
    host_write_trout_reg((REGBIT29 << 2), (UWORD32)rCOMM_INT_CLEAR);
}

INLINE void reset_machw_rx_comp_int(void)
{
    //chenq mod
    //rMAC_INT_STAT = REGBIT0;
    //host_write_trout_reg(REGBIT0, (UWORD32)rMAC_INT_STAT);
    host_write_trout_reg((REGBIT0 << 2), (UWORD32)rCOMM_INT_CLEAR);
}

INLINE void reset_machw_tx_comp_int(void)
{
    //chenq mod
    //rMAC_INT_STAT = REGBIT1;
    //host_write_trout_reg(REGBIT1, (UWORD32)rMAC_INT_STAT);
    host_write_trout_reg((REGBIT1 << 2), (UWORD32)rCOMM_INT_CLEAR);
}

INLINE void reset_machw_tbtt_int(void)
{
    //chenq mod
    //rMAC_INT_STAT = REGBIT2;
    host_write_trout_reg(REGBIT2, (UWORD32)rMAC_INT_STAT);	//this is needed, to fix hw bug, 2013.02.28
    host_write_trout_reg((REGBIT2 << 2), (UWORD32)rCOMM_INT_CLEAR);
}
/*zhq add for powersave*/
INLINE void root_reset_machw_tbtt_int(void)
{
    //chenq mod
    //rMAC_INT_STAT = REGBIT2;
    root_host_write_trout_reg(REGBIT2, (UWORD32)rMAC_INT_STAT);	//this is needed, to fix hw bug, 2013.02.28
    root_host_write_trout_reg((REGBIT2 << 2), (UWORD32)rCOMM_INT_CLEAR);
}

INLINE void reset_machw_hcca_txop_int(void)
{
    //chenq mod
    //rMAC_INT_STAT = REGBIT3;
    //host_write_trout_reg(REGBIT3, (UWORD32)rMAC_INT_STAT);
    host_write_trout_reg((REGBIT3 << 2), (UWORD32)rCOMM_INT_CLEAR);
}

INLINE void reset_machw_atim_int(void)
{
    //chenq mod
    //rMAC_INT_STAT = REGBIT4;
    //host_write_trout_reg(REGBIT4, (UWORD32)rMAC_INT_STAT);
    host_write_trout_reg((REGBIT4 << 2), (UWORD32)rCOMM_INT_CLEAR);
}

INLINE void reset_machw_wakeup_int(void)
{
    //chenq mod
    //rMAC_INT_STAT = REGBIT5;
    //host_write_trout_reg(REGBIT5, (UWORD32)rMAC_INT_STAT);
    host_write_trout_reg((REGBIT5 << 2), (UWORD32)rCOMM_INT_CLEAR);
}

INLINE void reset_machw_error_int(void)
{
    //chenq mod
    //rMAC_INT_STAT = REGBIT6;
    //host_write_trout_reg(REGBIT6, (UWORD32)rMAC_INT_STAT);
    host_write_trout_reg((REGBIT6 << 2), (UWORD32)rCOMM_INT_CLEAR);
}

INLINE void reset_machw_cap_end_int(void)
{
    //chenq mod
    //rMAC_INT_STAT = REGBIT7;
    //host_write_trout_reg(REGBIT7, (UWORD32)rMAC_INT_STAT);
    host_write_trout_reg((REGBIT7 << 2), (UWORD32)rCOMM_INT_CLEAR);
}

INLINE void reset_machw_cfend_int(void)
{
    //chenq mod
    //rMAC_INT_STAT = REGBIT8;
    //host_write_trout_reg(REGBIT8, (UWORD32)rMAC_INT_STAT);
    host_write_trout_reg((REGBIT8 << 2), (UWORD32)rCOMM_INT_CLEAR);
}

INLINE void reset_machw_hprx_comp_int(void)
{
    //chenq mod
    //rMAC_INT_STAT = REGBIT10;
    //host_write_trout_reg(REGBIT10, (UWORD32)rMAC_INT_STAT);
    host_write_trout_reg((REGBIT10 << 2), (UWORD32)rCOMM_INT_CLEAR);
}

INLINE void reset_tx_abort_start_int(void)
{
    //chenq mod
    //rMAC_INT_STAT = REGBIT11;
    //host_write_trout_reg(REGBIT11, (UWORD32)rMAC_INT_STAT);
    host_write_trout_reg((REGBIT11 << 2), (UWORD32)rCOMM_INT_CLEAR);
}

INLINE void reset_tx_abort_end_int(void)
{
    //chenq mod
    //rMAC_INT_STAT = REGBIT12;
    //host_write_trout_reg(REGBIT12, (UWORD32)rMAC_INT_STAT);
    host_write_trout_reg((REGBIT12 << 2), (UWORD32)rCOMM_INT_CLEAR);
}

INLINE void reset_machw_radar_det_int(void)
{
    //chenq mod
    //rMAC_INT_STAT = REGBIT13;
    //host_write_trout_reg(REGBIT13, (UWORD32)rMAC_INT_STAT);
    host_write_trout_reg((REGBIT13 << 2), (UWORD32)rCOMM_INT_CLEAR);
}

INLINE void reset_machw_txsus_int(void)
{
    //chenq mod
    //rMAC_INT_STAT = REGBIT14;
    //host_write_trout_reg(REGBIT14, (UWORD32)rMAC_INT_STAT);
    host_write_trout_reg((REGBIT14 << 2), (UWORD32)rCOMM_INT_CLEAR);
}

INLINE void reset_machw_deauth_int(void)
{
    //chenq mod
    //rMAC_INT_STAT = REGBIT15;
    //host_write_trout_reg(REGBIT15, (UWORD32)rMAC_INT_STAT);
    host_write_trout_reg((REGBIT15 << 2), (UWORD32)rCOMM_INT_CLEAR);
}

INLINE void reset_ab_period_start_int(void)
{
    //chenq mod
    //rMAC_INT_STAT = REGBIT16;
    //host_write_trout_reg(REGBIT16, (UWORD32)rMAC_INT_STAT);
    host_write_trout_reg((REGBIT16 << 2), (UWORD32)rCOMM_INT_CLEAR);
}

INLINE void reset_ab_period_end_int(void)
{
    //chenq mod
    //rMAC_INT_STAT = REGBIT17;
    //host_write_trout_reg(REGBIT17, (UWORD32)rMAC_INT_STAT);
    host_write_trout_reg((REGBIT17 << 2), (UWORD32)rCOMM_INT_CLEAR);
}

INLINE void reset_machw_ints(void)
{
    //chenq mod
    //rMAC_INT_STAT = convert_to_le((UWORD32)0xFFFF);
    //host_write_trout_reg( convert_to_le((UWORD32)0xFFFF),   //Hugh
    //host_write_trout_reg((UWORD32)0xFFFF, (UWORD32)rMAC_INT_STAT);
    host_write_trout_reg( convert_to_le(((UWORD32)0xFFFF <<2)), 
                          (UWORD32)rCOMM_INT_CLEAR);
}

INLINE UWORD8 get_machw_error_code(void)
{
    //chenq mod
    //return (convert_to_le(rMAC_ERROR_CODE) & 0xFF);
    return (convert_to_le(host_read_trout_reg( (UWORD32)rMAC_ERROR_CODE)) & 0xFF);
}

/* The interrupt mask register has various subfields described below.        */
/*                                                                           */
/* 9           8     7      6     5      4    3         2    1       0       */
/* RFVCOUnlock CFEnd CAPEnd Error WakeUp ATIM HCCA TXOP TBTT TX Comp RX Comp */
/* 31 - 16   15     14      13     12           11             10            */
/* Reserved Deauth PATxSus RadDet HwTxAbReqEnd HwTxAbReqStart HIPQRxComp     */
/*                                                                           */
/* 0 - Interrupt not masked                                                  */
/* 1 - Interrupt is masked                                                   */

INLINE void unmask_machw_rx_comp_int(void)
{
    //chenq mod
    //rMAC_INT_MASK &= ~REGBIT0;
    //host_write_trout_reg(host_read_trout_reg((UWORD32)rMAC_INT_MASK) & (~(REGBIT0)) ,
    //                    (UWORD32)rMAC_INT_MASK);
#if 0
    host_write_trout_reg(
        host_read_trout_reg((UWORD32)rCOMM_INT_MASK ) & (~(REGBIT0 << 2)),
        (UWORD32)rCOMM_INT_MASK);
#else
	UWORD32 int_mask;
	int_mask = host_read_trout_reg((UWORD32)rCOMM_INT_MASK) & (~(REGBIT0 << 2));
	host_write_trout_reg(int_mask, (UWORD32)rCOMM_INT_MASK);
	update_trout_int_mask(int_mask);
#endif
}

INLINE void mask_machw_rx_comp_int(void)
{
    //chenq mod
    //rMAC_INT_MASK |= REGBIT0;
    //host_write_trout_reg(
    //    host_read_trout_reg( (UWORD32)rMAC_INT_MASK ) | (REGBIT0) ,
    //    (UWORD32)rMAC_INT_MASK);
#if 0
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rCOMM_INT_MASK ) | (REGBIT0 << 2) ,
        (UWORD32)rCOMM_INT_MASK );
#else
	UWORD32 int_mask;
	int_mask = host_read_trout_reg((UWORD32)rCOMM_INT_MASK) | (REGBIT0 << 2);
	host_write_trout_reg(int_mask, (UWORD32)rCOMM_INT_MASK);
	update_trout_int_mask(int_mask);
#endif
}

INLINE void unmask_machw_tx_comp_int(void)
{
    //chenq mod
    //rMAC_INT_MASK &= ~REGBIT1;
    //host_write_trout_reg(
    //    host_read_trout_reg( (UWORD32)rMAC_INT_MASK ) & ~REGBIT1,
    //    (UWORD32)rMAC_INT_MASK);
#if 0
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rCOMM_INT_MASK ) & (~(REGBIT1 << 2)) ,
        (UWORD32)rCOMM_INT_MASK );
#else
	UWORD32 int_mask;
	int_mask = host_read_trout_reg((UWORD32)rCOMM_INT_MASK) & (~(REGBIT1 << 2));
	host_write_trout_reg(int_mask, (UWORD32)rCOMM_INT_MASK);
	update_trout_int_mask(int_mask);
#endif
}

INLINE void mask_machw_tx_comp_int(void)
{
    //chenq mod
    //rMAC_INT_MASK |= REGBIT1;
    //host_write_trout_reg(
    //    host_read_trout_reg( (UWORD32)rMAC_INT_MASK ) | (REGBIT1) ,
    //    (UWORD32)rMAC_INT_MASK);
#if 0
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rCOMM_INT_MASK ) | (REGBIT1 << 2) ,
        (UWORD32)rCOMM_INT_MASK );
#else
	UWORD32 int_mask;
	int_mask = host_read_trout_reg((UWORD32)rCOMM_INT_MASK) | (REGBIT1 << 2);
	host_write_trout_reg(int_mask, (UWORD32)rCOMM_INT_MASK);
	update_trout_int_mask(int_mask);
#endif
}

INLINE void unmask_machw_tbtt_int(void)
{
    //chenq mod
    //rMAC_INT_MASK &= ~REGBIT2;
    //host_write_trout_reg(
    //    host_read_trout_reg( (UWORD32)rMAC_INT_MASK ) & (~(REGBIT2)) ,
    //    (UWORD32)rMAC_INT_MASK);
#if 0
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rCOMM_INT_MASK ) & (~(REGBIT2 << 2)) ,
        (UWORD32)rCOMM_INT_MASK );
#else
	UWORD32 int_mask;
	host_write_trout_reg(
        host_read_trout_reg((UWORD32)rMAC_INT_MASK) & (~(REGBIT2)),
        (UWORD32)rMAC_INT_MASK);	//opened by chengwg to fix hw bug.2013-03-02
    
	int_mask = host_read_trout_reg((UWORD32)rCOMM_INT_MASK) & (~(REGBIT2 << 2));
	host_write_trout_reg(int_mask, (UWORD32)rCOMM_INT_MASK);
	update_trout_int_mask(int_mask);
#endif
}
/*zhq add for powersave*/
INLINE void root_unmask_machw_tbtt_int(void)
{
    //chenq mod
    //rMAC_INT_MASK &= ~REGBIT2;
    //host_write_trout_reg(
    //    host_read_trout_reg( (UWORD32)rMAC_INT_MASK ) & (~(REGBIT2)) ,
    //    (UWORD32)rMAC_INT_MASK);
#if 0
    host_write_trout_reg(
	host_read_trout_reg( (UWORD32)rCOMM_INT_MASK ) & (~(REGBIT2 << 2)) ,
	(UWORD32)rCOMM_INT_MASK );
#else
	UWORD32 int_mask;
	root_host_write_trout_reg(
	root_host_read_trout_reg((UWORD32)rMAC_INT_MASK) & (~(REGBIT2)),
	(UWORD32)rMAC_INT_MASK);	//opened by chengwg to fix hw bug.2013-03-02
    
	int_mask = root_host_read_trout_reg((UWORD32)rCOMM_INT_MASK) & (~(REGBIT2 << 2));
	root_host_write_trout_reg(int_mask, (UWORD32)rCOMM_INT_MASK);
	update_trout_int_mask(int_mask);
#endif
}

INLINE void mask_machw_tbtt_int(void)
{
    //chenq mod
    //rMAC_INT_MASK |= REGBIT2;
    //host_write_trout_reg(
    //    host_read_trout_reg( (UWORD32)rMAC_INT_MASK ) | (REGBIT2) ,
    //    (UWORD32)rMAC_INT_MASK);
#if 0
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rCOMM_INT_MASK ) | (REGBIT2 << 2) ,
        (UWORD32)rCOMM_INT_MASK );
#else
	UWORD32 int_mask;
	int_mask = host_read_trout_reg((UWORD32)rCOMM_INT_MASK) | (REGBIT2 << 2);
	host_write_trout_reg(int_mask, (UWORD32)rCOMM_INT_MASK);
	update_trout_int_mask(int_mask);
#endif
}
/*zhq add for powersave*/
INLINE void root_mask_machw_tbtt_int(void)
{
    //chenq mod
    //rMAC_INT_MASK |= REGBIT2;
    //host_write_trout_reg(
    //    host_read_trout_reg( (UWORD32)rMAC_INT_MASK ) | (REGBIT2) ,
    //    (UWORD32)rMAC_INT_MASK);
#if 0
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rCOMM_INT_MASK ) | (REGBIT2 << 2) ,
        (UWORD32)rCOMM_INT_MASK );
#else
	UWORD32 int_mask;
	int_mask = root_host_read_trout_reg((UWORD32)rCOMM_INT_MASK) | (REGBIT2 << 2);
	root_host_write_trout_reg(int_mask, (UWORD32)rCOMM_INT_MASK);
	update_trout_int_mask(int_mask);
#endif
}

INLINE BOOL_T is_machw_tbtt_int_masked(void)
{
    //chenq mod
    //if(rMAC_INT_MASK & REGBIT2)
    if(host_read_trout_reg((UWORD32)rCOMM_INT_MASK) & (REGBIT2 << 2))
        return BTRUE;

    return BFALSE;
}

INLINE void unmask_machw_error_int(void)
{
    //chenq mod
    //rMAC_INT_MASK &= ~REGBIT6;
    //host_write_trout_reg(
    //    host_read_trout_reg( (UWORD32)rMAC_INT_MASK ) & (~(REGBIT6)) ,
    //    (UWORD32)rMAC_INT_MASK);
#if 0
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rCOMM_INT_MASK ) & (~(REGBIT6 << 2)) ,
        (UWORD32)rCOMM_INT_MASK );
#else
	UWORD32 int_mask;
	/* first prepare common wifi error mask by zhao */
	host_write_trout_reg(MAC_ERROR_MASK_INIT_VALUE, (UWORD32)rCOMM_WIFI_ERR_MSK);

	/* then enable common error msk in INT_MASK(BIT21)  by zhao */
#ifndef ERROR_INT_ENABLE
	int_mask = host_read_trout_reg((UWORD32)rCOMM_INT_MASK);
#else
	int_mask = host_read_trout_reg((UWORD32)rCOMM_INT_MASK) & (~(REGBIT6 << 2)) & (~(REGBIT21 << 2));
#endif
	host_write_trout_reg(int_mask, (UWORD32)rCOMM_INT_MASK);
	update_trout_int_mask(int_mask);
#endif
}

INLINE void mask_machw_error_int(void)
{
    //chenq mod
    //rMAC_INT_MASK |= REGBIT6;
    //host_write_trout_reg(
    //    host_read_trout_reg((UWORD32)rMAC_INT_MASK ) | (REGBIT6) ,
    //    (UWORD32)rMAC_INT_MASK );
#if 0
    host_write_trout_reg(
        host_read_trout_reg((UWORD32)rCOMM_INT_MASK ) | (REGBIT6 << 2) ,
        (UWORD32)rCOMM_INT_MASK );
#else
	UWORD32 int_mask;
	int_mask = host_read_trout_reg((UWORD32)rCOMM_INT_MASK) | (REGBIT6 << 2) | (REGBIT21 << 2);
	host_write_trout_reg(int_mask, (UWORD32)rCOMM_INT_MASK);
	update_trout_int_mask(int_mask);
#endif
}

INLINE void unmask_machw_cap_end_int(void)
{
    //chenq mod
    //rMAC_INT_MASK &= ~REGBIT7;
    //host_write_trout_reg(
    //    host_read_trout_reg( (UWORD32)rMAC_INT_MASK ) & (~(REGBIT7)) ,
    //    (UWORD32)rMAC_INT_MASK );
#if 0
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rCOMM_INT_MASK ) & (~(REGBIT7 << 2)) ,
        (UWORD32)rCOMM_INT_MASK );
#else
	UWORD32 int_mask;
	int_mask = host_read_trout_reg((UWORD32)rCOMM_INT_MASK) & (~(REGBIT7 << 2));
	host_write_trout_reg(int_mask, (UWORD32)rCOMM_INT_MASK);
	update_trout_int_mask(int_mask);
#endif
}

INLINE void unmask_machw_cfend_int(void)
{
    //chenq mod
    //rMAC_INT_MASK &= ~REGBIT8;
    //host_write_trout_reg(
    //    host_read_trout_reg( (UWORD32)rMAC_INT_MASK ) & (~(REGBIT8)) ,
    //    (UWORD32)rMAC_INT_MASK );
#if 0
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rCOMM_INT_MASK ) & (~(REGBIT8 << 2)) ,
        (UWORD32)rCOMM_INT_MASK );
#else
	UWORD32 int_mask;
	int_mask = host_read_trout_reg((UWORD32)rCOMM_INT_MASK) & (~(REGBIT8 << 2));
	host_write_trout_reg(int_mask, (UWORD32)rCOMM_INT_MASK);
	update_trout_int_mask(int_mask);
#endif
}

INLINE void unmask_machw_hprx_comp_int(void)
{
    //chenq mod
    //rMAC_INT_MASK &= ~REGBIT10;
    //host_write_trout_reg(
    //    host_read_trout_reg( (UWORD32)rMAC_INT_MASK ) & (~(REGBIT10)) ,
    //    (UWORD32)rMAC_INT_MASK );
#if 0
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rCOMM_INT_MASK ) & (~(REGBIT10 << 2)) ,
        (UWORD32)rCOMM_INT_MASK );
#else
	UWORD32 int_mask;
	int_mask = host_read_trout_reg((UWORD32)rCOMM_INT_MASK) & (~(REGBIT10 << 2));
	host_write_trout_reg(int_mask, (UWORD32)rCOMM_INT_MASK);
	update_trout_int_mask(int_mask);
#endif
}

INLINE void mask_machw_hprx_comp_int(void)
{
    //chenq mod
    //rMAC_INT_MASK |= REGBIT10;
    //host_write_trout_reg(
    //    host_read_trout_reg((UWORD32)rMAC_INT_MASK ) | (REGBIT10) ,
    //    (UWORD32)rMAC_INT_MASK );
#if 0
    host_write_trout_reg(
        host_read_trout_reg((UWORD32)rCOMM_INT_MASK ) | (REGBIT10 << 2) ,
        (UWORD32)rCOMM_INT_MASK );
#else
	UWORD32 int_mask;
	int_mask = host_read_trout_reg((UWORD32)rCOMM_INT_MASK) | (REGBIT10 << 2);
	host_write_trout_reg(int_mask, (UWORD32)rCOMM_INT_MASK);
	update_trout_int_mask(int_mask);
#endif
}

INLINE void unmask_machw_txab_start_int(void)
{
    //chenq mod
    //rMAC_INT_MASK &= ~REGBIT11;
    //host_write_trout_reg(
    //    host_read_trout_reg( (UWORD32)rMAC_INT_MASK ) & (~(REGBIT11)) ,
    //    (UWORD32)rMAC_INT_MASK );
#if 0
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rCOMM_INT_MASK ) & (~(REGBIT11 << 2)) ,
        (UWORD32)rCOMM_INT_MASK );
#else
	UWORD32 int_mask;
	int_mask = host_read_trout_reg((UWORD32)rCOMM_INT_MASK) & (~(REGBIT11 << 2));
	host_write_trout_reg(int_mask, (UWORD32)rCOMM_INT_MASK);
	update_trout_int_mask(int_mask);
#endif
}

INLINE void mask_machw_txab_start_int(void)
{
    //chenq mod
    //rMAC_INT_MASK |= REGBIT11;
    //host_write_trout_reg(
    //    host_read_trout_reg( (UWORD32)rMAC_INT_MASK ) | (REGBIT11) ,
    //    (UWORD32)rMAC_INT_MASK );
#if 0
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rCOMM_INT_MASK ) | (REGBIT11 << 2) ,
        (UWORD32)rCOMM_INT_MASK );
#else
	UWORD32 int_mask;
	int_mask = host_read_trout_reg((UWORD32)rCOMM_INT_MASK) | (REGBIT11 << 2);
	host_write_trout_reg(int_mask, (UWORD32)rCOMM_INT_MASK);
	update_trout_int_mask(int_mask);
#endif
}

INLINE void unmask_machw_txab_end_int(void)
{
    //chenq mod
    //rMAC_INT_MASK &= ~REGBIT12;
    //host_write_trout_reg(
    //    host_read_trout_reg( (UWORD32)rMAC_INT_MASK ) & (~(REGBIT12)) ,
    //    (UWORD32)rMAC_INT_MASK );
#if 0
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rCOMM_INT_MASK ) & (~(REGBIT12 << 2)) ,
        (UWORD32)rCOMM_INT_MASK );
#else
	UWORD32 int_mask;
	int_mask = host_read_trout_reg((UWORD32)rCOMM_INT_MASK) & (~(REGBIT12 << 2));
	host_write_trout_reg(int_mask, (UWORD32)rCOMM_INT_MASK);
	update_trout_int_mask(int_mask);
#endif
}

INLINE void mask_machw_txab_end_int(void)
{
    //chenq mod
    //rMAC_INT_MASK |= REGBIT12;
    //host_write_trout_reg(
    //    host_read_trout_reg( (UWORD32)rMAC_INT_MASK ) | (REGBIT12) ,
    //    (UWORD32)rMAC_INT_MASK );
#if 0
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rCOMM_INT_MASK ) | (REGBIT12 << 2) ,
        (UWORD32)rCOMM_INT_MASK );
#else
	UWORD32 int_mask;
	int_mask = host_read_trout_reg((UWORD32)rCOMM_INT_MASK) | (REGBIT12 << 2);
	host_write_trout_reg(int_mask, (UWORD32)rCOMM_INT_MASK);
	update_trout_int_mask(int_mask);
#endif
}

INLINE void unmask_machw_radar_det_int(void)
{
    //chenq mod
    //rMAC_INT_MASK &= ~REGBIT13;
    //host_write_trout_reg(
    //    host_read_trout_reg( (UWORD32)rMAC_INT_MASK ) & (~(REGBIT13)) ,
    //    (UWORD32)rMAC_INT_MASK );
#if 0
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rCOMM_INT_MASK ) & (~(REGBIT13 << 2)) ,
        (UWORD32)rCOMM_INT_MASK );
#else
	UWORD32 int_mask;
	int_mask = host_read_trout_reg((UWORD32)rCOMM_INT_MASK) & (~(REGBIT13 << 2));
	host_write_trout_reg(int_mask, (UWORD32)rCOMM_INT_MASK);
	update_trout_int_mask(int_mask);
#endif
}

INLINE void mask_machw_radar_det_int(void)
{
    //chenq mod
    //rMAC_INT_MASK |= REGBIT13;
    //host_write_trout_reg(
    //    host_read_trout_reg( (UWORD32)rMAC_INT_MASK ) | (REGBIT13) ,
    //    (UWORD32)rMAC_INT_MASK );
#if 0
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rCOMM_INT_MASK ) | (REGBIT13 << 2) ,
        (UWORD32)rCOMM_INT_MASK );
#else
	UWORD32 int_mask;
	int_mask = host_read_trout_reg((UWORD32)rCOMM_INT_MASK) | (REGBIT13 << 2);
	host_write_trout_reg(int_mask, (UWORD32)rCOMM_INT_MASK);
	update_trout_int_mask(int_mask);
#endif
}

INLINE void unmask_machw_txsus_int(void)
{
    //chenq mod
    //rMAC_INT_MASK &= ~REGBIT14;
    //host_write_trout_reg(
    //    host_read_trout_reg( (UWORD32)rMAC_INT_MASK ) & (~(REGBIT14)) ,
    //    (UWORD32)rMAC_INT_MASK );
#if 0
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rCOMM_INT_MASK ) & (~(REGBIT14 << 2)) ,
        (UWORD32)rCOMM_INT_MASK );
#else
	UWORD32 int_mask;
	int_mask = host_read_trout_reg((UWORD32)rCOMM_INT_MASK) & (~(REGBIT14 << 2));
	host_write_trout_reg(int_mask, (UWORD32)rCOMM_INT_MASK);
	update_trout_int_mask(int_mask);
#endif
}

INLINE void unmask_machw_deauth_int(void)
{
    //chenq mod
    //rMAC_INT_MASK &= ~REGBIT15;
    //host_write_trout_reg(
    //    host_read_trout_reg( (UWORD32)rMAC_INT_MASK ) & (~(REGBIT15)) ,
    //    (UWORD32)rMAC_INT_MASK );
#if 0
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rCOMM_INT_MASK ) & (~(REGBIT15 << 2)) ,
        (UWORD32)rCOMM_INT_MASK );
#else
	UWORD32 int_mask;
	int_mask = host_read_trout_reg((UWORD32)rCOMM_INT_MASK) & (~(REGBIT15 << 2));
	host_write_trout_reg(int_mask, (UWORD32)rCOMM_INT_MASK);
	update_trout_int_mask(int_mask);
#endif
}

INLINE void mask_machw_deauth_int(void)
{
    //chenq mod
    //rMAC_INT_MASK |= REGBIT15;
    //host_write_trout_reg(
    //    host_read_trout_reg( (UWORD32)rMAC_INT_MASK ) | (REGBIT15) ,
    //    (UWORD32)rMAC_INT_MASK );
#if 0
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rCOMM_INT_MASK ) | (REGBIT15 << 2) ,
        (UWORD32)rCOMM_INT_MASK );
#else
	UWORD32 int_mask;
	int_mask = host_read_trout_reg((UWORD32)rCOMM_INT_MASK) | (REGBIT15 << 2);
	host_write_trout_reg(int_mask, (UWORD32)rCOMM_INT_MASK);
	update_trout_int_mask(int_mask);
#endif
}

INLINE void unmask_machw_arm2host_int(void)
{
    //chenq mod
    //rMAC_INT_MASK &= ~REGBIT29;
    //host_write_trout_reg(
    //    host_read_trout_reg( (UWORD32)rMAC_INT_MASK ) & (~(REGBIT29)) ,
    //    (UWORD32)rMAC_INT_MASK );
#if 0
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rCOMM_INT_MASK ) & (~(REGBIT29 << 2)) ,
        (UWORD32)rCOMM_INT_MASK );
#else
	UWORD32 int_mask;
	int_mask = host_read_trout_reg( (UWORD32)rCOMM_INT_MASK ) & (~(REGBIT29 << 2));
	host_write_trout_reg(int_mask, (UWORD32)rCOMM_INT_MASK);
	update_trout_int_mask(int_mask);
#endif
}

INLINE void mask_machw_arm2host_int(void)
{
    //chenq mod
    //rMAC_INT_MASK |= REGBIT29;
    //host_write_trout_reg(
    //    host_read_trout_reg( (UWORD32)rMAC_INT_MASK ) | (REGBIT29) ,
    //    (UWORD32)rMAC_INT_MASK );
#if 0
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rCOMM_INT_MASK ) | (REGBIT29 << 2) ,
        (UWORD32)rCOMM_INT_MASK );
#else
	UWORD32 int_mask;
	int_mask = host_read_trout_reg( (UWORD32)rCOMM_INT_MASK ) | (REGBIT29 << 2);
	host_write_trout_reg(int_mask, (UWORD32)rCOMM_INT_MASK);
	update_trout_int_mask(int_mask);
#endif
}

INLINE void reset_machw_error_int_code(UWORD32 error_code)
{
    //chenq mod
    //rMAC_ERROR_STAT |= convert_to_le(1 << error_code);
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_ERROR_STAT ) 
            | (convert_to_le(1 << error_code)) ,
        (UWORD32)rMAC_ERROR_STAT );
}

INLINE void mask_machw_error_int_code(UWORD32 error_code)
{
    //chenq mod
    //rMAC_ERROR_MASK |= convert_to_le(1 << error_code);
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_ERROR_MASK ) 
            | (convert_to_le(1 << error_code)) ,
        (UWORD32)rMAC_ERROR_MASK );
}

INLINE void unmask_machw_error_int_code(UWORD32 error_code)
{
    //chenq mod
    //rMAC_ERROR_MASK &= ~(convert_to_le(1 << error_code));
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_ERROR_MASK ) 
            & (~(convert_to_le(1 << error_code))) ,
        (UWORD32)rMAC_ERROR_MASK );
}

INLINE BOOL_T is_machw_error_int_mask(UWORD32 error_code)
{
    BOOL_T retval = BFALSE;

    //chenq mod
    //if(rMAC_ERROR_MASK & (convert_to_le(1 << error_code)))
    if(host_read_trout_reg( (UWORD32)rMAC_ERROR_MASK ) 
           & (convert_to_le(1 << error_code)))
        retval = BTRUE;

    return retval;
}

/* The Transmit Frame pointer contains the address of the descriptor of the  */
/* frame that was transmitted and for which current TX Complete interrupt    */
/* was generated.                                                            */
INLINE UWORD32 get_machw_tx_frame_pointer(void)
{
    //chenq mod
    //UWORD32 temp = convert_to_le(rMAC_TX_FRAME_POINTER);
    UWORD32 temp = convert_to_le( host_read_trout_reg( 
                                    (UWORD32)rMAC_TX_FRAME_POINTER) );

    return (phy_to_virt_addr(temp));
}

/* The Receive Frame pointer contains the address of the descriptor of the   */
/* frame that was transmitted and for which current RX Complete interrupt    */
/* was generated. If interrupt was raised due to lack of memory then this    */
/* address will have NULL pointer.                                           */
INLINE UWORD32 get_machw_rx_frame_pointer(UWORD8 q_num)
{
    UWORD32 temp = 0;

    if(q_num == HIGH_PRI_RXQ)
    {
        //chenq mod
        //temp = convert_to_le(rMAC_HIP_RX_FRAME_POINTER);
        temp = convert_to_le(host_read_trout_reg( 
                                (UWORD32)rMAC_HIP_RX_FRAME_POINTER));
    }
    else
    {
        //chenq mod
        //temp = convert_to_le(rMAC_RX_FRAME_POINTER);
        temp = convert_to_le(host_read_trout_reg( 
                                (UWORD32)rMAC_RX_FRAME_POINTER));
    }

    return (phy_to_virt_addr(temp));
}

/* The Tx Complete interrupt can be raised for multiple transmitted frames.  */
/* This function reads the register with the number of frames.               */
INLINE UWORD8 get_machw_num_tx_frames(void)
{
    //chenq mod
    //return convert_to_le(rMAC_TX_MPDU_COUNT);
    return convert_to_le(host_read_trout_reg( (UWORD32)rMAC_TX_MPDU_COUNT  ));
}

/* The Rx Complete interrupt can be raised for multiple received frames.     */
/* This function reads the register with the number of frames.               */
INLINE UWORD8 get_machw_num_rx_frames(UWORD8 q_num)
{
    if(q_num == HIGH_PRI_RXQ)
    {
        return 1; /* For High Pri Rx Q, number of frames will always be 1 */
    }

    //chenq mod
    //return convert_to_le(rMAC_RX_MPDU_COUNT);
    return convert_to_le(host_read_trout_reg( (UWORD32)rMAC_RX_MPDU_COUNT ));
}

/* This function clears all currently pending interrupts. */
INLINE void clear_machw_interrupts(void)
{
    UWORD8  count      = 0;
    UWORD32 dummy_dscr = 0, int_mask, v, cnt = 0;
    UWORD32 tmp;

	/* Disable TBTT interrupts */
	set_machw_tsf_disable();
        /* Mask all the MAC interrupts */
        //chenq mod
        //rMAC_INT_MASK = convert_to_le(0xFFFFFFFF);
        //host_write_trout_reg(convert_to_le(0xFFFFFFFF), (UWORD32)rMAC_INT_MASK);
        //tmp = host_read_trout_reg((UWORD32)rCOMM_INT_MASK);  
        tmp = 0xFFFFFFFF;  
        host_write_trout_reg(convert_to_le(tmp), (UWORD32)rCOMM_INT_MASK);
	//add by chengwg.
	int_mask = convert_to_le(host_read_trout_reg((UWORD32)rCOMM_INT_MASK));
	update_trout_int_mask(int_mask);
	 
	/* Clear Tx & Rx Complete interrupts */
	for(count = 0; count < 5; count++)
	{
		dummy_dscr += get_machw_tx_frame_pointer();
		dummy_dscr += get_machw_num_tx_frames();

		dummy_dscr += get_machw_rx_frame_pointer(NORMAL_PRI_RXQ);
		dummy_dscr += get_machw_num_rx_frames(NORMAL_PRI_RXQ);

		dummy_dscr += get_machw_rx_frame_pointer(HIGH_PRI_RXQ);
		dummy_dscr += get_machw_num_rx_frames(HIGH_PRI_RXQ);
	}

	/* take care the Tx suspend interrupt by zhao */
	v = convert_to_le(host_read_trout_reg( (UWORD32)rMAC_PA_CON));
	v = v & (~BIT12);
	host_write_trout_reg(convert_to_le(v), (UWORD32)rMAC_PA_CON);

	while((host_read_trout_reg( (UWORD32)rMAC_PA_STAT) & REGBIT6) != 0){
		if(++cnt > 1000)
			break;
	}
	if(cnt > 1000)
		printk("%s ..wait PA status BIT6 to 0 timeout!\n",__func__);
	/* Clear all outstanding interrupts */
	//host_write_trout_reg(convert_to_le(0xFFFFFFFF), 
	//                      (UWORD32)rMAC_INT_STAT);
	//chenq mod
	//rMAC_INT_STAT = convert_to_le(0xFFFFFFFF);
	host_write_trout_reg(convert_to_le(0xFFFFFFFF), (UWORD32)rMAC_INT_STAT);	//add by chengwg.
	host_write_trout_reg(convert_to_le(0xFFFFFFFF), (UWORD32)rCOMM_INT_CLEAR);
}
INLINE void root_clear_machw_interrupts(void)
{
	UWORD8  count      = 0;
	UWORD32 int_mask, v, cnt = 0;

	/* Disable TBTT interrupts */
	set_machw_tsf_disable();
	int_mask = 0xFFFFFFFF;
	root_host_write_trout_reg(convert_to_le(int_mask) , (UWORD32)rCOMM_INT_MASK);
	update_trout_int_mask(int_mask);

	/* Clear Tx & Rx Complete interrupts */
	for(count = 0; count < 5; count++)
	{
		root_host_read_trout_reg((UWORD32)rMAC_TX_FRAME_POINTER);
		root_host_read_trout_reg((UWORD32)rMAC_TX_MPDU_COUNT);

		root_host_read_trout_reg((UWORD32)rMAC_RX_FRAME_POINTER);
		root_host_read_trout_reg( (UWORD32)rMAC_RX_MPDU_COUNT);

		root_host_read_trout_reg((UWORD32)rMAC_HIP_RX_FRAME_POINTER);
	}

	/* take care the Tx suspend interrupt by zhao */
	v = convert_to_le(root_host_read_trout_reg( (UWORD32)rMAC_PA_CON));
	v = v & (~BIT12);
	root_host_write_trout_reg(convert_to_le(v), (UWORD32)rMAC_PA_CON);

	while((root_host_read_trout_reg( (UWORD32)rMAC_PA_STAT) & REGBIT6) != 0){
		if(++cnt > 1000)
			break;
	}
	if(cnt > 1000)
		printk("%s ..wait PA status BIT6 to 0 timeout!\n",__func__);

	root_host_write_trout_reg(convert_to_le(0xFFFFFFFF), (UWORD32)rMAC_INT_STAT);	//add by chengwg.
	root_host_write_trout_reg(convert_to_le(0xFFFFFFFF), (UWORD32)rCOMM_INT_CLEAR);
}
static inline void root_force_clear_interrupts(void)
{
	unsigned long v, cnt;


	v = convert_to_le(root_host_read_trout_reg((UWORD32)rCOMM_INT_STAT)) >> 2;
	v = v & 0xFFFFF;
	if(v & BIT14){
		v = convert_to_le(root_host_read_trout_reg( (UWORD32)rMAC_PA_CON));
		v = v & (~BIT12);
		root_host_write_trout_reg(convert_to_le(v), (UWORD32)rMAC_PA_CON);

		while((root_host_read_trout_reg( (UWORD32)rMAC_PA_STAT) & REGBIT6) != 0){
			if(++cnt > 1000)
				break;
		}
		if(cnt > 1000)
			printk("%s ..wait PA status BIT6 to 0 timeout!\n", __func__);
	root_host_write_trout_reg(convert_to_le(0xFFFFFFFF), (UWORD32)rMAC_INT_STAT);
	root_host_write_trout_reg(convert_to_le(0xFFFFFFFF), (UWORD32)rCOMM_INT_CLEAR);
	}
}





static inline void force_clear_interrupts(void)
{
	unsigned int count;
	unsigned long v, slot, cnt = 0;
	//struct trout_tx_shareram *tts;


	v = convert_to_le(host_read_trout_reg((UWORD32)rCOMM_INT_STAT)) >> 2;
	v = v & 0xFFFFF;
	if(v & BIT14 || v & BIT1){
		for(count = 0; count < 5; count++){
			get_machw_tx_frame_pointer();
			get_machw_num_tx_frames();
            /* do not clear RX interrupts, let rx_isr to handle it  by zhao */
/*
			get_machw_rx_frame_pointer(NORMAL_PRI_RXQ);
			get_machw_num_rx_frames(NORMAL_PRI_RXQ);

			get_machw_rx_frame_pointer(HIGH_PRI_RXQ);
			get_machw_num_rx_frames(HIGH_PRI_RXQ);
*/
		}
/*
		if(v & BIT0)
			rx_complete_isr(NORMAL_PRI_RXQ);
		
		if(v & BIT10)
			rx_complete_isr(HIGH_PRI_RXQ);
*/
		if(v & BIT14){
			v = convert_to_le(host_read_trout_reg( (UWORD32)rMAC_PA_CON));
			v = v & (~BIT12);
			host_write_trout_reg(convert_to_le(v), (UWORD32)rMAC_PA_CON);

			while((host_read_trout_reg( (UWORD32)rMAC_PA_STAT) & REGBIT6) != 0){
				if(++cnt > 1000)
					break;
			}
			if(cnt > 1000)
				printk("%s ..wait PA status BIT6 to 0 timeout!\n", __func__);
		}


		if(v & BIT1){
			for(slot = 0; slot < 2; slot++){
				if(!tx_shareram_slot_busy(slot))
					continue;
				mutex_lock(&g_q_handle.tx_handle.txq_lock);
				tx_pkt_process_new(slot, TX_ISR_CALL);
				mutex_unlock(&g_q_handle.tx_handle.txq_lock);
			}
		}
		host_write_trout_reg(convert_to_le(0xFFFFFFFF), (UWORD32)rMAC_INT_STAT);
		host_write_trout_reg(convert_to_le(0xFFFFFFFF), (UWORD32)rCOMM_INT_CLEAR);

	}
}

/*****************************************************************************/
/* PCF Registers                                                              */
/*****************************************************************************/

INLINE void set_machw_pcf_mode(UWORD8 mode)
{
    //chenq mod
    //rMAC_PCF_CON = convert_to_le((UWORD32)mode);
    host_write_trout_reg(  convert_to_le((UWORD32)mode), 
                           (UWORD32)rMAC_PCF_CON );
}

INLINE void set_machw_cfp_max_dur(UWORD16 val)
{
    //chenq mod
    //rMAC_CFP_MAX_DUR = convert_to_le((UWORD32)val);
    host_write_trout_reg(  convert_to_le((UWORD32)val), 
                           (UWORD32)rMAC_CFP_MAX_DUR );
}

INLINE void set_machw_cfp_period(UWORD8 val)
{
    //chenq mod
    //rMAC_CFP_INTERVAL = convert_to_le((UWORD32)val);
    host_write_trout_reg(  convert_to_le((UWORD32)val), 
                           (UWORD32)rMAC_CFP_INTERVAL );
}

INLINE void set_machw_cfp_param_set_byte_num(UWORD16 val)
{
    //chenq mod
    //rMAC_CFP_PARAM_SET_BYTE_NUM = convert_to_le((UWORD32)val);
    host_write_trout_reg(  convert_to_le((UWORD32)val), 
                           (UWORD32)rMAC_CFP_PARAM_SET_BYTE_NUM );
}

INLINE void set_machw_medium_occup_limit(UWORD16 val)
{
    //chenq mod
    //rMAC_MEDIUM_OCCUPANCY = convert_to_le((UWORD32)val);
    host_write_trout_reg(  convert_to_le((UWORD32)val), 
                           (UWORD32)rMAC_MEDIUM_OCCUPANCY );
}

INLINE UWORD8 get_machw_cfp_count(void)
{
    //chenq mod
    //return convert_to_le(rMAC_CFP_COUNT);
    return convert_to_le(host_read_trout_reg( 
                            (UWORD32)rMAC_CFP_COUNT  ));
}

INLINE UWORD16 get_machw_unused_cfp_dur(void)
{
    //chenq mod
    //return convert_to_le(rMAC_UNUSED_CFP_DUR);
    return convert_to_le(host_read_trout_reg( 
                            (UWORD32)rMAC_UNUSED_CFP_DUR  ));
}

/*****************************************************************************/
/* Power Management Registers                                                */
/*****************************************************************************/

/* This function sets the transmission rate for NULL frames */
INLINE void set_machw_null_frame_rate(UWORD16 rate)
{
    //chenq mod
    //rMAC_NULL_FRAME_RATE = convert_to_le(rate);
    host_write_trout_reg( convert_to_le(rate), (UWORD32)rMAC_NULL_FRAME_RATE );
}

/* This function sets the transmission rate for NULL frames */
INLINE void set_machw_null_frame_ptm(UWORD32 ptm)
{
    //chenq mod
    //rMAC_NULL_FRAME_PHY_TX_MODE = convert_to_le(ptm);
    host_write_trout_reg( convert_to_le(ptm), 
                            (UWORD32)rMAC_NULL_FRAME_PHY_TX_MODE );
}
/*zhq add for powersave*/
INLINE void root_set_machw_null_frame_ptm(UWORD32 ptm)
{
    //chenq mod
    //rMAC_NULL_FRAME_PHY_TX_MODE = convert_to_le(ptm);
    root_host_write_trout_reg( convert_to_le(ptm), 
                            (UWORD32)rMAC_NULL_FRAME_PHY_TX_MODE );
}

/* Power Management Registers */

INLINE void set_machw_ps_active_mode(void)
{
    //chenq mod
    //rMAC_PM_CON &= ~REGBIT0;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PM_CON ) & (~REGBIT0) ,
        (UWORD32)rMAC_PM_CON );
}
/*zhq add for powersave*/
INLINE void root_set_machw_ps_active_mode(void)
{
    //chenq mod
    //rMAC_PM_CON &= ~REGBIT0;
    root_host_write_trout_reg(
        root_host_read_trout_reg( (UWORD32)rMAC_PM_CON ) & (~REGBIT0) ,
        (UWORD32)rMAC_PM_CON );
}

INLINE void set_machw_ps_doze_mode(void)
{
    //chenq mod
    //rMAC_PM_CON |= REGBIT0;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PM_CON ) | (REGBIT0) ,
        (UWORD32)rMAC_PM_CON );
}

/*zhq add for powersave*/
INLINE void root_set_machw_ps_doze_mode(void)
{
    //chenq mod
    //rMAC_PM_CON |= REGBIT0;
    root_host_write_trout_reg(
        root_host_read_trout_reg( (UWORD32)rMAC_PM_CON ) | (REGBIT0) ,
        (UWORD32)rMAC_PM_CON );
}

INLINE void set_machw_ps_rdtim_bit(void)
{
    //chenq mod
    //rMAC_PM_CON |= REGBIT1;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PM_CON ) | (REGBIT1) ,
        (UWORD32)rMAC_PM_CON );
}
/*zhq add for powersave*/
INLINE void root_set_machw_ps_rdtim_bit(void)
{
    //chenq mod
    //rMAC_PM_CON |= REGBIT1;
    root_host_write_trout_reg(
        root_host_read_trout_reg( (UWORD32)rMAC_PM_CON ) | (REGBIT1) ,
        (UWORD32)rMAC_PM_CON );
}

INLINE void reset_machw_ps_rdtim_bit(void)
{
    //chenq mod
    //rMAC_PM_CON &= ~REGBIT1;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PM_CON ) & (~REGBIT1) ,
        (UWORD32)rMAC_PM_CON );
}
/*zhq add for powersave*/
INLINE void root_reset_machw_ps_rdtim_bit(void)
{
    //chenq mod
    //rMAC_PM_CON &= ~REGBIT1;
    root_host_write_trout_reg(
        root_host_read_trout_reg( (UWORD32)rMAC_PM_CON ) & (~REGBIT1) ,
        (UWORD32)rMAC_PM_CON );
}

/* This function sets the PS bit to 1 in the frames txd by MAC H/w */
INLINE void set_machw_ps_pm_tx_bit(void)
{
    //chenq mod
    //rMAC_PM_CON |= REGBIT2;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PM_CON ) | (REGBIT2) ,
        (UWORD32)rMAC_PM_CON );
}
/*zhq add for powersave*/
INLINE void root_set_machw_ps_pm_tx_bit(void)
{
    //chenq mod
    //rMAC_PM_CON |= REGBIT2;
    root_host_write_trout_reg(
        root_host_read_trout_reg( (UWORD32)rMAC_PM_CON ) | (REGBIT2) ,
        (UWORD32)rMAC_PM_CON );
}

/* This function sets the PS bit to 0 in the frames txd by MAC H/w */
INLINE void reset_machw_ps_pm_tx_bit(void)
{
    //chenq mod
    //rMAC_PM_CON &= ~REGBIT2;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PM_CON ) & (~REGBIT2) ,
        (UWORD32)rMAC_PM_CON );
}
/*zhq add for powersave*/
INLINE void root_reset_machw_ps_pm_tx_bit(void)
{
    //chenq mod
    //rMAC_PM_CON &= ~REGBIT2;
    root_host_write_trout_reg(
        root_host_read_trout_reg( (UWORD32)rMAC_PM_CON ) & (~REGBIT2) ,
        (UWORD32)rMAC_PM_CON );
}

INLINE void set_machw_atim_window(UWORD16 aw)
{
    //chenq mod
    //rMAC_ATIM_WINDOW = convert_to_le((UWORD32)aw);
    host_write_trout_reg( convert_to_le((UWORD32)aw),
    			(UWORD32)rMAC_ATIM_WINDOW);
}

INLINE void set_machw_listen_interval(UWORD16 li)
{
    //chenq mod
    //rMAC_LISTEN_INTERVAL =  convert_to_le((UWORD32)li);
    host_write_trout_reg(  convert_to_le((UWORD32)li), 
						  (UWORD32)rMAC_LISTEN_INTERVAL );
    
}
/*zhq add for powersave*/
INLINE void root_set_machw_listen_interval(UWORD16 li)
{
    //chenq mod
    //rMAC_LISTEN_INTERVAL =  convert_to_le((UWORD32)li);
    root_host_write_trout_reg(  convert_to_le((UWORD32)li), 
						  (UWORD32)rMAC_LISTEN_INTERVAL );
    
}

INLINE void set_machw_offset_interval(UWORD16 oi)
{
    //chenq mod
    //rMAC_OFFSET_INTERVAL =  convert_to_le((UWORD32)oi);
    host_write_trout_reg(  convert_to_le((UWORD32)oi), 
						  (UWORD32)rMAC_OFFSET_INTERVAL );
}

INLINE void set_machw_apsd_ssp(UWORD32 apsd)
{
    //chenq mod
    //rMAC_S_APSD_SSP = convert_to_le((UWORD32)apsd);
    host_write_trout_reg(  convert_to_le((UWORD32)apsd), 
						  (UWORD32)rMAC_S_APSD_SSP );
}

INLINE void set_machw_apsd_si(UWORD32 apsd)
{
    //chenq mod
    //rMAC_S_APSD_SI = convert_to_le((UWORD32)apsd);
    host_write_trout_reg(  convert_to_le((UWORD32)apsd), 
						  (UWORD32)rMAC_S_APSD_SI );
}

/* SMPS Control Register                                                     */
/* ------------------------------------------------------------------------- */
/* | 31 - 10        | 9 - 8            |         7     -      0             | */
/* ------------------------------------------------------------------------- */
/* | Reserved       | SMPS RX Mode     |            Reserved                | */
/* ------------------------------------------------------------------------- */
/*                                                                           */
/* SMPS RX Mode     00  - Reserved. Will have no effect on the mode          */
/*                  01  - Static SM Power Save mode enabled                  */
/*                  10  - Dynamic SM Power Save mode enabled                 */
/*                  11  - SM Power Save mode disabled                        */
/* TX MCS for SMPS  MCS value for Tx in SMPS mode                            */

/* This function sets the SMPS Rx mode */
INLINE void set_machw_smps_rx_mode(UWORD8 val)
{
    UWORD32 temp = ((UWORD16)(val & 0x03)) << 8;

    //chenq mod
    //rMAC_SMPS_CONTROL = convert_to_le(temp);   
    host_write_trout_reg( convert_to_le(temp), 
                          (UWORD32)rMAC_SMPS_CONTROL );
}

/*****************************************************************************/
/* Block Ack register settings                                               */
/*****************************************************************************/

/* BA Parameters register                                                    */
/* ------------------------------------------------------------------------- */
/* | 31 - 25  | 24            | 23 - 22  | 21 - 16  | 15 - 4     | 3 - 0   | */
/* ------------------------------------------------------------------------- */
/* | Reserved | Partial State | Reserved |WinSize   | Seq Number | TID     | */
/* ------------------------------------------------------------------------- */

/* This function sets the Block ACK parameters */
INLINE void set_machw_ba_params(UWORD8 *da, UWORD8 tid, UWORD16 seq_no,
                                UWORD8 win_size, UWORD8 partial_state)
{
    UWORD32 temp = 0;

    temp = 0;
    temp = da[0] << 8;
    temp |= da[1];

    //chenq mod
    //rMAC_BA_PEER_STA_ADDR_MSB = convert_to_le(temp);
    host_write_trout_reg( convert_to_le(temp), 
                          (UWORD32)rMAC_BA_PEER_STA_ADDR_MSB );

    temp  = 0;
    temp  = da[5];
    temp |= (da[4] << 8);
    temp |= (da[3] << 16);
    temp |= (da[2] << 24);

    //chenq mod
    //rMAC_BA_PEER_STA_ADDR_LSB = convert_to_le(temp);
    host_write_trout_reg( convert_to_le(temp), 
                          (UWORD32)rMAC_BA_PEER_STA_ADDR_LSB );

    temp  = (tid & 0x0F);
    temp |= (seq_no & 0x0FFF) << 4;
    temp |= (win_size & 0x3F) << 16;
    temp |= (partial_state & 0x01) << 24;

    //chenq mod
    //rMAC_BA_PARAMS = convert_to_le(temp);
    host_write_trout_reg( convert_to_le(temp), 
                          (UWORD32)rMAC_BA_PARAMS );
}

/* This function resets the Block ACK parameters. This is used for deletion. */
INLINE void reset_machw_ba_params(void)
{
    //chenq mod
    //rMAC_BA_PEER_STA_ADDR_MSB = 0x00000000;
    //rMAC_BA_PEER_STA_ADDR_LSB = 0x00000000;
    //rMAC_BA_PARAMS            = 0x00000000;
    host_write_trout_reg( 0x00000000, 
                          (UWORD32)rMAC_BA_PEER_STA_ADDR_MSB );
    host_write_trout_reg( 0x00000000, 
                          (UWORD32)rMAC_BA_PEER_STA_ADDR_LSB );
    host_write_trout_reg( 0x00000000, 
                          (UWORD32)rMAC_BA_PARAMS );
}

/* This function gets the Block ACK information corresponding to a LUT entry */
/* for a Compressed Block ACK session. Note that this function can be used   */
/* only after read operation on the BA LUT (done by calling the function     */
/* read_machw_ba_lut_entry)                                                  */
INLINE void get_machw_ba_comp_info(UWORD32 *bmaphsw, UWORD32 *bmaplsw,
                                   UWORD32 *params)
{
    //chenq mod
    //*bmaplsw = convert_to_le(rMAC_BA_CBMAP_LSW);
    //*bmaphsw = convert_to_le(rMAC_BA_CBMAP_MSW);
    //*params  = convert_to_le(rMAC_BA_PARAMS);
    *bmaplsw = convert_to_le(host_read_trout_reg( (UWORD32)rMAC_BA_CBMAP_LSW  ));
    *bmaphsw = convert_to_le(host_read_trout_reg( (UWORD32)rMAC_BA_CBMAP_MSW  ));
    *params = convert_to_le(host_read_trout_reg( (UWORD32)rMAC_BA_PARAMS  ));
}

/* BA Control register                                                       */
/* ------------------------------------------------------------------------- */
/* | 31 - 7      | 6 - 3         | 2              | 1          | 0         | */
/* ------------------------------------------------------------------------- */
/* | Reserved    | LUT index     | LUT operation  | LUT enable | BA enable | */
/* ------------------------------------------------------------------------- */
/*                                                                           */
/* BA enable         0 - Feature is disabled                                 */
/*                   1 - Feature is enabled                                  */
/* LUT enable        0 - Idle.                                               */
/*                   1 - Will trigger LUT operation.                         */
/* LUT operation     1 - Write entry                                         */
/*                   0 - Read entry.                                         */
/* LUT index         Index of LUT for updation                               */

/* This function enables the Block ACK feature in MAC H/w */
INLINE void enable_machw_ba(void)
{
    //chenq mod
    //rMAC_BA_CTRL = REGBIT0;
    host_write_trout_reg( REGBIT0, (UWORD32)rMAC_BA_CTRL );
}

/* This function disables the Block ACK feature in MAC H/w */
INLINE void disable_machw_ba(void)
{
    //chenq mod
    //rMAC_BA_CTRL = 0x00000000;
    host_write_trout_reg( 0x00000000, (UWORD32)rMAC_BA_CTRL );
}

/* This function adds a new entry to the BA LUT */
INLINE void add_machw_ba_lut_entry(UWORD8 ba_index, UWORD8 *da, UWORD8 tid,
                                   UWORD16 seq_no, UWORD8 win_size,
                                   UWORD8 partial_state)
{
    UWORD32 temp = 0;

    /* MAC H/w Reqt: PA should be disabled while BA-LUT is programmed */
    set_machw_tx_suspend();
    disable_machw_phy_and_pa();

    /* Wait for current LUT operation to get over */

    //chenq mod
    //while(rMAC_BA_CTRL & REGBIT1)
    while(host_read_trout_reg( (UWORD32)rMAC_BA_CTRL) & REGBIT1)
    {
        temp = temp - 1;
    }

    /* Set all Block ACK parameters */
    set_machw_ba_params(da, tid, seq_no, (win_size-1), partial_state);

    /* Set BA control word as follows,                                       */
    /* BIT3 - BIT6: index; BIT2 = 1 (Write); BIT1 = 1 (enable LUT operation) */
    temp = (ba_index & 0x0F) << 3;
    temp |= BIT2;
    temp |= BIT1;
    temp |= BIT0;

    /* Write operation to LUT */

    //chenq mod
    //rMAC_BA_CTRL = convert_to_le(temp);
    host_write_trout_reg( convert_to_le(temp), (UWORD32)rMAC_BA_CTRL );

    enable_machw_phy_and_pa();
    set_machw_tx_resume();
}

/* This function reads an entry from the BA LUT */
INLINE void read_machw_ba_lut_entry(UWORD8 ba_index)
{
    UWORD32 temp = 0;

    /* MAC H/w Reqt: PA should be disabled while BA-LUT is programmed */
    set_machw_tx_suspend();
    disable_machw_phy_and_pa();

    /* Wait for current LUT operation to get over */

    //chenq mod
    //while(rMAC_BA_CTRL & REGBIT1)
    while(host_read_trout_reg( (UWORD32)rMAC_BA_CTRL) & REGBIT1)
    {
        temp = temp - 1;
    }

    /* Set BA control word as follows,                                       */
    /* BIT3 - BIT6: index; BIT2 = 0 (Read); BIT1 = 1 (enable LUT operation)  */
    temp = (ba_index & 0x0F) << 3;
    temp |= BIT1;
    temp |= BIT0;

    /* Read operation from LUT */

    //chenq mod
    //rMAC_BA_CTRL = convert_to_le(temp);
    host_write_trout_reg( convert_to_le(temp), (UWORD32)rMAC_BA_CTRL );

    /* Wait for current LUT operation to get over */
    //chenq mod
    //while(rMAC_BA_CTRL & REGBIT1)
    while( host_read_trout_reg( (UWORD32)rMAC_BA_CTRL ) & REGBIT1 )
    {
        temp = temp - 1;
    }

    enable_machw_phy_and_pa();
    set_machw_tx_resume();
}

/* This function removes an entry from the BA LUT. This is done by writing a */
/* null entry to the same index.                                             */
INLINE void remove_machw_ba_lut_entry(UWORD8 ba_index)
{
    UWORD32 temp = 0;
    int retry = 0;

    /* MAC H/w Reqt: PA should be disabled while BA-LUT is programmed */
    set_machw_tx_suspend();
    disable_machw_phy_and_pa();

    /* Wait for current LUT operation to get over */
    while((host_read_trout_reg((UWORD32)rMAC_BA_CTRL) & REGBIT1) && (retry++ < 3))
    {
        temp = temp - 1;
        udelay(10);
    }

    if(retry != 0 && retry >= 3)
   		TROUT_DBG4("Warning: wait ba lut op done timeout, BA_CTRL=0x%x!\n", 
   							host_read_trout_reg((UWORD32)rMAC_BA_CTRL));

    /* Reset all Block ACK parameters */
    reset_machw_ba_params();

    /* Set BA control word as follows,                                       */
    /* BIT3 - BIT6: index; BIT2 = 1 (Write); BIT1 = 1 (enable LUT operation) */
    temp = (ba_index & 0x0F) << 3;
    temp |= BIT2;
    temp |= BIT1;
    temp |= BIT0;

    /* Write operation to LUT */

    //chenq mod
    //rMAC_BA_CTRL = convert_to_le(temp);
    host_write_trout_reg( convert_to_le(temp), (UWORD32)rMAC_BA_CTRL );

    enable_machw_phy_and_pa();
    set_machw_tx_resume();
}

/*****************************************************************************/
/* AMPDU LUT                                                                 */
/*****************************************************************************/

/* This function adds a new entry to the AMPDU LUT */
INLINE void add_machw_ampdu_lut_entry(UWORD8 index, UWORD8 maxnum,
                                      UWORD8 minmps, UWORD8 maxraf)
{
    UWORD32 temp = 0;

    /* Wait for current LUT operation to get over */

    //chenq mod
    //while(rMAC_AMPDU_LUT_CTRL & REGBIT0)
    while(host_read_trout_reg((UWORD32)rMAC_AMPDU_LUT_CTRL) & REGBIT0)
    {
        temp = temp - 1;
    }

    /* Set AMPDU control word as follows,                                    */
    /* BIT23: Aggr Enable; BIT15 - 22: MAX NUM Aggr; BIT14 - 13 = MaxRxAMPDU */
    /* Factor; BIT12 - 10: MMPS; BIT9 - 2: LUT index; BIT1 = 0 (Write)       */
    /* BIT0 = 1 (enable LUT operation)                                       */
    temp  = ((UWORD32)maxnum & 0xFF) << 15;
    temp |= ((UWORD32)maxraf & 0x03) << 13;
    temp |= ((UWORD32)minmps & 0x07) << 10;
    temp |= ((UWORD32)index  & 0xFF) << 2;
    temp |= BIT23;
    temp |= BIT0;

    /* Write operation to LUT */
    //chenq mod
    //rMAC_AMPDU_LUT_CTRL = convert_to_le(temp);
    host_write_trout_reg(convert_to_le(temp), (UWORD32)rMAC_AMPDU_LUT_CTRL );
}


/* This function initializes the AMPDU LUT by adding a dummy entry at index  */
/* 0 with aggregation enable bit set to 0. This entry will be referred to by */
/* all sessions having no MPDU aggregation.                                  */
INLINE void init_machw_ra_lut(void)
{
    UWORD32 temp = 0;

    /* Wait for current LUT operation to get over */
    //chenq mod
    //while(rMAC_AMPDU_LUT_CTRL & REGBIT0)
    while(host_read_trout_reg( (UWORD32)rMAC_AMPDU_LUT_CTRL ) & REGBIT0)
    {
        temp = temp - 1;
    }

    /* Set AMPDU control word as follows,                                    */
    /* All parameters = 0; BIT1 = 0 (Write); BIT0 = 1 (enable LUT operation) */

    //chenq mod
    //rMAC_AMPDU_LUT_CTRL = REGBIT0;
    host_write_trout_reg( REGBIT0, (UWORD32)rMAC_AMPDU_LUT_CTRL);
}

/* This function reads an entry from the AMPDU LUT */
INLINE void read_machw_ampdu_lut_entry(UWORD8 index, UWORD32 *ampdu_ctrl)
{
    UWORD32 temp = 0;

    /* Wait for current LUT operation to get over */
    while(host_read_trout_reg((UWORD32)rMAC_AMPDU_LUT_CTRL) & REGBIT0)    
    {
        temp = temp - 1;
    }

    /* Set AMPDU control word as follows,                                    */
    /* BIT23: Aggr Enable; BIT15 - 22: MAX NUM Aggr; BIT14 - 13 = MaxRxAMPDU */
    /* Factor; BIT12 - 10: MMPS; BIT9 - 2: LUT index; BIT1 = 1 (Read )       */
    /* BIT0 = 1 (enable LUT operation)                                       */
    temp = ((UWORD32)index  & 0xFF) << 2;
    temp |= BIT1;
    temp |= BIT0;

    /* Read operation from LUT */
    
    //chenq mod
    //rMAC_AMPDU_LUT_CTRL = convert_to_le(temp);
    host_write_trout_reg(convert_to_le(temp), (UWORD32)rMAC_AMPDU_LUT_CTRL);

    /* Wait for read operation to get over before getting the values */

    //chenq mod
    //while(rMAC_AMPDU_LUT_CTRL & REGBIT0)
    while(host_read_trout_reg( (UWORD32)rMAC_AMPDU_LUT_CTRL) & REGBIT0)
    {
        temp = temp - 1;
    }

    /* Read the register */
    //chenq mod
    //*ampdu_ctrl = convert_to_le(rMAC_AMPDU_LUT_CTRL);
    *ampdu_ctrl = convert_to_le(host_read_trout_reg((UWORD32)rMAC_AMPDU_LUT_CTRL));
}

/* This function removes entry from the AMPDU LUT. This is done by writing a */
/* null entry to the same index.                                             */
INLINE void remove_machw_ampdu_lut_entry(UWORD8 index)
{
    UWORD32 temp = 0;

    /* Wait for current LUT operation to get over */
	while(host_read_trout_reg((UWORD32)rMAC_AMPDU_LUT_CTRL) & REGBIT0)
    {
        temp = temp - 1;
    }

    /* Set AMPDU control word as follows,                                */
    /* BIT9 - 2: LUT index; BIT1 = 0 (Write); BIT0 = 1 (enable LUT oprn) */
    /* All other parameters = 0                                          */
    temp  = ((UWORD32)index  & 0xFF) << 2;
    temp |= BIT0;

    //rMAC_AMPDU_LUT_CTRL = convert_to_le(temp);
    host_write_trout_reg(convert_to_le(temp), (UWORD32)rMAC_AMPDU_LUT_CTRL);
}

/* This function returns the total number of AMPDUs transmitted */
INLINE UWORD32 get_machw_txd_ampdu_count(void)
{
    //chenq mod 
    //return convert_to_le(rMAC_AMPDU_TXD_COUNT);
    return convert_to_le(host_read_trout_reg( 
                            (UWORD32)rMAC_AMPDU_TXD_COUNT));
}

/* This function returns the total number of MPDUs transmitted in the A-MPDU */
INLINE UWORD32 get_machw_tx_mpdus_in_ampdu_count(void)
{
    //chenq mod
    //return convert_to_le(rMAC_TX_MPDUS_IN_AMPDU_COUNT);
    return convert_to_le(host_read_trout_reg( 
                            (UWORD32)rMAC_TX_MPDUS_IN_AMPDU_COUNT));
}

/* This function returns the total number of bytes transmitted in the A-MPDU */
INLINE UWORD32 get_machw_tx_bytes_in_ampdu_count(void)
{
    //chenq mod
    //return convert_to_le(rMAC_TX_BYTES_IN_AMPDU_COUNT);
    return convert_to_le(host_read_trout_reg( 
                            (UWORD32)rMAC_TX_BYTES_IN_AMPDU_COUNT));
}

/* This function returns the number of frames transmitted in 20MHz TXOP */
INLINE UWORD32 get_machw_tx_num_20mhz_txop(void)
{
    //chenq mod
    //return convert_to_le(rTX_NUM_20MHZ_TXOP);
    return convert_to_le(host_read_trout_reg( 
                            (UWORD32)rTX_NUM_20MHZ_TXOP));
}

/* This function returns the number of frames transmitted in 40MHz TXOP */
INLINE UWORD32 get_machw_tx_num_40mhz_txop(void)
{
    //chenq mod
    //return convert_to_le(rTX_NUM_40MHZ_TXOP);
    return convert_to_le(host_read_trout_reg( 
                            (UWORD32)rTX_NUM_40MHZ_TXOP));
}

/* This function returns the number of 20MHz MPDUs frames transmitted in */
/* 40MHz TXOP                                                            */
INLINE UWORD32 get_machw_tx_num_20mhz_mpdu_in_40mhz_txop(void)
{
    //chenq mod
    //return convert_to_le(rTX_NUM_20MHZ_MPDU_IN_40MHZ_TXOP);
    return convert_to_le(host_read_trout_reg( 
                            (UWORD32)rTX_NUM_20MHZ_MPDU_IN_40MHZ_TXOP));
}

/* This function returns the number of MPDUs which are promoted to 40MHz */
INLINE UWORD32 get_machw_tx_num_promoted_mpdu(void)
{
    //chenq mod
    //return convert_to_le(rTX_NUM_PROMOTED_MPDU);
    return convert_to_le(host_read_trout_reg( 
                            (UWORD32)rTX_NUM_PROMOTED_MPDU));
}

/* This function returns the number of MPDUs which are demoted to 20MHz */
INLINE UWORD32 get_machw_tx_num_demoted_mpdu(void)
{
    //chenq mod
    //return convert_to_le(rTX_NUM_MPDU_DEMOTED);
    return convert_to_le(host_read_trout_reg( 
                            (UWORD32)rTX_NUM_MPDU_DEMOTED));
}

/* This function returns the number of MPDUs which are promoted to 40MHz by */
/* using protection frame exchange                                          */
INLINE UWORD32 get_machw_tx_num_prom_prot(void)
{
    //chenq mod
    //return convert_to_le(rTX_NUM_PROMOTED_PROT);
    return convert_to_le(host_read_trout_reg( 
                            (UWORD32)rTX_NUM_PROMOTED_PROT));
}

/* This function returns the number of MPDUs which are promoted to 40MHz by */
/* using protection frame exchange                                          */
INLINE UWORD32 get_machw_tx_num_fc_prot(void)
{
    //chenq mod
    //return convert_to_le(rTX_NUM_PROT_DUE_TO_FC);
    return convert_to_le(host_read_trout_reg( 
                            (UWORD32)rTX_NUM_PROT_DUE_TO_FC));
}

/* This function returns the number of times the TXOP is aborted due to the */
/* Secondary Channel being busy                                             */
INLINE UWORD32 get_machw_tx_num_txop_abort_sec_busy(void)
{
    //chenq mod
    //return convert_to_le(rTX_NUM_TXOP_ABORT_ON_SEC_BUSY);
    return convert_to_le(host_read_trout_reg( 
                            (UWORD32)rTX_NUM_TXOP_ABORT_ON_SEC_BUSY));
}

/*****************************************************************************/
/* HCCA AP Registers                                                         */
/*****************************************************************************/

/* Schedule Link Address register          */
/* --------------------------------------- */
/* | 31 - 24    | 23 - 0                 | */
/* --------------------------------------- */
/* | Reserved   | Schedule Link Address  | */
/* --------------------------------------- */

INLINE void set_machw_sched_link_addr(UWORD32 value)
{
    UWORD32 temp = value & 0x00FFFFFF;

    //chenq mod
    //rMAC_SCHEDULE_LINK_ADDR = convert_to_le(temp);
    host_write_trout_reg( convert_to_le(temp), 
                         (UWORD32)rMAC_SCHEDULE_LINK_ADDR );
}

/* CAP Start Time                                                        */
/* --------------------------------------------------------------------- */
/* 31 - 24   | 23 - 16              | 15 - 8   | 7 - 0                 | */
/* --------------------------------------------------------------------- */
/* Reserved  | CAP Pre-start offset | Reserved | CAP start time before | */
/*           | time                 |          | TBTT                  | */
/* --------------------------------------------------------------------- */

INLINE void set_machw_cap_start(UWORD8 value)
{
    //chenq mod
    //rMAC_CAP_START_TIME &= MASK_INVERSE(8, 0);
    //rMAC_CAP_START_TIME |= convert_to_le((UWORD32)value) & MASK(8, 0);
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_CAP_START_TIME ) 
            & (MASK_INVERSE(8, 0)) ,
        (UWORD32)rMAC_CAP_START_TIME );

    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_CAP_START_TIME ) 
            | (convert_to_le((UWORD32)value) & MASK(8, 0)) ,
        (UWORD32)rMAC_CAP_START_TIME );
}

INLINE void set_machw_cap_pre_start(UWORD8 value)
{
    UWORD32 temp = ((UWORD32)value) << 16;

    //chenq mod
    //rMAC_CAP_START_TIME &= MASK_INVERSE(8, 16);
    //rMAC_CAP_START_TIME |= convert_to_le(temp) & MASK(8, 16);

    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_CAP_START_TIME ) 
            & (MASK_INVERSE(8, 16)) ,
        (UWORD32)rMAC_CAP_START_TIME );

    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_CAP_START_TIME ) 
            | (convert_to_le(temp) & MASK(8, 16)) ,
        (UWORD32)rMAC_CAP_START_TIME );
}

/* Inline functions for miscellaneous actions involving MAC H/w registers    */

/* This function introduces a delay by performing a dummy loop operation. A  */
/* read-only register is written to so that the empty loop is not removed at */
/* compile time for optimization.                                            */

//#define HOST_DELAY_FOR_TROUT_PHY 0x00000000	//add by chengq, value need to be confirmed!!!
extern struct timespec tv;
//#define HOST_DELAY_FOR_TROUT_PHY
#define GET_TIME()		do{	getnstimeofday(&tv);	}while(0)

INLINE void add_delay(UWORD32 count)
{
    UWORD32 i = 0;


    for(i = 0; i < count; i++)
    {
        //chenq mod
        //rMAC_PA_VER = i;
        //host_write_trout_reg( i, (UWORD32)rMAC_PA_VER );
       // (*(volatile UWORD32 *)HOST_DELAY_FOR_TROUT_PHY);
        #if 0 //dumy add for debug 0814
         GET_TIME();
        #else
        udelay(10);
        #endif
    }
}

/* This function returns the value of the register containing  total number */
/* of frames received without PHY level errors.                             */
INLINE UWORD32 get_machw_rx_end_count(void)
{
    return convert_to_le(host_read_trout_reg((UWORD32)rMAC_RX_END_COUNT));
}

INLINE void set_machw_rx_end_count(UWORD32 inp)
{
    host_write_trout_reg(inp, (UWORD32)rMAC_RX_END_COUNT);
}

/* This function returns the value of the register containing the total */
/* number of PHY receive header error indications.                      */
INLINE UWORD32 get_machw_rx_error_end_count(void)
{
    //chenq mod
    //return convert_to_le(rMAC_RX_ERROR_END_COUNT);
    return convert_to_le(host_read_trout_reg( 
                                (UWORD32)rMAC_RX_ERROR_END_COUNT ));
}

INLINE void set_machw_rx_error_end_count(UWORD32 inp)
{
    host_write_trout_reg(inp, (UWORD32)rMAC_RX_ERROR_END_COUNT);
}


/* This function returns the total number of AMPDUs received */
INLINE UWORD32 get_machw_rxd_ampdu_count(void)
{
    //chenq mod
    //return convert_to_le(rMAC_AMPDU_RXD_COUNT);
    return convert_to_le(host_read_trout_reg( 
                                (UWORD32)rMAC_AMPDU_RXD_COUNT));
}

/* This function returns the total number of MPDUs received in the A-MPDU */
INLINE UWORD32 get_machw_rx_mpdus_in_ampdu_count(void)
{
    //chenq mod
    //return convert_to_le(rMAC_RX_MPDUS_IN_AMPDU_COUNT);
    return convert_to_le(host_read_trout_reg( 
                                (UWORD32)rMAC_RX_MPDUS_IN_AMPDU_COUNT ));
}

/* This function returns the total number of bytes received in the A-MPDU */
INLINE UWORD32 get_machw_rx_bytes_in_ampdu_count(void)
{
    //chenq mod
    //return convert_to_le(rMAC_RX_BYTES_IN_AMPDU_COUNT);
    return convert_to_le(host_read_trout_reg( 
                                (UWORD32)rMAC_RX_BYTES_IN_AMPDU_COUNT ));
}

/* This function returns the value of CRC errors of an AMPDU delimiter */
/* when this is the first CRC error in the received A-MPDU or when the */
/* previous delimiter has been decoded correctly.                      */
INLINE UWORD32 get_machw_ampdu_dlmt_crc_error_count(void)
{
    //chenq mod
    //return convert_to_le(rMAC_AMPDU_DLMT_ERROR_COUNT);
    return convert_to_le(host_read_trout_reg( 
                                (UWORD32)rMAC_AMPDU_DLMT_ERROR_COUNT ));
}

/* The function polls the TSF Timer for a specified duration in msecs (<4000s)*/
/* TSF is started before and disabled after the operation is completed        */
INLINE void machw_poll_after_reset(UWORD32 msec)
{
    if(msec > 4000000 ||msec==0 )
    {
       return;
    }
    mdelay(msec);
}

/*  TX Power Setting Control Register                                                */
/* --------------------------------------------------------------------------------  */
/* 31 - 25  |      24      |    23 - 16   | 15 - 9   |       8      |    7 - 0     | */
/* --------------------------------------------------------------------------------  */
/* Reserved | 11A TX Power | 11A TX Power | Reserved | 11B TX Power | 11B TX Power | */
/*          | Reset Enable | Setting Value|          | Reset Enable | Setting Value| */
/* --------------------------------------------------------------------------------  */
INLINE void set_tx_power_11a_after_spi_lock(UWORD8 val)
{
    UWORD32 temp = 0;

    /* Set the power for OFDM transmission */
    temp = ((UWORD32)val << 8);

    //chenq mod
    //rMAC_PHY_TX_PWR_SET_REG &= MASK_INVERSE(8, 8);
    //rMAC_PHY_TX_PWR_SET_REG |= convert_to_le(temp) & MASK(8, 8);
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PHY_TX_PWR_SET_REG ) 
            & (MASK_INVERSE(8, 8)) ,
        (UWORD32)rMAC_PHY_TX_PWR_SET_REG );
    
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PHY_TX_PWR_SET_REG ) 
            | (convert_to_le(temp) & MASK(8, 8)) ,
        (UWORD32)rMAC_PHY_TX_PWR_SET_REG );

    /* Set the power for HT transmission */
    temp = ((UWORD32)val << 16);

    //chenq mod
    //rMAC_PHY_TX_PWR_SET_REG &= MASK_INVERSE(8, 16);
    //rMAC_PHY_TX_PWR_SET_REG |= convert_to_le(temp) & MASK(8, 16);

    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PHY_TX_PWR_SET_REG ) 
            & (MASK_INVERSE(8, 16)) ,
        (UWORD32)rMAC_PHY_TX_PWR_SET_REG );

    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PHY_TX_PWR_SET_REG ) 
            | (convert_to_le(temp) & MASK(8, 16)) ,
        (UWORD32)rMAC_PHY_TX_PWR_SET_REG );
}

INLINE void set_tx_power_11n_after_spi_lock(UWORD8 val)
{
    UWORD32 temp = 0;

    /* Set the power for HT transmission */
    temp = ((UWORD32)val << 16);

    //chenq mod
    //rMAC_PHY_TX_PWR_SET_REG &= MASK_INVERSE(8, 16);
    //rMAC_PHY_TX_PWR_SET_REG |= convert_to_le(temp) & MASK(8, 16);

    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PHY_TX_PWR_SET_REG ) 
            & (MASK_INVERSE(8, 16)) ,
        (UWORD32)rMAC_PHY_TX_PWR_SET_REG );

    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PHY_TX_PWR_SET_REG ) 
            | (convert_to_le(temp) & MASK(8, 16)) ,
        (UWORD32)rMAC_PHY_TX_PWR_SET_REG );
}

INLINE void set_tx_power_11b_after_spi_lock(UWORD8 val)
{
    UWORD32 temp = ((UWORD32)val);

    //chenq mod
    //rMAC_PHY_TX_PWR_SET_REG &= MASK_INVERSE(8, 0);
    //rMAC_PHY_TX_PWR_SET_REG |= convert_to_le((temp << 0)) & MASK(8, 0);

    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PHY_TX_PWR_SET_REG ) 
            & (MASK_INVERSE(8, 0)) ,
        (UWORD32)rMAC_PHY_TX_PWR_SET_REG );

    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PHY_TX_PWR_SET_REG ) 
            | (convert_to_le((temp << 0)) & MASK(8, 0)) ,
        (UWORD32)rMAC_PHY_TX_PWR_SET_REG );
}

// 20120830 caisf add, merged ittiam mac v1.3 code
INLINE void set_machw_ant_set(UWORD32 val)
{
	//rMAC_ANTENNA_SET = convert_to_le(val);
    host_write_trout_reg(convert_to_le(val), (UWORD32)rMAC_ANTENNA_SET );
}

/* The following functions are used only for H/w unit testing */

/* Test mode register                                                        */
/*                                                                           */
/* 31 - 2              1                             0                       */
/* Reserved            Sequence number control       Tx FCS control          */
/*                                                                           */
/* Tx FCS Control              0 - Do not corrupt Tx FCS                     */
/*                             1 - Corrupt Tx FCS                            */
/* Sequence Number Control     0 - Insert sequence number                    */
/*                             1 - Do not insert sequence number             */

/* This function enables Tx FCS corruption */
INLINE void enable_machw_tx_fcs_corruption(void)
{
    //chenq mod
    //rMAC_TEST_MODE |= REGBIT0;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_TEST_MODE ) | (REGBIT0) ,
        (UWORD32)rMAC_TEST_MODE );
}

/* This function disables Tx FCS corruption */
INLINE void disable_machw_tx_fcs_corruption(void)
{
    //chenq mod
    //rMAC_TEST_MODE &= ~REGBIT0;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_TEST_MODE ) & (~REGBIT0) ,
        (UWORD32)rMAC_TEST_MODE );
}

/* This function enables sequence number insertion by MAC H/w */
INLINE void enable_machw_seq_num_insertion(void)
{
    //chenq mod
    //rMAC_TEST_MODE &= ~REGBIT1;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_TEST_MODE ) & (~REGBIT1) ,
        (UWORD32)rMAC_TEST_MODE );
}

/* This function disables sequence number insertion by MAC H/w */
INLINE void disable_machw_seq_num_insertion(void)
{
    //chenq mod
    //rMAC_TEST_MODE |= REGBIT1;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_TEST_MODE ) | (REGBIT1) ,
        (UWORD32)rMAC_TEST_MODE );
}

/* MAC H/w Reset Control register                                            */
/* ------------------------------------------------------------------------- */
/* | 31 - 6 |         5          |      4   |  3   |   2    |     1 |     0 |*/
/* ------------------------------------------------------------------------- */
/* |Reserved| Duplicate detction |Defrag LUT| TSF  |CE core |PA core|MAC HW |*/
/* ------------------------------------------------------------------------- */
/*                                                                           */
/* MAC H/W reset: Toggling of this bit causes the entire MAC H/W to be reset */
/*                including the register banks of PA and CE                  */
/* PA core reset: Toggling of this bit causes the PA core logic to be reset. */
/*                The register bank of PA, TSF control block, Defrag LUT,    */
/*                Duplicate detection and interrupt FIFOs are not reset.     */
/* CE Core reset: Toggling of this bit causes the CE core logic to be reset. */
/*                The register bank of CE is not reset                       */
/* TSF Reset    : Toggling of this bit causes the TSF control block to be    */
/*                reset.                                                     */
/* Defrag LUT Reset : Toggling of this bit causes the Defrag LUT to be reset */
/* Duplicate detection reset : Toggling of this bit causes the duplicate     */
/*                             detection block to be reset.                  */

/* This function resets the MAC H/w */
INLINE void reset_machw(void)
{
    //chenq mod
    //rMAC_RESET_CTRL = rMAC_RESET_CTRL ^ REGBIT0;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_RESET_CTRL ) ^ (REGBIT0) ,
        (UWORD32)rMAC_RESET_CTRL );
}


/* This function resets the PA Core and the register bank of PA,TSF control  */
/* block, Defrag LUT,Duplicate detection and interrupt FIFOs are not reset   */
INLINE void reset_pacore(void)
{
    //chenq mod
    //rMAC_RESET_CTRL = rMAC_RESET_CTRL ^ REGBIT1;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_RESET_CTRL ) ^ (REGBIT1) ,
        (UWORD32)rMAC_RESET_CTRL );
}

/* This function resets the CE Core and the register bank of CE is not reset */
INLINE void reset_cecore(void)
{
    //chenq mod
    //rMAC_RESET_CTRL = rMAC_RESET_CTRL ^ REGBIT2;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_RESET_CTRL ) ^ (REGBIT2) ,
        (UWORD32)rMAC_RESET_CTRL );
}

/* This function resets TSF Control block */
INLINE void reset_tsf(void)
{
    //chenq mod
    //rMAC_RESET_CTRL = rMAC_RESET_CTRL ^ REGBIT3;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_RESET_CTRL ) ^ (REGBIT3) ,
        (UWORD32)rMAC_RESET_CTRL );
}

/* This function resets DEFRAG LUT */
INLINE void reset_defrag_lut(void)
{
    //chenq mod
    //rMAC_RESET_CTRL = rMAC_RESET_CTRL ^ REGBIT4;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_RESET_CTRL ) ^ (REGBIT4) ,
        (UWORD32)rMAC_RESET_CTRL );
}

/* This function resets Duplicate detection block */
INLINE void reset_duplicate_det(void)
{
    //chenq mod
    //rMAC_RESET_CTRL = rMAC_RESET_CTRL ^ REGBIT5;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_RESET_CTRL ) ^ (REGBIT5) ,
        (UWORD32)rMAC_RESET_CTRL );
}


/* This function checks whether the error is a H/w serious error */
INLINE BOOL_T is_serious_hw_error(UWORD8 error_code)
{
    switch(error_code)
    {
       case UNEXPECTED_MAC_TX_HANG:
       case UNEXPECTED_PHY_TX_HANG:
       case UNEXPECTED_TX_STATUS3:
       case BUS_ERROR:
       case BUS_WRAP_SIG_ERROR:
       case DMA_NO_ERROR:
       case TX_CE_NOT_READY:
       case RX_CE_NOT_READY:
       case SEQNUM_GEN_ADDR_FAIL:
       case UNEXPECTED_MSDU_ADDR:
       {
          return BTRUE;
       }
       default:
       {
          return BFALSE;
       }
    }
}


// 20120709 caisf add, merged ittiam mac v1.2 code
#ifdef MWLAN
/* MAC H/w ODDR Control register                                             */
/* ------------------------------------------------------------------------- */
/* | 31 - 2 |     1                |     0                                  |*/
/* ------------------------------------------------------------------------- */
/* |Reserved| Invert Phy RegIf Clk |Invert Phy Tx Clk                       |*/
/* ------------------------------------------------------------------------- */
/* Invert Phy Tx Clk   : 0 - Do not invert Phy Tx Clk ,1 - Invert Phy Tx Clk */
/* Invert Phy RegIf Clk: 0 - Do not invert Phy RegIf Clk,                    */
/*                     : 1 - Invert Phy RegIf Clk                            */

INLINE void set_mac_oddr_inv_phy_txclk(void)
{
    //rMAC_ODDR_CTRL |= REGBIT0;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_ODDR_CTRL ) 
        |  REGBIT0,
        (UWORD32)rMAC_ODDR_CTRL );
}

INLINE void set_mac_oddr_inv_phyreg_ifclk(void)
{
    //rMAC_ODDR_CTRL |= REGBIT1;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_ODDR_CTRL ) 
        |  REGBIT1,
        (UWORD32)rMAC_ODDR_CTRL );
}
#endif /* MWLAN */

#ifdef TX_ABORT_FEATURE

/* TX abort frame duration - timeout value                                   */
/* ------------------------------------------------------------------------- */
/* | 31 - 16                          | 15 - 0                             | */
/* ------------------------------------------------------------------------- */
/* | TX abort frame timeout           | TX abort frame duration/ID value   | */
/* ------------------------------------------------------------------------- */
/*                                                                           */
/* TX abort frame Dur/ID  : Value to be inserted in Dur/ID field in the Self */
/*                          CTS frame used for TX abort (in usec)            */
/* TX abort frame timeout : Timeout period for the transmission of the Self  */
/*                          CTS after TX abort request is asserted (in usec) */

INLINE void set_machw_tx_abort_dur(UWORD16 val)
{
    //chenq mod
    //rMAC_TX_ABORT_FRM_DUR_TIMEOUT &= MASK_INVERSE(16, 0);
    //rMAC_TX_ABORT_FRM_DUR_TIMEOUT |= convert_to_le(val) & MASK(16, 0);
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_TX_ABORT_FRM_DUR_TIMEOUT ) 
        & (MASK_INVERSE(16, 0)) ,
        (UWORD32)rMAC_TX_ABORT_FRM_DUR_TIMEOUT );

    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_TX_ABORT_FRM_DUR_TIMEOUT ) 
        | (convert_to_le(val) & MASK(16, 0)) ,
        (UWORD32)rMAC_TX_ABORT_FRM_DUR_TIMEOUT );
}

INLINE void set_machw_tx_abort_timeout(UWORD16 val)
{
    //chenq mod
    //rMAC_TX_ABORT_FRM_DUR_TIMEOUT &= MASK_INVERSE(16, 16);
    //rMAC_TX_ABORT_FRM_DUR_TIMEOUT |= convert_to_le((val << 16)) & MASK(16, 16);
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_TX_ABORT_FRM_DUR_TIMEOUT ) 
        & (MASK_INVERSE(16, 16)) ,
        (UWORD32)rMAC_TX_ABORT_FRM_DUR_TIMEOUT );

    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_TX_ABORT_FRM_DUR_TIMEOUT ) 
        | (convert_to_le((val << 16)) & MASK(16, 16)) ,
        (UWORD32)rMAC_TX_ABORT_FRM_DUR_TIMEOUT );
}

/* TX abort frame rate                                                       */
/* ------------------------------------------------------------------------- */
/* | 31 - 9   | 15 - 8                        | 7 - 0                      | */
/* ------------------------------------------------------------------------- */
/* | Reserved | TX abort frame Tx power level | TX abort frame rate        | */
/* ------------------------------------------------------------------------- */

INLINE void set_machw_tx_abort_frame_rate(UWORD8 val)
{
    //chenq mod
    //rMAC_TX_ABORT_FRM_RATE &= MASK_INVERSE(8, 0);
    //rMAC_TX_ABORT_FRM_RATE |= convert_to_le(val) & MASK(8, 0);
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_TX_ABORT_FRM_RATE ) 
        & (MASK_INVERSE(8, 0)) ,
        (UWORD32)rMAC_TX_ABORT_FRM_RATE );
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_TX_ABORT_FRM_RATE ) 
        | (convert_to_le(val) & MASK(8, 0)) ,
        (UWORD32)rMAC_TX_ABORT_FRM_RATE );
}

INLINE void set_machw_tx_abort_frame_tx_pow_level(UWORD8 val)
{
    //chenq mod
    //rMAC_TX_ABORT_FRM_RATE &= MASK_INVERSE(8, 8);
    //rMAC_TX_ABORT_FRM_RATE |= convert_to_le((val << 8)) & MASK(8, 8);
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_TX_ABORT_FRM_RATE ) 
        & (MASK_INVERSE(8, 8)) ,
        (UWORD32)rMAC_TX_ABORT_FRM_RATE );
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_TX_ABORT_FRM_RATE ) 
        | ( convert_to_le((val << 8)) & MASK(8, 8) ) ,
        (UWORD32)rMAC_TX_ABORT_FRM_RATE );
}

/* TX abort Frame PHY TX Mode                                                */
/* ------------------------------------------------------------------------- */
/* | 31 - 0                                                                | */
/* ------------------------------------------------------------------------- */
/* | TX abort Frame PHY TX Mode                                            | */
/* ------------------------------------------------------------------------- */

INLINE void set_machw_tx_abort_frame_ptm(UWORD32 val)
{
    //chenq mod
    //rMAC_TX_ABORT_FRM_PHY_TX_MODE = convert_to_le(val);
    host_write_trout_reg( convert_to_le(val), (UWORD32)rMAC_TX_ABORT_FRM_PHY_TX_MODE );
}

#endif /* TX_ABORT_FEATURE */

/* The Extended PA control register has various subfields described below.   */
/*                                                                           */
/*                      3                2              1                0   */
/*        SifsRespAtNonHT  PAandBusClkSync  RxBufBasedAck   DoNotTouchMPDU   */
/*                                                                           */
/* DoNotTouchMPDU  0 - H/w will insert relevant field in MPDUs during Tx     */
/*                 1 - H/W will not insert any field in MPDU during Tx       */
/* RxBufBasedAck   0 - H/w will Tx ACK frames for all received frames which  */
/*                     pass FCS check.                                       */
/*                 1 - H/W will not Tx ACK frames even for FCS check passed  */
/*                     frames if it does not have Buffers for receiving it.  */

INLINE void enable_machw_dont_touch_mpdu(void)
{
    //chenq mod
    //rMAC_EXTENDED_PA_CON |= REGBIT0;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_EXTENDED_PA_CON ) | (REGBIT0) ,
        (UWORD32)rMAC_EXTENDED_PA_CON );
}

INLINE void disable_machw_dont_touch_mpdu(void)
{
    //chenq mod
    //rMAC_EXTENDED_PA_CON &= ~REGBIT0;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_EXTENDED_PA_CON ) & (~REGBIT0) ,
        (UWORD32)rMAC_EXTENDED_PA_CON );
}

INLINE void enable_rx_buff_based_ack(void)
{
    //chenq mod
    //rMAC_EXTENDED_PA_CON |= REGBIT1;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_EXTENDED_PA_CON ) | (REGBIT1) ,
        (UWORD32)rMAC_EXTENDED_PA_CON );
}

INLINE void disable_rx_buff_based_ack(void)
{
    //chenq mod
    //rMAC_EXTENDED_PA_CON &= ~REGBIT1;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_EXTENDED_PA_CON ) & (~REGBIT1) ,
        (UWORD32)rMAC_EXTENDED_PA_CON );
}

/* This function indicates to MAC H/w that PA Clock and the AHB Master clocks */
/* are synchronous.                                                           */
INLINE void set_pa_bus_clock_sync(void)
{
    //chenq mod
    //rMAC_EXTENDED_PA_CON |= REGBIT2;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_EXTENDED_PA_CON ) | (REGBIT2) ,
        (UWORD32)rMAC_EXTENDED_PA_CON );
}

/* This function indicates to MAC H/w that PA Clock and the AHB Master clocks */
/* are not synchronous.                                                       */
INLINE void reset_pa_bus_clock_sync(void)
{
    //chenq mod
    //rMAC_EXTENDED_PA_CON &= ~REGBIT2;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_EXTENDED_PA_CON ) & (~REGBIT2) ,
        (UWORD32)rMAC_EXTENDED_PA_CON );
}

/* This function enables transmission of the SIFS response frame at basic */
/* rate only.                                                             */
INLINE void enable_basic_rt_sifs_resp(void)
{
    //chenq mod
    //rMAC_EXTENDED_PA_CON |= REGBIT3;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_EXTENDED_PA_CON ) | (REGBIT3) ,
        (UWORD32)rMAC_EXTENDED_PA_CON );
}

/* This function enables transmission of the SIFS response frame at basic */
/* rate only.                                                             */
INLINE void disable_basic_rt_sifs_resp(void)
{
    //chenq mod
    //rMAC_EXTENDED_PA_CON &= ~REGBIT3;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_EXTENDED_PA_CON ) & (~REGBIT3) ,
        (UWORD32)rMAC_EXTENDED_PA_CON );
}

//chenq add for wapi 20120919
/*2012 09 19 add bit5 for wapi*/
#ifdef MAC_WAPI_SUPP
INLINE void enable_wapi(void)
{
    //chenq mod
    //rMAC_EXTENDED_PA_CON |= REGBIT5;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_EXTENDED_PA_CON ) | (REGBIT5) ,
        (UWORD32)rMAC_EXTENDED_PA_CON );
}

INLINE void disable_wapi(void)
{
    //chenq mod
    //rMAC_EXTENDED_PA_CON &= ~REGBIT5;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_EXTENDED_PA_CON ) & (~REGBIT5) ,
        (UWORD32)rMAC_EXTENDED_PA_CON );
}
#endif


/* RX Lifetime limit                                                         */
/* ------------------------------------------------------------------------- */
/* | 31 - 10                              |  9 - 0                         | */
/* ------------------------------------------------------------------------- */
/* | Reserved                             |  RX lifetime limit             | */
/* ------------------------------------------------------------------------- */
/*                                                                           */
/* Rx lifetime limit : Rx lifetime value used during defragmentation (in ms) */

INLINE void set_machw_rx_lifetime(UWORD16 val)
{
    //chenq mod
    //rMAC_RX_LIFETIME_LIMIT &= MASK_INVERSE(10, 0);
    //rMAC_RX_LIFETIME_LIMIT |= convert_to_le(val) & MASK(10, 0);
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_RX_LIFETIME_LIMIT ) 
                            & (MASK_INVERSE(10, 0)) ,
        (UWORD32)rMAC_RX_LIFETIME_LIMIT );

    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_RX_LIFETIME_LIMIT ) 
                            | (convert_to_le(val) & MASK(10, 0)) ,
        (UWORD32)rMAC_RX_LIFETIME_LIMIT );
}

/* Hi priority RX Queue buffer address                                       */
/* ------------------------------------------------------------------------- */
/* | 31 - 0                                                                | */
/* ------------------------------------------------------------------------- */
/* | Hi priority RX Queue buffer address                                   | */
/* ------------------------------------------------------------------------- */

INLINE void set_machw_hip_rx_buff_addr(UWORD32 addr)
{
    //chenq mod
    //rMAC_HIP_RX_BUFF_ADDR = convert_to_le(virt_to_phy_addr(addr));
    host_write_trout_reg( convert_to_le(virt_to_phy_addr(addr)), 
                          (UWORD32)rMAC_HIP_RX_BUFF_ADDR );
}

/* High Priority Rx Queue control                                            */
/* 31 - 5     4     3           2          1       0                         */
/* Reserved  ATIM   Probe Rsp   Probe Req  Beacon  HighPriorityQEnable       */
/*                                                                           */
/* High Priority Queue Enable  1 - Enable HP RxQ                             */
/*                             0 - Disable HP RxQ                            */
/* Beacon                      1 - keep Beacon frame in HP RxQ               */
/*                             0 - keep it in normal Queue                   */
/* Probe Request               1 - keep Probe Request frame in HP RxQ        */
/*                             0 - keep it in normal Queue                   */
/* Probe Response              1 - keep Probe Response frame in HP RxQ       */
/*                             0 - keep it in normal Queue                   */
/* ATIM                        1 - keep ATIM frame in HP RxQ                 */
/*                             0 - keep it in normal Queue                   */

INLINE void set_machw_hip_rxq_con(UWORD32 val)
{
    //chenq mod
    //rMAC_HIP_RXQ_CON = convert_to_le(val);
    host_write_trout_reg( convert_to_le(val), (UWORD32)rMAC_HIP_RXQ_CON );
}

/* Sub-MSDU gap                                                              */
/* ------------------------------------------------------------------------- */
/* | 31 - 16                         | 7 - 0                               | */
/* ------------------------------------------------------------------------- */
/* | Reserved                        | Sub-MSDU gap                        | */
/* ------------------------------------------------------------------------- */
/*                                                                           */
/* Sub-MSDU gap : Gap between the MSDUs of an AMSDU (in words)               */

INLINE void set_machw_sub_msdu_gap(UWORD8 val)
{
    //chenq mod
    //rMAC_SUB_MSDU_GAP &= MASK_INVERSE(8, 0);
    //rMAC_SUB_MSDU_GAP |= convert_to_le(val) & MASK(8, 0);

    
    host_write_trout_reg(
            host_read_trout_reg( (UWORD32)rMAC_SUB_MSDU_GAP ) 
                & (MASK_INVERSE(8, 0)) ,
            (UWORD32)rMAC_SUB_MSDU_GAP );

    
    host_write_trout_reg(
            host_read_trout_reg( (UWORD32)rMAC_SUB_MSDU_GAP ) 
                | (convert_to_le(val) & MASK(8, 0)) ,
            (UWORD32)rMAC_SUB_MSDU_GAP );

}

/* De-authentication reason code                                             */
/* ------------------------------------------------------------------------- */
/* | 31 - 4                        |  3 - 0                                | */
/* ------------------------------------------------------------------------- */
/* | Reserved                      |  De-authentication reason Code        | */
/* ------------------------------------------------------------------------- */

INLINE UWORD16 get_machw_deauth_reason_code(void)
{
    //chenq mod
    //UWORD32 temp = convert_to_le(rMAC_DEAUTH_REASON_CODE);
    UWORD32 temp = convert_to_le(host_read_trout_reg( 
                                    (UWORD32)rMAC_DEAUTH_REASON_CODE));

    return(UWORD16)temp;
}

/* This function is used to get duration of Tx abort */
INLINE UWORD32 get_machw_tx_abort_dur(void)
{
    //chenq mod
    //return (rMAC_TX_ABORT_FRM_DUR_TIMEOUT & 0xFFFF);
    return (host_read_trout_reg( (UWORD32)rMAC_TX_ABORT_FRM_DUR_TIMEOUT )
                & 0xFFFF);
}

/* This function is used to get Tx abort frame time out duartion  */
INLINE UWORD32 get_machw_tx_abort_timeout(void)
{
    //chenq mod
    //return ((rMAC_TX_ABORT_FRM_DUR_TIMEOUT & 0xFFFF0000) >> 16);
    return ( (host_read_trout_reg( (UWORD32)rMAC_TX_ABORT_FRM_DUR_TIMEOUT )
                & 0xFFFF0000) >> 16 );
}

/* This function is used to get rate of self CTS frame */
INLINE UWORD16 get_machw_tx_abort_frame_rate(void)
{
    //chenq mod
    //return rMAC_TX_ABORT_FRM_RATE;
    return host_read_trout_reg( (UWORD32)rMAC_TX_ABORT_FRM_RATE );
}

/* This function initializes the Tx abort parameters */
INLINE void init_tx_abort_params(void)
{
#ifdef TX_ABORT_FEATURE
    disable_machw_tx_abort();
    enable_machw_txabort_sw_trig();
    disable_machw_selfcts_on_txab();
    set_machw_tx_abort_frame_rate(0x3);
    set_machw_tx_abort_dur(0x7FFF);
    set_machw_tx_abort_frame_ptm(0x00010140);
    set_machw_tx_abort_timeout(0x7FFF);
    unmask_machw_txab_start_int();
    unmask_machw_txab_end_int();
#else /* TX_ABORT_FEATURE */
    mask_machw_txab_start_int();
    mask_machw_txab_end_int();
#endif /* TX_ABORT_FEATURE */
}

INLINE void enable_machw_rx_watchdog_timer(UWORD8 val)
{
    //chenq mod
    //rMAC_PHY_TIMEOUT_ADJUST |= REGBIT8;

    //rMAC_PHY_TIMEOUT_ADJUST &= MASK_INVERSE(8, 9);
    //rMAC_PHY_TIMEOUT_ADJUST |= convert_to_le((val << 9)) & MASK(8, 9);
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PHY_TIMEOUT_ADJUST ) 
            | (REGBIT8) ,
        (UWORD32)rMAC_PHY_TIMEOUT_ADJUST );
    
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PHY_TIMEOUT_ADJUST ) 
            & (MASK_INVERSE(8, 9)) ,
        (UWORD32)rMAC_PHY_TIMEOUT_ADJUST );
    
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PHY_TIMEOUT_ADJUST ) 
            | (convert_to_le((val << 9)) & MASK(8, 9)) ,
        (UWORD32)rMAC_PHY_TIMEOUT_ADJUST );
}

INLINE void disable_machw_rx_watchdog_timer(void)
{
    //chenq mod
    //rMAC_PHY_TIMEOUT_ADJUST &= ~REGBIT8;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PHY_TIMEOUT_ADJUST ) 
            & (~REGBIT8) ,
        (UWORD32)rMAC_PHY_TIMEOUT_ADJUST );
}


INLINE void handle_machw_utxstat1_erri(void)
{
    mask_machw_txab_start_int();
    mask_machw_txab_end_int();
    disable_machw_selfcts_on_txab();
    enable_machw_txabort_sw_trig();
    enable_machw_tx_abort();
    wait_for_machw_tx_abort_start();
    disable_machw_tx_abort();
    wait_for_machw_tx_abort_end();
    /* in this case, we must reset entire mac and phy  by zhao  6-25 2013 */
    reset_phy_machw();
#ifdef TX_ABORT_FEATURE
    unmask_machw_txab_start_int();
    unmask_machw_txab_end_int();
#endif /* TX_ABORT_FEATURE */
}

/* This function sets the Max Receive Buffer length in the MAC H/w */
INLINE void set_machw_max_rx_buffer_len(UWORD16 val)
{
    //chenq mod
    //rMAC_MAX_RX_BUFFER_LEN = convert_to_le(val);
    host_write_trout_reg( convert_to_le(val), 
                          (UWORD32)rMAC_MAX_RX_BUFFER_LEN );
}

#ifdef MAC_P2P

// 20120709 caisf add, merged ittiam mac v1.2 code

INLINE void set_machw_p2p_opp_ps(void)
{
    //r_P2P_CNTRL_REG |= REGBIT1;
    host_write_trout_reg(
    host_read_trout_reg( (UWORD32)r_P2P_CNTRL_REG ) 
        | REGBIT1,
    (UWORD32)r_P2P_CNTRL_REG );
	
}

INLINE void reset_machw_p2p_opp_ps(void)
{
    //r_P2P_CNTRL_REG &= ~REGBIT1;
    host_write_trout_reg(
    host_read_trout_reg( (UWORD32)r_P2P_CNTRL_REG ) 
        & (~REGBIT1),
    (UWORD32)r_P2P_CNTRL_REG );
}

/* This funtion sets the timeout value in MAC Hw for the case where the p2p */
/* client does not receive GO's beacon                                      */
INLINE void set_machw_p2p_rx_beacon_to(UWORD16 rx_beacon_to)
{
    //r_P2P_CNTRL_REG &= MASK_INVERSE(16, 12);
    //r_P2P_CNTRL_REG |= ((UWORD32)rx_beacon_to) << 12;
    host_write_trout_reg(
    host_read_trout_reg( (UWORD32)r_P2P_CNTRL_REG ) 
        & (MASK_INVERSE(16, 12)),
    (UWORD32)r_P2P_CNTRL_REG );
	
    host_write_trout_reg(
    host_read_trout_reg( (UWORD32)r_P2P_CNTRL_REG ) 
        | (((UWORD32)rx_beacon_to) << 12),
    (UWORD32)r_P2P_CNTRL_REG );
}

/* This funtion returns the timeout value stored in MAC Hw for the case */
/* where the p2p  lient does not receive GO's beacon                    */
INLINE UWORD16 get_machw_p2p_rx_beacon_to(void)
{
    //return((UWORD8 )(r_P2P_CNTRL_REG  &  MASK(16, 12)) >> 12);
    return((UWORD8 )(host_read_trout_reg( 
                        (UWORD32)r_P2P_CNTRL_REG)  &  MASK(16, 12)) >> 12);
}

/* This funtion sets the CTwindow in MAC H/w */
INLINE void set_machw_CTwindow(UWORD8 CTwindow)
{
    //chenq mod
    //rP2P_CNTRL_REG &= MASK_INVERSE(8, 4);
    //rP2P_CNTRL_REG |= ((UWORD32)CTwindow) << 4;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)r_P2P_CNTRL_REG ) 
            & (MASK_INVERSE(8, 4)) ,
        (UWORD32)r_P2P_CNTRL_REG );

    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)r_P2P_CNTRL_REG ) 
            | (((UWORD32)CTwindow) << 4) ,
        (UWORD32)r_P2P_CNTRL_REG );
	
	// 20120709 caisf add, merged ittiam mac v1.2 code
    if(0 != CTwindow)
        set_machw_p2p_opp_ps();
    else
        reset_machw_p2p_opp_ps();
}

/* This funtion returns the value of CTwindow stored in MAC H/w */
INLINE UWORD8 get_machw_CTwindow(void)
{
    //chenq mod
    //return((UWORD8 )(rP2P_CNTRL_REG  &  MASK(8, 4)) >> 4);
    return((UWORD8 )(host_read_trout_reg( 
                        (UWORD32)r_P2P_CNTRL_REG)  &  MASK(8, 4)) >> 4);
}

// 20120709 caisf add, merged ittiam mac v1.2 code
INLINE void set_machw_p2p_mode_client(void)
{
    //r_P2P_CNTRL_REG |= REGBIT2;
	host_write_trout_reg(
    host_read_trout_reg( (UWORD32)r_P2P_CNTRL_REG ) 
        | REGBIT2,
    (UWORD32)r_P2P_CNTRL_REG );
}

INLINE void set_machw_p2p_mode_go(void)
{
    //r_P2P_CNTRL_REG &= ~REGBIT2;
	host_write_trout_reg(
    host_read_trout_reg( (UWORD32)r_P2P_CNTRL_REG ) 
        & (~REGBIT2),
    (UWORD32)r_P2P_CNTRL_REG );
}

INLINE void set_machw_p2p_enable(void)
{
    //r_P2P_CNTRL_REG |= REGBIT0;
	host_write_trout_reg(
    host_read_trout_reg( (UWORD32)r_P2P_CNTRL_REG ) 
        | REGBIT0,
    (UWORD32)r_P2P_CNTRL_REG );
}

INLINE void set_machw_p2p_disable(void)
{
    //r_P2P_CNTRL_REG &= ~REGBIT0;
	host_write_trout_reg(
    host_read_trout_reg( (UWORD32)r_P2P_CNTRL_REG ) 
        & (~REGBIT0),
    (UWORD32)r_P2P_CNTRL_REG );
}

INLINE void set_machw_cancel_noa(void)
{
    //r_P2P_CNTRL_REG |= REGBIT14 + REGBIT2 + REGBIT0;
    //r_P2P_CNTRL_REG |= REGBIT28;
	host_write_trout_reg(
    host_read_trout_reg( (UWORD32)r_P2P_CNTRL_REG ) 
        | (REGBIT14 + REGBIT2 + REGBIT0),
    (UWORD32)r_P2P_CNTRL_REG );
	
	host_write_trout_reg(
    host_read_trout_reg( (UWORD32)r_P2P_CNTRL_REG ) 
        | (REGBIT28),
    (UWORD32)r_P2P_CNTRL_REG );
}

INLINE void reset_machw_cancel_noa(void)
{
    //r_P2P_CNTRL_REG &= ~REGBIT28;
	host_write_trout_reg(
    host_read_trout_reg( (UWORD32)r_P2P_CNTRL_REG ) 
        & (~REGBIT28),
    (UWORD32)r_P2P_CNTRL_REG );
}

/* This funtion sets the NOA duration, interval and start time of the first  */
/* NOA schedule in the MAC H/w                                               */
INLINE void set_machw_noa1_schedule_dscr(UWORD32 duration, UWORD32 interval,
                                   UWORD32 start_time)
{
    //chenq mod
    //r_P2P_NOA1_DURATION_REG   = convert_to_le(duration);
    //r_P2P_NOA1_INTERVAL_REG   = convert_to_le(interval);
    //r_P2P_NOA1_START_TIME_REG = convert_to_le(start_time);
    host_write_trout_reg( convert_to_le(duration), 
                          (UWORD32)r_P2P_NOA1_DURATION_REG );
    host_write_trout_reg( convert_to_le(interval), 
                          (UWORD32)r_P2P_NOA1_INTERVAL_REG );
    host_write_trout_reg( convert_to_le(start_time), 
                          (UWORD32)r_P2P_NOA1_START_TIME_REG );
}

/* This funtion sets the NOA duration, interval and start time of the second */
/* NOA schedule in the MAC H/w                                               */
INLINE void set_machw_noa2_schedule_dscr(UWORD32 duration, UWORD32 interval,
                                   UWORD32 start_time)
{
    //chenq mod
    //r_P2P_NOA2_DURATION_REG   = convert_to_le(duration);
    //r_P2P_NOA2_INTERVAL_REG   = convert_to_le(interval);
    //r_P2P_NOA2_START_TIME_REG = convert_to_le(start_time);
    host_write_trout_reg( convert_to_le(duration), 
                          (UWORD32)r_P2P_NOA2_DURATION_REG );
    host_write_trout_reg( convert_to_le(interval), 
                          (UWORD32)r_P2P_NOA2_INTERVAL_REG );
    host_write_trout_reg( convert_to_le(start_time), 
                          (UWORD32)r_P2P_NOA2_START_TIME_REG );
}

/* This funtion sets the count of the first NOA schedule in MAC H/w */
INLINE void set_machw_noa1_cnt(UWORD8 cnt)
{
    //chenq mod
    //r_P2P_NOA_CNT_STATUS_REG &= MASK_INVERSE(8, 0);
    //r_P2P_NOA_CNT_STATUS_REG |= cnt;
    
    host_write_trout_reg(
            host_read_trout_reg( (UWORD32)r_P2P_NOA_CNT_STATUS_REG ) 
                & (MASK_INVERSE(8, 0)) ,
            (UWORD32)r_P2P_NOA_CNT_STATUS_REG );

    
    host_write_trout_reg(
            host_read_trout_reg( (UWORD32)r_P2P_NOA_CNT_STATUS_REG ) 
                | (cnt) ,
            (UWORD32)r_P2P_NOA_CNT_STATUS_REG );
    
}

/* This funtion returns the value of count of the first NOA schedule stored  */
/* in MAC H/w                                                                */
INLINE UWORD8 get_machw_noa1_cnt(void)
{
    //chenq mod
    //return((UWORD8)(r_P2P_NOA_CNT_STATUS_REG & MASK(8, 0)));
    return((UWORD8)(host_read_trout_reg( 
                        (UWORD32)r_P2P_NOA_CNT_STATUS_REG) & MASK(8, 0)));
}

/* This funtion sets the count of the second NOA schedule in MAC H/w */
INLINE void set_machw_noa2_cnt(UWORD8 cnt)
{
    //chenq mod
    //r_P2P_NOA_CNT_STATUS_REG &= MASK_INVERSE(8, 8);
    //r_P2P_NOA_CNT_STATUS_REG |= cnt << 8;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)r_P2P_NOA_CNT_STATUS_REG ) 
            & (MASK_INVERSE(8, 8)) ,
        (UWORD32)r_P2P_NOA_CNT_STATUS_REG );

    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)r_P2P_NOA_CNT_STATUS_REG ) 
            | (cnt << 8) ,
        (UWORD32)r_P2P_NOA_CNT_STATUS_REG );
}

/* This funtion returns the value of count of the second NOA schedule stored */
/* in MAC H/w                                                                */
INLINE UWORD8 get_machw_noa2_cnt(void)
{
    //chenq mod
    //return((UWORD8)((r_P2P_NOA_CNT_STATUS_REG & MASK(8,8)) >> 8));
    return((UWORD8)((host_read_trout_reg( 
                        (UWORD32)r_P2P_NOA_CNT_STATUS_REG) & MASK(8,8)) >> 8));
}

/* This funtion sets the status of the first NOA schedule in MAC H/w */
INLINE void set_machw_noa1_status(UWORD8 status)
{
    //chenq mod
    //r_P2P_NOA_CNT_STATUS_REG &= MASK_INVERSE(2, 16);
    //r_P2P_NOA_CNT_STATUS_REG |= status << 16;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)r_P2P_NOA_CNT_STATUS_REG ) 
            & (MASK_INVERSE(2, 16)) ,
        (UWORD32)r_P2P_NOA_CNT_STATUS_REG );

    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)r_P2P_NOA_CNT_STATUS_REG ) 
            | (status << 16) ,
        (UWORD32)r_P2P_NOA_CNT_STATUS_REG );
}

/* This funtion returns the status of the first NOA schedule */
INLINE UWORD8 get_machw_noa1_status(void)
{
    //chenq mod
    //return((UWORD8)((r_P2P_NOA_CNT_STATUS_REG & MASK(2,16)) >> 16));
    return((UWORD8)((host_read_trout_reg( 
                        (UWORD32)r_P2P_NOA_CNT_STATUS_REG) & MASK(2,16)) >> 16));
}

/* This funtion sets the status of the second NOA schedule in MAC H/w */
INLINE void set_machw_noa2_status(UWORD8 status)
{
    //chenq mod
    //r_P2P_NOA_CNT_STATUS_REG &= MASK_INVERSE(2, 18);
    //r_P2P_NOA_CNT_STATUS_REG |= status << 18;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)r_P2P_NOA_CNT_STATUS_REG ) 
            & (MASK_INVERSE(2, 18)) ,
        (UWORD32)r_P2P_NOA_CNT_STATUS_REG );

    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)r_P2P_NOA_CNT_STATUS_REG ) 
            | (status << 18) ,
        (UWORD32)r_P2P_NOA_CNT_STATUS_REG );
}

/* This funtion returns the status of the second NOA schedule */
INLINE UWORD8 get_machw_noa2_status(void)
{
    //chenq mod
    //return((UWORD8)((r_P2P_NOA_CNT_STATUS_REG & MASK(2,18)) >> 18));
    return((UWORD8)((host_read_trout_reg( 
                        (UWORD32)r_P2P_NOA_CNT_STATUS_REG) & MASK(2,18)) >> 18));
}

/* This funtion sets the status of any NOA schedule in MAC HW */
INLINE void set_machw_noa_status(UWORD8 idx, UWORD8 status)
{
    if(0 == idx)
    {
        set_machw_noa1_status(status);
    }
    else
    {
        set_machw_noa2_status(status);
    }
}

/* This funtion sets the NOA duration, interval and start time of the second */
/* NOA schedule in the MAC H/w                                               */
INLINE void set_machw_noa_sched_dscr(UWORD8 idx, UWORD32 duration,
                                     UWORD32 interval, UWORD8 cnt,
                                     UWORD32 start_time)
{
    if(0 == idx)
    {
        set_machw_noa1_schedule_dscr(duration, interval, start_time);
        set_machw_noa1_cnt(cnt);
    }
    else
    {
        set_machw_noa2_schedule_dscr(duration, interval, start_time);
        set_machw_noa2_cnt(cnt);
    }
}

// 20120709 caisf add, merged ittiam mac v1.2 code
INLINE void set_machw_p2p_eoa_offset(UWORD16 oi)
{
    //r_P2P_EOA_OFFSET =  convert_to_le((UWORD32)oi);
    host_write_trout_reg(convert_to_le((UWORD32)oi), (UWORD32)r_P2P_EOA_OFFSET );
}

/* P2P Status Register Details                                               */
/* NOA schedule-1 absent period is in progress(1) (BIT0)                     */
/* NOA schedule-1 absent period is periodic(0)/non-periodic(1) (BIT1)        */
/* NOA schedule-2 absent period is in progress(1) (BIT2)                     */
/* NOA schedule-2 absent period is periodic(0)/non-periodic(1) (BIT3)        */
/* CTWindow is in progress (1) (BIT4)                                        */

/* This function checks if NOA schedule is in progress */
INLINE BOOL_T is_noa_sched_in_progress(UWORD8 noa_sched_idx)
{
    BOOL_T ret_val = BFALSE;

    if(0 == noa_sched_idx) /* Schedule 1 */
    {
        //if((r_P2P_STATUS_REG & REGBIT0) == REGBIT0)
		if(((host_read_trout_reg( (UWORD32)r_P2P_STATUS_REG) ) & REGBIT0) == REGBIT0)
            ret_val = BTRUE;
    }
    else if(1 == noa_sched_idx) /* Schedule 2 */
    {
        //if((r_P2P_STATUS_REG & REGBIT2) == REGBIT2)
		if(((host_read_trout_reg( (UWORD32)r_P2P_STATUS_REG) ) & REGBIT2) == REGBIT2)
            ret_val = BTRUE;
    }

    return ret_val;
}

/* This function checks if NOA schedule is in progress */
INLINE BOOL_T is_ct_window_in_progress(void)
{
    BOOL_T ret_val = BFALSE;

    //if((r_P2P_STATUS_REG & REGBIT4) == REGBIT4)
	if(((host_read_trout_reg( (UWORD32)r_P2P_STATUS_REG) ) & REGBIT4) == REGBIT4)
        ret_val = BTRUE;

    return ret_val;
}

/* This function checks if NOA schedule is non-periodic */
INLINE BOOL_T is_noa_sched_non_periodic(UWORD8 noa_sched_idx)
{
    BOOL_T ret_val = BFALSE;

    if(0 == noa_sched_idx) /* Schedule 1 */
    {
        //if((r_P2P_STATUS_REG & REGBIT1) == REGBIT1)
		if(((host_read_trout_reg( (UWORD32)r_P2P_STATUS_REG) ) & REGBIT1) == REGBIT1)
            ret_val = BTRUE;
    }
    else if(1 == noa_sched_idx) /* Schedule 2 */
    {
        //if((r_P2P_STATUS_REG & REGBIT3) == REGBIT3)
		if(((host_read_trout_reg( (UWORD32)r_P2P_STATUS_REG) ) & REGBIT3) == REGBIT3)
            ret_val = BTRUE;
    }

    return ret_val;
}

#endif /* MAC_P2P */

#ifdef MWLAN
/* This function writes the supplied values to the S/w debug register */
INLINE void write_mac_sw_dbg_reg(UWORD16 sw_debug)
{
    //chenq mod
    //rSW_HANG_DBG_REG = sw_debug;
    //host_write_trout_reg( sw_debug, (UWORD32)rSW_HANG_DBG_REG ); //HUgh
}

/* This function writes the supplied value to the S/w debug register and */
/* return the last value.                                                */
INLINE UWORD16 read_write_mac_sw_dbg_reg(UWORD16 sw_debug)
{
        //chenq mod
        //UWORD16 temp = (UWORD16)rSW_HANG_DBG_REG;
        //UWORD16 temp = (UWORD16)host_read_trout_reg( (UWORD32)rSW_HANG_DBG_REG); ////HUgh

        //rSW_HANG_DBG_REG = sw_debug;
        //host_write_trout_reg( sw_debug, (UWORD32)rSW_HANG_DBG_REG ); //HUgh

        return 0;  //temp;  //HUgh
}

/* This function resets all FC Assertion Registers */
INLINE void reset_machw_all_fc_asserts(void)
{
/*  //Hugh
//chenq mod
    //volatile UWORD32 *reg_addr = FC_ASSERT_COUNT_REG_BASE;
    UWORD32 reg_addr = FC_ASSERT_COUNT_REG_BASE;
    UWORD32 regnum = 0;

    for(regnum = 0; regnum < NUM_FC_ASSERT_COUNT_REG; regnum++)
    {
        // *reg_addr = 0;
        host_write_trout_reg( 0, (UWORD32)reg_addr );
        reg_addr++;
    }
    */
}

/* This function returns the value of the FC Assertion of the required */
/* regsiter number.                                                    */
INLINE UWORD32 get_machw_fc_assert_value(UWORD8 regnum)
{
 /* //Hugh
    //chenq mod
    //volatile UWORD32 *reg_addr = FC_ASSERT_COUNT_REG_BASE;
    UWORD32 reg_addr = FC_ASSERT_COUNT_REG_BASE;

    if(regnum > NUM_FC_ASSERT_COUNT_REG)
        return 0;

    //reg_addr += regnum;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)reg_addr ) + (regnum) ,
        (UWORD32)reg_addr );

    //return convert_to_le(*reg_addr);
    return convert_to_le( host_read_trout_reg( (UWORD32)reg_addr) );
    */

    return 0;
}

// 20120709 caisf add, merged ittiam mac v1.2 code
INLINE void wait_machw_suspend_state(void)
{
    UWORD32 cnt = 0;
    /* Wait till MAC H/w enters in suspend state */
    //while((rMAC_PA_STAT & REGBIT6) == 0)
	while(((host_read_trout_reg( (UWORD32)rMAC_PA_STAT) ) & REGBIT6) == 0)
    {
        if(++cnt > 1000)
            break;
    }
}


extern void get_txrx_count(UWORD32 *tx_ok,
                       UWORD32 *tx_fail,
                       UWORD32 *rx_ok,
                       UWORD32 *rx_fail);
extern void tx_complete_isr(void);
#endif /* MWLAN */
#endif /* MH_H */
