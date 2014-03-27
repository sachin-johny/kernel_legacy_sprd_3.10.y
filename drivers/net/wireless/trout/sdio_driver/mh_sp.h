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

#ifndef MH_H_SP
#define MH_H_SP

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
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


#define VIRT_ITM_MWLANBAR (0x2000U << 2)  //add by Hugh
#define PLD0_ADDR           (VIRT_ITM_MWLANBAR)
#define PA_BASE             (PLD0_ADDR + 0x00000000)    // (0x2000U << 2)+0x0

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
#define MAC_RXSTART_DELAY_REG_INIT_VALUE     0x221BFFFF  //mengyuan.du mod 2013-04-24,fix 11b ack
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

#endif /* MH_H */
