#ifndef _TROUT_REG_DEF_H
#define _TROUT_REG_DEF_H

#define PA_BASE                       (0x00008000)

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/
#define VIRT_IMMRBAR 0       //add by Hugh
#define VIRT_ITM_MWLANBAR (0x2000U << 2)  //add by Hugh
#define TROUT_COMM_REG    (0x4000U << 2)  //add by Hugh


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
#define rCOMM_INT_STAT                (TROUT_COMM_REG + (0x0004<<2))
#define rCOMM_INT_MASK                (TROUT_COMM_REG + (0x0002<<2))
#define rCOMM_INT_CLEAR               (TROUT_COMM_REG + (0x0000<<2))
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


#endif
