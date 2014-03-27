/*******************************************************************************
* Copyright (c) 2011-2012,Spreadtrum Corporation
* All rights reserved.
* Filename: ittiam_mac\Common\trout_share_mem.h
* Abstract: 
* 
* Version: 1.0
* Revison Log: 
* 	2012/6/20, Chengwg: Create this file.
* CVS Log: 
* 	$Id$
*******************************************************************************/

#ifndef __TROUT_SHARE_MEM_H
#define __TROUT_SHARE_MEM_H

/*        trout wifi share memory layout      */
/* +----------------------------------------+ */
/* |        Beacon memory(512byte)          | */
/* +----------------------------------------+ */
/* |      rx high priority memory(5)        | */
/* +----------------------------------------+ */
/* |   rx normal priority memory(dynamic)   | */
/* +----------------------------------------+ */
/* |tx memory(dynamic, depend on normal rxq)| */
/* +----------------------------------------+ */
/* | coex slot info memory(8byte,dynamic)   | */
/* +----------------------------------------+ */
/* |   coex cts null data memory(200byte)   | */
/* +----------------------------------------+ */
/* |   coex ps null data memory(200byte)    | */
/* +----------------------------------------+ */
/* |    trout share memory cfg(128byte)     | */
/* +----------------------------------------+ */

/* Requirement is to support both old and new receive descriptor formats.    */
/* 1 new word (WORD4) is added in the new receive descriptor. All other      */
/* fields are same as before. For this an extra word offset constant is      */
/* defined. This takes care of accessing the appropriate word in the receive */
/* descriptor based on old or new format.                                    */
/* To support the old receive descriptor format the extra word offset is     */
/* defined as 0 while to support the new one it is defined as 1. For all     */
/* fields in word 4 and above, this extra word offset is added to make it    */
/* work for the receive descriptor in use.                                   */
/* The length in bytes and 32-bit words are also appropriately defined.      */

#ifdef OLD_RX_DSCR_FORMAT_v2_2
#define RX_DSCR_EXTRA_WORD_OFFSET   0
#define RX_DSCR_LEN                 36 /* 9 * 4 Bytes */
#define RX_DSCR_NUM_WORDS           9
#else /* OLD_RX_DSCR_FORMAT_v2_2 */
#define RX_DSCR_EXTRA_WORD_OFFSET   1
//#define RX_DSCR_LEN               40 	   /* 10 * 4 Bytes */
#define RX_DSCR_LEN                 (40+4) /* reserve 4byte used for trout rx mem management(chengwg) */

#define RX_DSCR_NUM_WORDS           ((RX_DSCR_LEN)/sizeof(UWORD32))
#endif /* OLD_RX_DSCR_FORMAT_v2_2 */

/* Number of buffers in the Normal and High Priority Rx queues */
#ifdef HOST_LOOP_BACK_MODE

#define NUM_RX_BUFFS                  1
#define NUM_HIPR_RX_BUFFS             1

#else  /* HOST_LOOP_BACK_MODE */

#ifdef MWLAN
    #define NUM_RX_BUFFS            22
    #define NUM_HIPR_RX_BUFFS       5
#endif /* MWLAN */

#ifdef GENERIC_PLATFORM
    #define NUM_RX_BUFFS              32
    #define NUM_HIPR_RX_BUFFS         5
#endif /* GENERIC_PLATFORM */

#endif /* HOST_LOOP_BACK_MODE */


#define TX_SHARERAM_SLOTS    2

/* used for dv envoriment */
#ifndef NUM_RX_BUFFS
#define NUM_RX_BUFFS              32
#endif

#ifndef NUM_HIPR_RX_BUFFS
#define NUM_HIPR_RX_BUFFS         5
#endif

//zhuyg modify for npi test
#ifdef TROUT_WIFI_NPI
#define CW_TX_BEGIN 0
#define CW_TX_SIZE 0x100
#define SHARE_MEM_BEGIN CW_TX_BEGIN+CW_TX_SIZE
#else
#define SHARE_MEM_BEGIN		4U
#endif

//#define SHARE_MEM_SIZE	(((20U * 1024U) << 2) - SHARE_MEM_BEGIN)	//80kb share ram.
#define SHARE_MEM_SIZE		((144U * 1024U) - SHARE_MEM_BEGIN)	//144kb share ram.
#define SHARE_MEM_END		(SHARE_MEM_BEGIN + SHARE_MEM_SIZE)

#define RX_PACKET_SIZE		1600U
#define RX_Q_SIZE			(RX_DSCR_LEN + RX_PACKET_SIZE)

/////////////////////////////////////////////////////////////////////
#define TX_DSCR_LEN         144 /* 35 * 4 Bytes + host descr addr */

#define BEACON_MEM_BEGIN	SHARE_MEM_BEGIN
#define BEACON_MEM_SIZE		512
#define BEACON_MEM_END		(BEACON_MEM_BEGIN + BEACON_MEM_SIZE)

#define HIGH_RX_MEM_BEGIN	BEACON_MEM_END
#define HIGH_RX_MEM_SIZE	(RX_Q_SIZE * NUM_HIPR_RX_BUFFS)
#define HIGH_RX_MEM_END		(HIGH_RX_MEM_BEGIN + HIGH_RX_MEM_SIZE)

#define NORMAL_RX_MEM_BEGIN	HIGH_RX_MEM_END

#define DFT_NUM_RX_BUFFS	30	//default normal rxq number.
#define MAX_NUM_RX_BUFFS	79	//in this case(144kb), tx ram is minimum, about 8kb.
#define MIN_NUM_RX_BUFFS	16	//rx free state.

#define DFT_NORMAL_RX_SIZE	(RX_Q_SIZE * DFT_NUM_RX_BUFFS)

#define MIN_TX_MEM_SIZE		(8U * 1024)
#define MAX_RX_MEM_SIZE		(SHARE_MEM_END - NORMAL_RX_MEM_BEGIN - MIN_TX_MEM_SIZE)

#define COEX_SLOT_INFO_SIZE		(4 * TX_SHARERAM_SLOTS)

/* Reserved 128byte used for save trout share memory config info, this will be */
/* used by Arm7                                                                */
#define TROUT_MEM_CFG_SIZE		128
#define TROUT_MEM_CFG_BEGIN		(SHARE_MEM_END - TROUT_MEM_CFG_SIZE)
#define TROUT_MEM_MAGIC			0x756F7274

#define COEX_PS_NULL_DATA_SIZE	200	//include tx dscr & pkt content.
#define COEX_PS_NULL_DATA_BEGIN		(TROUT_MEM_CFG_BEGIN - COEX_PS_NULL_DATA_SIZE)

#define COEX_SELF_CTS_NULL_DATA_SIZE	200	//include tx dscr & pkt content.
#define COEX_SELF_CTS_NULL_DATA_BEGIN	(COEX_PS_NULL_DATA_BEGIN - COEX_SELF_CTS_NULL_DATA_SIZE)

/* this is setting according to alloc_sb_buf(), the min host normal rx buufer is
 * 64kb, so the min trout normal rx pkt buffer size is: (64kb/HOST_RX_Q_NUM)x2. 
 */
#define MIN_RX_MEM_SIZE		((19 * RX_Q_SIZE) * 2)	//add by chengwg 2013.7.19.
/////////////////////////////////////////////////////////////////////


INLINE void trout_share_ram_show(void)
{
/*
#if 0
	TROUT_DBG4("-------------- trout share mem region -------------\n");
	TROUT_DBG4("trout share mem size:       0x%08X, range: 0x%08X ~ 0x%08X\n", 
					SHARE_MEM_SIZE, SHARE_MEM_BEGIN, SHARE_MEM_END);					
	TROUT_DBG4("trout wifi rx mem size:     0x%08X, range: 0x%08X ~ 0x%08X\n",
								(RX_MEM_END - RX_MEM_BEGIN), RX_MEM_BEGIN, RX_MEM_END);
	TROUT_DBG4("trout wifi tx mem size:     0x%08X, range: 0x%08X ~ 0x%08X\n",
								(TX_MEM_END - TX_MEM_BEGIN), TX_MEM_BEGIN, TX_MEM_END);								
	TROUT_DBG4("trout wifi beacon mem size: 0x%08X, range: 0x%08X ~ 0x%08X\n",
				(BEACON_MEM_END - BEACON_MEM_BEGIN), BEACON_MEM_BEGIN, BEACON_MEM_END);
#ifdef MAC_P2P
	TROUT_DBG4("trout wifi P2P cfg mem size: 0x%08X, range: 0x%08X ~ 0x%08X\n",
				(CFG_CONF_MEM_END - CFG_CONF_MEM_BEGIN), CFG_CONF_MEM_BEGIN, CFG_CONF_MEM_END);
#endif
	TROUT_DBG4("----------------------------------------------------\n");
#else
	TROUT_DBG4("-------------- trout share mem region -------------\n");
	TROUT_DBG4("Trout share mem size:       0x%08X, range: 0x%08X ~ 0x%08X\n", 
					SHARE_MEM_SIZE, SHARE_MEM_BEGIN, SHARE_MEM_END);
	TROUT_DBG4("Beacon mem size:     0x%08X, range: 0x%08X ~ 0x%08X\n",
								BEACON_MEM_SIZE, BEACON_MEM_BEGIN, BEACON_MEM_END);
	TROUT_DBG4("High Rx mem size:     0x%08X, range: 0x%08X ~ 0x%08X\n",
								HIGH_RX_MEM_SIZE, HIGH_RX_MEM_BEGIN, HIGH_RX_MEM_END);
#endif
*/
}


#endif	/* __TROUT_SHARE_MEM_H */
