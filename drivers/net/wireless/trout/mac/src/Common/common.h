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
/*  File Name         : common.h                                             */
/*                                                                           */
/*  Description       : This file contains all the common definitions for    */
/*                      MAC implementation.                                  */
/*                                                                           */
/*  List of Functions : print_log_debug_level_1                              */
/*                      print_log_debug_level_2                              */
/*                      mac_addr_cmp                                         */
/*                      mac_addr_cpy                                         */
/*                      ip_addr_cpy                                          */
/*                      memcpy32                                             */
/*                      memcpy_le                                            */
/*                      memcpy32_le                                          */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef COMMON_H
#define COMMON_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#ifdef OS_LINUX_CSL_TYPE
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/kallsyms.h>
#else /* OS_LINUX_CSL_TYPE */
#include <string.h>
#endif /* OS_LINUX_CSL_TYPE */

#include "itypes.h"
#include "iparams.h"

#include "trout_trace.h"

/*****************************************************************************/
/* Defines                                                                   */
/*****************************************************************************/

/* By default, the system is assumed to be Little Endian */
#ifdef GENERIC_PLATFORM
#define LITTLE_ENDIAN
#endif /* GENERIC_PLATFORM */

/* Define endianness depending on the platform type */
#ifdef MWLAN
#define LITTLE_ENDIAN
#endif /* MWLAN */

/* In case of BIG_ENDIAN the set/get descriptor Macros will not work. Hence  */
/* the flag DSCR_MACROS_NOT_DEFINED must be defined.                         */
#ifdef BIG_ENDIAN
#define DSCR_MACROS_NOT_DEFINED
#endif /* BIG_ENDIAN */

/* Define flags for test purpose */
#define HANDLE_ERROR_INTR
//#define DSCR_MACROS_NOT_DEFINED

#ifdef DEBUG_MODE
#undef  MEM_DEBUG_MODE
#define MEM_DEBUG_MODE
#endif /* DEBUG_MODE */

#ifdef MEM_DEBUG_MODE
//#define MEM_STRUCT_SIZES_INIT
#endif /* MEM_DEBUG_MODE */

//#define AUTORATE_PING

#ifdef IBSS_BSS_STATION_MODE
//#define AUTO_RX_SENSITIVITY
#endif /* IBSS_BSS_STATION_MODE */

//#define TX_ABORT_FEATURE
// 20120830 caisf masked, merged ittiam mac v1.3 code
#if 0
/* SUPP_11I is the new name for INTERNAL_11I */
#ifdef INTERNAL_11I
#define SUPP_11I
#endif /* INTERNAL_11I */
#endif

#ifdef BSS_ACCESS_POINT_MODE
#ifdef MAC_802_1X
#ifdef MWLAN
#define ENABLE_AP_1X_LINUX_PROCESSING
#endif /* MWLAN */
#endif /* MAC_802_1X */

#define ENABLE_PS_PKT_FLUSH
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef BSS_ACCESS_POINT_MODE
#ifdef MAC_802_11I
/* Following flags are needed for 11i AP Mode */
#ifndef AUTH_11I
    #define AUTH_11I
#endif /* AUTH_11I */
#endif /* MAC_802_11I */
#define ENABLE_PS_PKT_FLUSH

#ifdef MAC_802_1X
#ifdef MWLAN
#define ENABLE_AP_1X_LINUX_PROCESSING
#endif /* MWLAN */
#endif /* MAC_802_1X */

#endif /* BSS_ACCESS_POINT_MODE */

#ifdef IBSS_BSS_STATION_MODE
#ifdef MAC_802_11I
/* Following flags are needed for 11i Station Mode */

#ifdef ETHERNET_HOST
/* 11i Supplicant code is needed for Ethernet host */
#ifndef SUPP_11I
    #define SUPP_11I
#endif /* SUPP_11I */
#endif /* ETHERNET_HOST */

#endif /* MAC_802_11I */
#endif /* IBSS_BSS_STATION_MODE */

#ifndef MAC_802_11I
#ifdef SUPP_11I
    #undef SUPP_11I
#endif /* SUPP_11I */
#endif /*   MAC_802_11I   */

#ifdef SUPP_11I
    #ifndef UTILS_11I
       #define UTILS_11I
    #endif /* UTILS_11I */
#endif /* SUPP_11I */

#ifdef AUTH_11I
    #ifndef UTILS_11I
       #define UTILS_11I
    #endif /* UTILS_11I */
#endif /* AUTH_11I */

/* 20/40 valid only in 11n Mode */
#ifndef MAC_802_11N
    #undef MAC_802_11N_2040
#endif /* MAC_802_11N */

/* Use Protection when Operating mode is 3. */
//#define HT_OP_MODE_3_PROT

#ifdef INT_WPS_SUPP
#ifndef SUPP_11I
    #define SUPP_11I
#endif /* SUPP_11I */
#undef DEVICE_SPECIFIC_CONFIG
#define DEVICE_SPECIFIC_CONFIG
#endif /* INT_WPS_SUPP */

#ifdef ENABLE_P2P_PS_TESTING
/* P2P PS test support flags */
#ifdef BSS_ACCESS_POINT_MODE
#define ENABLE_P2P_GO_TEST_SUPPORT
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef IBSS_BSS_STATION_MODE
#define ENABLE_OVERWRITE_SUBTYPE
#endif /* IBSS_BSS_STATION_MODE */
#endif /* ENABLE_P2P_PS_TESTING */

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define MAC_ADDRESS_LEN         6

/* Bit Values */
#define BIT31                   ((UWORD32)(1 << 31))
#define BIT30                   ((UWORD32)(1 << 30))
#define BIT29                   ((UWORD32)(1 << 29))
#define BIT28                   ((UWORD32)(1 << 28))
#define BIT27                   ((UWORD32)(1 << 27))
#define BIT26                   ((UWORD32)(1 << 26))
#define BIT25                   ((UWORD32)(1 << 25))
#define BIT24                   ((UWORD32)(1 << 24))
#define BIT23                   ((UWORD32)(1 << 23))
#define BIT22                   ((UWORD32)(1 << 22))
#define BIT21                   ((UWORD32)(1 << 21))
#define BIT20                   ((UWORD32)(1 << 20))
#define BIT19                   ((UWORD32)(1 << 19))
#define BIT18                   ((UWORD32)(1 << 18))
#define BIT17                   ((UWORD32)(1 << 17))
#define BIT16                   ((UWORD32)(1 << 16))
#define BIT15                   ((UWORD32)(1 << 15))
#define BIT14                   ((UWORD32)(1 << 14))
#define BIT13                   ((UWORD32)(1 << 13))
#define BIT12                   ((UWORD32)(1 << 12))
#define BIT11                   ((UWORD32)(1 << 11))
#define BIT10                   ((UWORD32)(1 << 10))
#define BIT9                    ((UWORD32)(1 << 9))
#define BIT8                    ((UWORD32)(1 << 8))
#define BIT7                    ((UWORD32)(1 << 7))
#define BIT6                    ((UWORD32)(1 << 6))
#define BIT5                    ((UWORD32)(1 << 5))
#define BIT4                    ((UWORD32)(1 << 4))
#define BIT3                    ((UWORD32)(1 << 3))
#define BIT2                    ((UWORD32)(1 << 2))
#define BIT1                    ((UWORD32)(1 << 1))
#define BIT0                    ((UWORD32)(1 << 0))
#define ALL                     0xFFFF

#define WORD_OFFSET_0           0
#define WORD_OFFSET_1           1
#define WORD_OFFSET_2           2
#define WORD_OFFSET_3           3
#define WORD_OFFSET_4           4
#define WORD_OFFSET_5           5
#define WORD_OFFSET_6           6
#define WORD_OFFSET_7           7
#define WORD_OFFSET_8           8
#define WORD_OFFSET_9           9
#define WORD_OFFSET_10          10
#define WORD_OFFSET_11          11
#define WORD_OFFSET_12          12
#define WORD_OFFSET_20          20
#define WORD_OFFSET_21          21
#define WORD_OFFSET_22          22
#define WORD_OFFSET_23          23
#define WORD_OFFSET_24          24
#define WORD_OFFSET_25          25
#define WORD_OFFSET_26          26
#define WORD_OFFSET_27          27
#define WORD_OFFSET_28          28
#define WORD_OFFSET_29          29
#define WORD_OFFSET_30          30
#define WORD_OFFSET_31          31
#define WORD_OFFSET_32          32
#define WORD_OFFSET_33          33
#define WORD_OFFSET_34          34

#define WORD_OFFSET_35          35  //add by Hugh


#define FIELD_WIDTH_2           2
#define FIELD_WIDTH_3           3
#define FIELD_WIDTH_4           4
#define FIELD_WIDTH_8           8
#define FIELD_WIDTH_12          12
#define FIELD_WIDTH_14          14
#define FIELD_WIDTH_16          16
#define FIELD_WIDTH_24          24
#define FIELD_WIDTH_32          32

#ifdef LITTLE_ENDIAN

#define REGBIT31  BIT31
#define REGBIT30  BIT30
#define REGBIT29  BIT29
#define REGBIT28  BIT28
#define REGBIT27  BIT27
#define REGBIT26  BIT26
#define REGBIT25  BIT25
#define REGBIT24  BIT24

#define REGBIT23  BIT23
#define REGBIT22  BIT22
#define REGBIT21  BIT21
#define REGBIT20  BIT20
#define REGBIT19  BIT19
#define REGBIT18  BIT18
#define REGBIT17  BIT17
#define REGBIT16  BIT16

#define REGBIT15  BIT15
#define REGBIT14  BIT14
#define REGBIT13  BIT13
#define REGBIT12  BIT12
#define REGBIT11  BIT11
#define REGBIT10  BIT10
#define REGBIT9   BIT9
#define REGBIT8   BIT8

#define REGBIT7   BIT7
#define REGBIT6   BIT6
#define REGBIT5   BIT5
#define REGBIT4   BIT4
#define REGBIT3   BIT3
#define REGBIT2   BIT2
#define REGBIT1   BIT1
#define REGBIT0   BIT0

#endif /* LITTLE_ENDIAN */

#ifdef BIG_ENDIAN

#define REGBIT31  BIT7
#define REGBIT30  BIT6
#define REGBIT29  BIT5
#define REGBIT28  BIT4
#define REGBIT27  BIT3
#define REGBIT26  BIT2
#define REGBIT25  BIT1
#define REGBIT24  BIT0

#define REGBIT23  BIT15
#define REGBIT22  BIT14
#define REGBIT21  BIT13
#define REGBIT20  BIT12
#define REGBIT19  BIT11
#define REGBIT18  BIT10
#define REGBIT17  BIT9
#define REGBIT16  BIT8

#define REGBIT15  BIT23
#define REGBIT14  BIT22
#define REGBIT13  BIT21
#define REGBIT12  BIT20
#define REGBIT11  BIT19
#define REGBIT10  BIT18
#define REGBIT9   BIT17
#define REGBIT8   BIT16

#define REGBIT7   BIT31
#define REGBIT6   BIT30
#define REGBIT5   BIT29
#define REGBIT4   BIT28
#define REGBIT3   BIT27
#define REGBIT2   BIT26
#define REGBIT1   BIT25
#define REGBIT0   BIT24

#endif /* BIG_ENDIAN */

#define MAX_STRING_LEN 256
#define MAX_CFG_LEN    MAX_STRING_LEN

#ifdef GENERIC_PLATFORM
#define DEFAULT_CALIB_COUNT  38
#endif /* GENERIC_PLATFORM */

#ifdef MWLAN
#define DEFAULT_CALIB_COUNT  38
#endif /* MWLAN */

/* Maximum word alignment buffer */
#define MAX_WORD_ALIGNMENT_BUFFER 3

/* Maximum 32-bit bitmap size */
#define MAX_BMAP_SZ 32

/*****************************************************************************/
/* Function Macros                                                           */
/*****************************************************************************/
/* check if one bit is set in the variable */
#define CHECK_BIT(var,pos) ((var) & (pos))

/* Macros to compute minimum and maximum */
#define MAX(A, B) (((A) > (B))? (A) : (B))
#define MIN(A, B) (((A) < (B))? (A) : (B))

/* Macros to compute absolute value */
#define ABS(A)    ((A < 0) ? -(A) : (A))

#define HEX2ASCII(x)  (((x) > 9) ? ('A'+ (x)-10):('0'+ (x)))

/* Macros to make 16-bit and 32-bit words */
#define MAKE_WORD16(lsb, msb) ((((UWORD16)(msb) << 8) & 0xFF00) | (lsb))
#define MAKE_WORD32(lsw, msw) ((((UWORD32)(msw) << 16) & 0xFFFF0000) | (lsw))

/* Macros to swap byte order */
#define SWAP_BYTE_ORDER_WORD(val) ((((val) & 0x000000FF) << 24) + \
                                   (((val) & 0x0000FF00) << 8)  + \
                                   (((val) & 0x00FF0000) >> 8)   + \
                                   (((val) & 0xFF000000) >> 24))
#define SWAP_BYTE_ORDER_SHORT(val) ((((val) & 0x00FF) << 8) + \
                                   (((val) & 0xFF00) >> 8))


/* Macros to set/get descriptor fields */
#define SHIFT_DSCR_OFFSET(offst, wdth)  (32 - (wdth) - (offst))
#define DSCR_MASK(offst, wdth)          (((wdth) < 32) ? \
        ((WORD32)((1 << (wdth)) - 1) << (SHIFT_DSCR_OFFSET(offst, wdth))) : \
        (0xFFFFFFFF))
		
/* when CPU do write and read, needn't voliate by zhao */		
#define GET_DSCR(offst, wdth, ptr)      (((*(WORD32 *) ptr) & \
        (DSCR_MASK(offst,wdth))) >> (SHIFT_DSCR_OFFSET(offst, wdth)))
#define SET_DSCR(offst, wdth, ptr, val) (*(WORD32 *)ptr = ((((WORD32)(val) << \
        (SHIFT_DSCR_OFFSET(offst,wdth))) & (DSCR_MASK(offst,wdth))) | \
        ((*(WORD32 *) (ptr)) & (~(DSCR_MASK(offst, wdth))))))

#define WORD_ALIGN(val) (((val) & 0x03) ? ((val) + 4 - ((val) & 0x03)) : (val))


#define DSCR_TSF_SUB(tsf1, tsf2) (((tsf1) - (tsf2)) & 0xFFFF)

#define PUT_U16_BE(buff, val)   buff[0] = (UWORD8)(((UWORD16)val >> 8)&0xFF);\
                                buff[1] = (UWORD8)((UWORD16)val&0xFF);
#define GET_U16_BE(buff)  (UWORD16)(((UWORD16)(buff)[0]<<8)+(UWORD16)(buff)[1])

/* Macros for printing log and debug messages */

#ifdef OS_LINUX_CSL_TYPE
#define PRINTK(args...)  printk(args)
#else  /* OS_LINUX_CSL_TYPE */
#define PRINTK(args...)
#endif /* OS_LINUX_CSL_TYPE */

#define DEBUG_PRINT_LEVEL_1      1
#define DEBUG_PRINT_LEVEL_2      2
#define MAX_DEBUG_PRINT_LEVEL    DEBUG_PRINT_LEVEL_2


#define TROUT_PRINT_STACK  if(1){ PRINT_TROUT_STACK_FN(__func__, __LINE__);}
INLINE void PRINT_TROUT_STACK_FN(const char *func_name, UWORD32 line)
{
    printk("%s-%d: ", func_name, line);    
#ifdef DV_SIM
    print_symbol("at %s", (unsigned long)__builtin_return_address(0));
    print_symbol(" <= %s", (unsigned long)__builtin_return_address(1));
    print_symbol(" <= %s\n", (unsigned long)__builtin_return_address(2));
#else
    print_symbol("at %s\n", (unsigned long)__builtin_return_address(0));
#endif
}

INLINE void hex_dump(UWORD8 *info, UWORD8 *str, UWORD32 len)
{
#ifdef DEBUG_MODE
	UWORD32  i = 0;
#endif

    if(str == NULL || len == 0)
        return;
    
#ifdef DEBUG_MODE
	printk("dump %s, len: %d; data:\n",info,len);
	for(i = 0; i<len; i++)
	{
		if(((UWORD8 *)str+i)==NULL)
			break;
		printk("%02x ",*((UWORD8 *)str+i));
		if((i+1)%16 == 0)
			printk("\n");
	}
	printk("\n");
#endif
}

/*
#undef TROUT_PRI_DEBUG
#ifdef  TROUT_PRI_DEBUG
#define PRINT_TROUT_STACK  if(1){ PRINT_TROUT_STACK_FN(__FUNCTION__,__LINE__); }
#define TROUT_PRINT(format, arg...)  if(1){PRINTK("%s-%d -- "format"\n",__FUNCTION__,__LINE__,## arg);}
#else
#define PRINT_TROUT_STACK  
#define TROUT_PRINT(format, arg...)   do {} while (0)
#endif
*/

#ifdef DEBUG_MODE
#if 0
#define PRINTD(args...)  if(g_enable_debug_print >= DEBUG_PRINT_LEVEL_1)\
                         {\
                            PRINTK(args);\
                         }

#define PRINTD2(args...)  if(g_enable_debug_print >= DEBUG_PRINT_LEVEL_2)\
                         {\
                            PRINTK(args);\
                         }
#else
#define PRINTD(args...) 	printk(args)
#define PRINTD2(args...) 	//printk(args)
#endif

#define PRINTH(args...)  printh(args)
#define PRINTH_END()

#else  /* DEBUG_MODE */
#define PRINTD(args...)
#define PRINTD2(args...)
#define PRINTH(args...)
#define PRINTH_END()
#endif /* DEBUG_MODE */

#define PRINTS(string, args...)  sprintf(string, args)

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/
/* Status */
typedef enum {FAILURE = 0,
              SUCCESS = 1
} STATUS_T;

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

#ifdef DEBUG_MODE
typedef struct
{
    /* Interrupt related */
    UWORD32 imac;                 /* Number of mac interrupt                */
    UWORD32 itbtt;                /* Number of TBTT interrupt               */
    UWORD32 itxc;                 /* Number of TX complete interrupt        */
    UWORD32 irxc;                 /* Number of RX complete interrupt        */
    UWORD32 ihprxc;               /* Number of Hi-RQ complete interrupt     */
    UWORD32 ierr;                 /* Number of error interrupt              */
    UWORD32 ideauth;              /* Number of deauth interrupt             */
    UWORD32 icapend;              /* Number of CAP End interrupt            */

    /* Error interrupt counts */
    UWORD32 enpmsdu;
    UWORD32 erxqemp;
    UWORD32 ehprxqemp;
    UWORD32 etxsus1machang;
    UWORD32 etxsus1phyhang;
    UWORD32 etxsus3;
    UWORD32 ebus;
    UWORD32 ebwrsig;
    UWORD32 emsaddr;
    UWORD32 etxfifo;
    UWORD32 erxfifo;
    UWORD32 ehprxfifo;
    UWORD32 etxqempt;
    UWORD32 edmanoerr;
    UWORD32 etxcenr;
    UWORD32 erxcenr;
    UWORD32 esgaf;
    UWORD32 eother;
    UWORD32 ewdtofcsfailed;
    UWORD32 ewdtofcspassed;

    /* LAN Interrupt related */
    UWORD32 li;
    UWORD32 litxe;
    UWORD32 litx;
    UWORD32 lirxo;
    UWORD32 lialloc;
    UWORD32 lirrd;
    UWORD32 lirrdexcsize;
    UWORD32 liexc;

    /* PCMCIA Interrupt related */
    UWORD32 plenexc;
    UWORD32 pidis;
    UWORD32 pienab;
    UWORD32 pi;
    UWORD32 pitxc;
    UWORD32 pirxc;
    UWORD32 prxderr;
    UWORD32 pitxnomatch;

    /* System DMA related */
    UWORD32 sdmacntregerr;
    UWORD32 sdmaerr;
    UWORD32 sdmarxtrig;
    UWORD32 sdmatxtrig;
    UWORD32 sdmapktskip;
    UWORD32 sdmai;
    UWORD32 sdmarxcnt;
    UWORD32 sdmatxcnt;
    UWORD32 sdmarxexccnt;
    UWORD32 sdmatxexccnt;
    UWORD32 sdmaprogerr;
    UWORD32 sdmaskip;

    /* Event processing related */
    UWORD32 peadd[7];
    UWORD32 pedel[7];
    UWORD32 pemax[7];
    UWORD32 pehrxml;
    UWORD32 pehrxms;
    UWORD32 pewrx;
    UWORD32 pewrxfcs;
    UWORD32 pewrxu;
    UWORD32 pewrxb;
    UWORD32 pewrxenc;
    UWORD32 pewrxnf;
    UWORD32 pewrxtmf;
    UWORD32 pewrxft;
    UWORD32 pewrxmf;
    UWORD32 pewrxkf;
    UWORD32 pewrxicvf;
    UWORD32 pewrxrply;
    UWORD32 pewrxwep40;
    UWORD32 pewrxtkip;
    UWORD32 pewrxccmp;
    UWORD32 pewrxwep104;
    UWORD32 pewrxunknown;
    UWORD32 pewrxwrongid;
    UWORD32 pewrxnew;
    UWORD32 pewrxdup;
    UWORD32 pewrxtkiprepf;
    UWORD32 pewrxccmprepf;
    UWORD32 pewrxtkipmicf;
    UWORD32 pewrxdip;
    UWORD32 pewrxdeferr;

    UWORD32 coalesce;
    UWORD32 pewmc;
    UWORD32 w2w;
    UWORD32 psbuff;
    UWORD32 psqlim;

    /* Process WLAN Rx exceptions */
    UWORD32 pwrx_unexp_newdscr;             /* rcvd invalid rx dscr count     */
    UWORD32 pwrx_unexp_end;
    UWORD32 pwrx_null_dscr;
    UWORD32 pwrx_notstart;
    UWORD32 pwrx_numexceed;
    UWORD32 rxfrmhdraddr4prsnt;
    UWORD32 rxfrmhdrbssidchkfail;           /* rcvd frame BSSID Match Fail    */
    UWORD32 rxfrmhdrprotverfail;            /* rcvd Protocol Version Match Fail */
    UWORD32 rxfrmhdraddrchkfail;
    UWORD32 rxunexpfrm;

    /* QMU related counts */
    UWORD32 qatxp;
    UWORD32 qdtxp;

    /* LAN related counts */
    UWORD32 lsp;
    UWORD32 lsphc;

    /* Host related counts */
    UWORD32 hifqfail;
    UWORD32 hifq;
    UWORD32 hifbusy;
    UWORD32 hifnotready;
    UWORD32 hifhtnull;
    UWORD32 hifalloc;
    UWORD32 hifcnfgnotready;
    UWORD32 hifcnfgbusy;
    UWORD32 hifcnfgq;
    UWORD32 hifcnfgqfail;
    UWORD32 hifcnfgqnull;
    UWORD32 hifinvldtype;
    UWORD32 hifnobuffc;
    UWORD32 hiftxlcfc;

    /* Bug fix count */
    UWORD32 bug;

    /* Event post exception counts */
    UWORD32 emiscexc;
    UWORD32 etxcexc;
    UWORD32 erxcexc;
    UWORD32 ehrxexc1;            /* tx pkt length exceeds maximum limit(1596) */
    UWORD32 peexc[7];
    UWORD32 hrxpeexc;
    UWORD32 rxcpeexc;
    UWORD32 txcpeexc;

    /* Queue exception counts */
    UWORD32 qaexc;
    UWORD32 qdexc;
    UWORD32 qarexc;
    UWORD32 qa[5];
    UWORD32 qfull[5];
    UWORD32 qn35exc[5];
    UWORD32 qn30exc[5];
    UWORD32 qn25exc[5];
    UWORD32 qn20exc[5];
    UWORD32 qn15exc[5];
    UWORD32 qn_no_sync[5];
    UWORD32 qa0exc[5];
    UWORD32 qn_no_sync_pnf[5];
    UWORD32 qinvalid[5];
    UWORD32 qrxa[2];
    UWORD32 qrxd[2];
    UWORD32 qrx_nosync_del[2];
    UWORD32 qrx_unexp_end[2];
    UWORD32 qrx_nosync_nodel[2];

    /* Block ACK counts */
    UWORD32 bapendingtx;          /* Num of frames added to the pending Q */
    UWORD32 bapendingrx;
    UWORD32 bapendingrxhosttxd;
    UWORD32 babarrcvdigned;
    UWORD32 babasent;
    UWORD32 barxmiss;
    UWORD32 barxbuffail;
    UWORD32 baduprx;

    UWORD32 babarcvdigned;
    UWORD32 batxcompigned1;
    UWORD32 batxcompigned2;
    UWORD32 basenddelba;
    UWORD32 babuffexc;
    UWORD32 banumbuff;
    UWORD32 bapcfdelba;
    UWORD32 barxboflow;
    UWORD32 baothba;

    UWORD32 babarxd;              /* Number of BA Frames Rxd.                */
    UWORD32 babartxd;             /* Number of BAR Frames successfully Txd.  */
    UWORD32 babarrxd;             /* Number of BAR Frames Rxd.               */
    UWORD32 badatretx;            /* Number of Data Frames retransmitted.    */
    UWORD32 bawinmove;            /* Number of times Window is moved.        */
    UWORD32 babarfail;            /* Number of BAR Tx-Failures.              */
    UWORD32 badatfail;            /* Number of Data Tx-Failures.             */
    UWORD32 babufmiss;            /* Number of Missing Buffers.              */
    UWORD32 badatclnup;           /* Number of Frames deleted during Buffer  */
                                  /* cleanup.                                */
    UWORD32 bapenddrop;           /* Number of Pending Frames discarded.     */
    UWORD32 bapendingtxwlantxd;   /* Number of frames Txd from the Pending Q.*/
    UWORD32 baoldbarxd;           /* Number of Stale BA frames received.     */
    UWORD32 baoldbarrxd;          /* Number of Stale BAR frames received.    */
    UWORD32 barxdatoutwin;        /* Number of frames received out of window */
                                  /* and hence droped.                       */
    UWORD32 banumblks;            /* Number of blocks Qed.                   */
    UWORD32 banumqed;             /* Number of packets Qed.                  */
    UWORD32 baemptyQ;             /* Number of times the pending Q was empty */
                                  /* while enqueing.                         */
    UWORD32 num_buffto;           /* Num of times grp TO.                    */
    UWORD32 ba_num_dq;
    UWORD32 ba_num_dqed;
    UWORD32 batxfba;
    UWORD32 banobuffrx;           /* Number of frames passed up without      */
                                  /* buffering.                              */
    UWORD32 bafilunexp;           /* BA received though it is filtered.      */
    UWORD32 bartrqfail;
    UWORD32 batemp[10];
    UWORD32 babartxqfail;
    UWORD32 basechkfail;          /* Number of Replay Failures on BA-RX.     */
    UWORD32 batxwinstallco;       /* Number of BA-TX Window Stall seen afer  */
                                  /* Count-Out.                              */
    UWORD32 batxwinstallto;       /* Number of BA-TX Window Stall seen afer  */
                                  /* Timeout.                                */
    UWORD32 batxwinstalltoscn1;
    UWORD32 batxwinstalltoscn2;

    /* FSM exception counts */
    UWORD32 wtdnull;
    UWORD32 wrdnull;

    /* WMM PS debug and exception counts */
    UWORD32 wmmpssta_tacdl;       /* Trigger enabled Downlink traffic        */
                                  /* exception.                              */
    UWORD32 wmmpssta_dacul;       /* Delivery enabled Uplink traffic         */
                                  /* exception.                              */
    UWORD32 wmmpssta_spsw;        /* Wait for service period start.          */
    UWORD32 wmmpssta_sps;         /* Service period start                    */
    UWORD32 wmmpssta_spe;         /* Service period end                      */
    UWORD32 wmmpssta_trigsp;      /* Trigger service period                  */
    UWORD32 wmmpssta_trspnr;      /* Trigger service period is not required  */

    /* Misc exception counters */
    BOOL_T  max_query_rsp_len_exceeded;
    UWORD32 tx_timeout;
    UWORD32 num_amsdu_drop;
    UWORD32 num_frag_drop;

    UWORD32 fc_buff_limit;        /* Flow Control on Host data path due to   */
                                  /* Buffer Limitation.                      */
    UWORD32 fc_no_buff;           /* Flow Control on Host data path due to   */
                                  /* Falied Malloc.                          */
    UWORD32 fc_cfg_no_buff;       /* Flow Control on Host cfg path due to    */
                                  /* Falied Malloc.                          */
    UWORD32 num_wlan_fc;          /* Flow Control on WLAN data path due to   */
                                  /* Buffer Limitation.                      */
    UWORD32 num_wlan_fc_nobuff;   /* Flow Control on WLAN data path due to   */
                                  /* Falied Malloc                           */
    UWORD32 no_free_pbuff;        /* Zero Packet Buffers available.          */
    UWORD32 rsvd_buff_alloc[5];   /* Reserved buffer alloced to module.      */

    UWORD32 invalidct;            /* Number of invalid cipher type detected. */
    UWORD32 pwrx_maxlenexc;       /* Number of Rx frames exceeding the       */
                                  /* maximum acceptable length.              */
    UWORD32 pwrx_minlenexc;       /* Number of Rx frames less than the       */
                                  /* minimum acceptable length.              */
    UWORD32 brx_frame_droped;     /* Number of broadcast Rx frames dropped.  */
    UWORD32 urx_frame_droped;     /* Number of unicast Rx frames dropped.    */
    UWORD32 txrate_reinit_err;    /* Number of times the Tx rate index failed*/
                                  /* to reinitialize.                        */
    UWORD32 no_mcs_index;         /* Number of times the Tx rate could not be*/
                                  /* updated.                                */
    UWORD32 non_mcs_txrate;       /* Number of times the Tx rate is          */
                                  /* found to be non MCS.                    */

    /* Configuration Error */
    UWORD32 stanotassc;           /* STA could not be associated.            */
    UWORD32 nonhtwmmsta;          /* The STA is not WMM and HT capable.      */
    UWORD32 nostafound;           /* No STA found.                           */
    UWORD32 celutdelfail;    /* Number of CE-LUT Key delete failures */
    UWORD32 celutaddfail;    /* Number of CE-LUT Key Add failures */
    UWORD32 celutpnfail;     /* Number of CE-LUT PN update failures */
    UWORD32 phyregrwfail;    /* Number of PHY Register Access failures */

    /* Counters related to STA mode powersave */
    UWORD32 num_wakeup_on_hostrx; /* No of wakeups based on host-rx event    */
    UWORD32 num_wakeup_on_tbtt;   /* No of wakeups based on TBTT event       */
    UWORD32 num_wakeup_on_dtim;   /* No of wakeups based on DTIM period      */
    UWORD32 num_wakeup_on_tim;    /* No of wakeups based on TIM set in Bcns  */
    UWORD32 num_wake2active_trn;  /* No of transition from wake 2 active     */
    UWORD32 num_active2wake_trn;  /* No of transition from active 2 wake     */
    UWORD32 num_sta_doze;         /* No of STA dozes                         */

	/* tx pkt info statistics */
	UWORD32 tx_data_frame_len;		/* tx data frame total length in some time*/
	UWORD32 tx_data_frame_count;	/* tx send data times in some time        */
	UWORD32 tx_mgmt_frame_len;		/* tx mgmt frame total length in some time*/
	UWORD32 tx_mgmt_frame_count;	/* tx send mgmt times in some time        */

	/* rx pkt info statistics */
	UWORD32 rx_data_frame_len;      /* rx data frame total length in some time*/
	UWORD32 rx_data_frame_count;    /* rx rcv data times in some time         */
	UWORD32 rx_mgmt_frame_len;      /* rx mgmt frame total length in some time*/
	UWORD32 rx_mgmt_frame_count;    /* rx rcv mgmt times in some time         */
	UWORD32 rx_ctrl_frame_len;      /* rx rcv control frame total length in some time */
	UWORD32 rx_ctrl_frame_count;	/* rx rcv control frame times in some time*/

	/* tx error info statistics */
	UWORD32 no_mem_count;           /* no descr or buffer count               */
	UWORD32 event_no_mem_count;     /* event struct alloc no mem              */
	UWORD32 tx_queue_full_count;    /* tx queue full count                    */
	UWORD32 tx_busy_count;          /* tx share memory busy count             */

	UWORD32 rcv_rx_dscr_num;        /* received dscr number                   */
	UWORD32 host_rx_queue_full;     /* host rx queue full count               */
	UWORD32 rx_drop_pkt_count;      /* rx drop pkt count */
	UWORD32 rcv_prob_rsp_count;	    /* received probe response pkt count      */
	UWORD32 rcv_beacon_fram_count;  /* received beacon frame count            */
//============================================================================//
    /* Dumy add for Rx Buff accumulate*/
    UWORD32 rx_data_dscr_total_num;
    UWORD32 rx_data_count;
    UWORD32 rx_manage_dscr_total_num;
    UWORD32 rx_manage_count;
    UWORD32 num_dsc[2000];
    UWORD32 num_1_count;
    UWORD32 num_2_count;
    UWORD32 num_lis_10_count;
    UWORD32 num_lis_20_count;
    UWORD32 num_lis_30_count;
    UWORD32 num_lis_40_count;   
     
    /* pkt count */
    UWORD32 scan_ap_count;
    
} mac_stats_t;

typedef struct
{
    /* Rare system error exceptions */
    UWORD32 no_local_mem;
    UWORD32 no_shred_mem;
    UWORD32 no_event_mem;
    UWORD32 numresets;
    UWORD32 txpnvalexc;
    UWORD32 hackerexc;
    UWORD32 gtkidexc;
    UWORD32 txdcsrexc;
    UWORD32 txbuffexc;
    UWORD32 nxtdcsrexc;
    UWORD32 rxqptrexc;
    UWORD32 nxtqptrexc;
    UWORD32 rxbuffexc;
    UWORD32 tx_q_error;
    UWORD32 rx_q_error;

    /* Other reasons for resetting the system */
    UWORD32 pacorruptcnt;
    UWORD32 linklosscnt;
    UWORD32 linklosspacnt;
    UWORD32 linklossbuffcnt;
    UWORD32 deauthcnt;
    UWORD32 rsnahstimeout;
    UWORD32 rsnahsfailcnt;
    UWORD32 hwerrint;
    UWORD32 configchange;

} reset_stats_t;

#endif /* DEBUG_MODE */

#ifdef MEM_DEBUG_MODE

typedef struct
{
    /* Shared Dscr Memory Pool details */
    UWORD32 sdalloc;
    UWORD32 sdfree;
    UWORD32 sdtotalfree;

    /* Shared Pkt Memory Pool details */
    UWORD32 spalloc;
    UWORD32 spfree;
    UWORD32 sptotalfree;

    /* Local Memory Pool details */
    UWORD32 lalloc;
    UWORD32 lfree;
    UWORD32 ltotalfree;

    /* Event Memory Pool details */
    UWORD32 ealloc;
    UWORD32 efree;
    UWORD32 etotalfree;

    /* Mem Alloc exception counts */
    UWORD32 nosizeallocexc;     /* no need size buffer             */
    UWORD32 nofreeallocexc;     /* No free chunks are available in the sub pool */
    UWORD32 reallocexc;
    UWORD32 corruptallocexc;

    /* Mem Free exception counts */
    UWORD32 nullfreeexc;
    UWORD32 oobfreeexc;
    UWORD32 refreeexc;
    UWORD32 corruptfreeexc;
    UWORD32 invalidfreeexc;
    UWORD32 excessfreeexc;
    UWORD32 lockedfreeexc;

    /* Mem Add User exception counts */
    UWORD32 nulladdexc;
    UWORD32 oobaddexc;
    UWORD32 freeaddexc;
    UWORD32 invalidaddexc;
    UWORD32 excessaddexc;

    /* Scratch Memory Pool details */
    UWORD32 scrinit;
    UWORD32 scrsave;
    UWORD32 scrrestore;
    UWORD32 scralloc;
    UWORD32 scrallocfail;
    UWORD32 scrrestoreerr;

    /* ITM-DEBUG */
    UWORD32 nofreeDscrallocexc[2];
    UWORD32 nofreePktallocexc[5];
    UWORD32 nofreeLocalallocexc[8];
    UWORD32 nofreeEventallocexc;

} mem_stats_t;

#endif /* MEM_DEBUG_MODE */

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/

#ifdef DEBUG_MODE
extern mac_stats_t   g_mac_stats;
extern reset_stats_t g_reset_stats;
extern UWORD8        g_enable_debug_print;
extern UWORD8        g_11n_print_stats;
#endif /* DEBUG_MODE */

#ifdef MEM_DEBUG_MODE
extern mem_stats_t g_mem_stats;
#endif /* MEM_DEBUG_MODE */

extern UWORD8 g_cfg_val[MAX_CFG_LEN];
//chenq add
extern UWORD8 g_cfg_val_for_gscan[512];

//chenq add for wpa/rsn ie buf
#ifdef IBSS_BSS_STATION_MODE //chenq add a ifdef for sta mode 20 ap info 2013-02-05

#ifdef CONFIG_NUM_AP_SCAN
    #define MAX_SITES_FOR_SCAN   CONFIG_NUM_AP_SCAN
#else /* CONFIG_NUM_AP_SCAN */
    #define MAX_SITES_FOR_SCAN   8
#endif /* CONFIG_NUM_AP_SCAN */

#else/*BSS_ACCESS_POINT_MODE*/

#ifdef CONFIG_NUM_STA
    #define MAX_SITES_FOR_SCAN   CONFIG_NUM_STA
#else /* CONFIG_NUM_STA */
    #define MAX_SITES_FOR_SCAN   8
#endif /* CONFIG_NUM_STA */

#endif

extern UWORD8 g_wpa_rsn_ie_buf_2[MAX_SITES_FOR_SCAN][512];

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
/*zhou huiquan add for close mac clock*/
extern inline void notify_cp_with_handshake(uint msg, uint retry);
extern inline void root_notify_cp_with_handshake(uint msg, uint retry);
#endif
#ifdef DSCR_MACROS_NOT_DEFINED
extern void    set_dscr_fn(UWORD8 offst, UWORD16 wdth, UWORD32 *ptr,
                           UWORD32 val);
extern UWORD32 get_dscr_fn(UWORD8 offst, UWORD16 wdth, UWORD32 *ptr);
#endif /* DSCR_MACROS_NOT_DEFINED */

#ifdef MWLAN
extern void    *itm_memset(void *buff, UWORD8 val, UWORD32 num);
#endif /* MWLAN */

#ifdef DEBUG_MODE
extern void print_debug_stats(void);
extern void printh(WORD8 *msg, ...);
extern void print_build_flags(void);
extern void print_ba_debug_stats(void);
extern UWORD8 print_mem_stats(void);
#endif /* DEBUG_MODE */

extern void calibrate_delay_loop(void);
extern void add_calib_delay(UWORD32 delay);

#ifdef COMBO_SCAN
extern void calibrate_delay_loop_plus(void);
#endif

/*****************************************************************************/
/* Inline functions                                                          */
/*****************************************************************************/

/* This function calls the set descriptor function or Macro */
INLINE void set_dscr(UWORD8 offset, UWORD16 width, UWORD32 *ptr, UWORD32 val)
{
#ifdef DSCR_MACROS_NOT_DEFINED
    set_dscr_fn(offset, width, ptr, val);
#else /* DSCR_MACROS_NOT_DEFINED */
    SET_DSCR(offset, width, ptr, val);
#endif /* DSCR_MACROS_NOT_DEFINED */
}

/* This function calls the get descriptor function or Macro */
INLINE UWORD32 get_dscr(UWORD8 offset, UWORD16 width, UWORD32 *ptr)
{
#ifdef DSCR_MACROS_NOT_DEFINED
    return get_dscr_fn(offset, width, ptr);
#else /* DSCR_MACROS_NOT_DEFINED */
    return GET_DSCR(offset, width, ptr);
#endif /* DSCR_MACROS_NOT_DEFINED */
}

/* This function prints log messages for debug level 1 */
INLINE void print_log_debug_level_1(UWORD8 *x)
{
#ifdef DEBUG_MODE
#ifdef DEBUG_LEVEL_1
    print_log(x);
#endif /* DEBUG_LEVEL_1 */
#endif /* DEBUG_MODE */
}

/* This function prints log messages for debug level 2 */
INLINE void print_log_debug_level_2(UWORD8 *x)
{
#ifdef DEBUG_MODE
#ifdef DEBUG_LEVEL_2
    print_log(x);
#endif /* DEBUG_LEVEL_2 */
#endif /* DEBUG_MODE */
}

/* This function swaps the byte order in the given 32-bit word in BIG_ENDIAN */
/* mode to convert to Little Endian format. For LITTLE_ENDIAN mode no change */
/* is required.                                                              */
INLINE UWORD32 convert_to_le(UWORD32 val)
{
#ifdef BIG_ENDIAN
    return SWAP_BYTE_ORDER_WORD(val);
#endif /* BIG_ENDIAN */

#ifdef LITTLE_ENDIAN
    return val;
#endif /* LITTLE_ENDIAN */
}

/* This function swaps the byte order in the given 16-bit word in BIG_ENDIAN */
/* mode to convert to Little Endian format. For LITTLE_ENDIAN mode no change */
/* is required.                                                              */
INLINE UWORD16 convert_to_le_w16(UWORD16 val)
{
#ifdef BIG_ENDIAN
    return SWAP_BYTE_ORDER_SHORT(val);
#endif /* BIG_ENDIAN */

#ifdef LITTLE_ENDIAN
    return val;
#endif /* LITTLE_ENDIAN */
}

/* This function compares two given MAC addresses (m1 and m2).               */
/* Returns BOOL_T, BTRUE if the two addresses are same and BFALSE otherwise. */
INLINE BOOL_T mac_addr_cmp(UWORD8* m1, UWORD8* m2)
{
    if(memcmp(m1, m2, 6) == 0)
        return BTRUE;

    return BFALSE;
}

/* This function compares two given MAC addresses (m1 and m2).               */
/* Returns BOOL_T, BTRUE if the two addresses are same and BFALSE otherwise. */
INLINE BOOL_T is_mac_addr_null(UWORD8* m1)
{
    if(memcmp(m1, "\x00\x00\x00\x00\x00\x00", 6) == 0)
        return BTRUE;

    return BFALSE;
}


/* This function copies the MAC address from a source to destination */
INLINE void mac_addr_cpy(UWORD8 *dst, UWORD8 *src)
{
    UWORD8 i = 0;

    for(i = 0; i < 6; i++)
        dst[i] = src[i];
}

/* This function copies the IP address from a source to destination */
INLINE void ip_addr_cpy(UWORD8 *dst, UWORD8 *src)
{
    UWORD8 i = 0;

    for(i = 0; i < 4; i++)
        dst[i] = src[i];
}
/* This function copies contents from source address to destination address  */
/* word-wise with no change in format.                                       */
INLINE void memcpy32(UWORD8 *dst, UWORD8 *src, UWORD16 len)
{
     UWORD32 i = 0;

     if((((UWORD32)dst)%4) || (((UWORD32)src)%4))
     {
        /* Exception case - Source / Destination address is not word-aligned.*/
        /* Call function to do memcpy byte-wise.                             */
        memcpy(dst, src, len);
        return;
     }

     /* Compute the number of word transfers to be done */
     if(len % 4)
        len = len/4 + 1;
     else
        len /= 4;

     /* Copy the required number of words from source to destination */
     for(i = 0; i < len; i++)
     {
         *((UWORD32 *)dst + i) = *((UWORD32 *)src + i);
     }
}

/* This function copies from source address to destination address byte-wise */
/* and ensures that the data in the destination is in Little endian mode.    */
INLINE void memcpy_le(UWORD8 *dst, UWORD8 *src, UWORD16 len)
     {
         UWORD32 i = 0;

    /* Copy required number of bytes from source to destination in little    */
    /* endian format.                                                        */
    for(i = 0; i < len; i += 4)
         {
#ifdef BIG_ENDIAN
        dst[i + 3] = src[i];
        dst[i + 2] = src[i + 1];
        dst[i + 1] = src[i + 2];
        dst[i + 0] = src[i + 3];
#endif /* BIG_ENDIAN */

#ifdef LITTLE_ENDIAN
        dst[i]     = src[i];
        dst[i + 1] = src[i + 1];
        dst[i + 2] = src[i + 2];
        dst[i + 3] = src[i + 3];
#endif /* LITTLE_ENDIAN */
    }
}

/* This function copies from source address to destination address word-wise */
/* and ensures that the data in the destination is in Little endian mode.    */
INLINE void memcpy32_le(UWORD8 *dst, UWORD8 *src, UWORD16 len)
{
    UWORD32 i    = 0;
    UWORD32 temp = 0;

    if((((UWORD32)dst) % 4) || (((UWORD32)src) % 4))
    {
        /* Exception case - Source / Destination address is not word-aligned.*/
        /* Call function to do memcpy byte-wise.                             */
        memcpy_le(dst, src, len);
        return;
    }

    /* Compute the number of word transfers to be done */
    if(len % 4)
        len = len/4 + 1;
    else
        len /= 4;

    /* Copy the required number of words from source to destination after    */
    /* converting to little endian format, if required.                      */
    for(i = 0; i < len; i++)
    {
        temp = *(( UWORD32 *)src + i);

        *((UWORD32 *)dst + i) = convert_to_le(temp);
     }
}

/* This function chooses between standard library or Ittiam implementation */
/* of the memset function depending upon the platform.                     */
INLINE void *mem_set(void *buff, WORD32 val,  UWORD32 num)
{
#ifdef MWLAN
    return itm_memset(buff, val, num);
#else  /* MWLAN */
    return memset(buff, val, num);
#endif /* MWLAN */
}

/* This function returns the offset required from the input address(input) */
/* to achieve the desired alignment (align).                               */
INLINE UWORD32 get_align_offset(UWORD32 input, UWORD32 align)
{
    UWORD32 offset = input % align;

    if(offset != 0)
        offset = align - offset;

    return offset;
}


/* This funtion prints the value if its is greater than zero and return      */
/* BTRUE else if the value is less than zero it returns BFALSE.              */
INLINE BOOL_T printe(WORD8 *strn, UWORD32 val)
{
    if(val > 0)
    {
        PRINTK("%-48s= %d\n",strn,val);
        return BTRUE;
    }
    return BFALSE;
}

/* This function returns the number of bits set in a 32-bit value */
INLINE UWORD8 get_num_bits_set(UWORD32 val)
{
    UWORD32 count = 0;

    for(count = 0; val; count++)
    {
        val &= val - 1;
    }

    return count;
}

/* This function returns the number of bits set in a 32-bit value preceding  */
/* the given bit position. For example, for val = 0x90037F53, bit_pos = 10,  */
/* this function should return 5                                             */
/* Create a mask for all bits upto bit_pos i.e. mask = 0x000001FF            */
/* Find the number of bits set in given val masked with this i.e. 0x00000153 */
/* (0001 0101 0011) : 5 1's                                                  */
INLINE UWORD8 get_num_preceding_bits_set(UWORD32 val, UWORD8 bit_pos)
{
    UWORD32 mask = (1 << bit_pos) - 1;

    return get_num_bits_set(val & mask);
}

/* This function returns the bit position of a bit with given rank.          */
/* For example, for val = 0x90037F53, bit_rank = 5, this function should     */
/* return 9                                                                  */
/* ... 1 1 0 1 0 1 0 0 1 1 : Bitmap                                          */
/* ... 5 4 x 3 x 2 x x 1 0 : Bit rank                                        */
/* ... 9 8 7 6 5 4 3 2 1 0 : Bit position                                    */
/* In a loop, increment a bit count and check the least significant bit in   */
/* the bitmap. If it is set decrement the bit rank count. If the bit rank    */
/* count becomes zero it indicates that the bit position has been found. If  */
/* the bitmap becomes zero it indicates that no bit position could be found  */
/* for the given bit rank. Return an invalid value to indicate this.         */
INLINE UWORD8 get_bit_position(UWORD32 bmap, UWORD8 bit_rank)
{
    UWORD8 bit_cnt     = 0;
    UWORD8 set_bit_cnt = 0;

    while(bmap > 0)
    {
        if(bmap & BIT0) /* LSB is set */
        {
            if(set_bit_cnt == bit_rank)
            {
                /* Number of bits set before this match the bit rank. This   */
                /* bit is the required position for the given bit rank.      */
                break;
            }

            set_bit_cnt++; /* Increment set bit count */
        }

        /* Go to next bit */
        bmap >>= 1;
        bit_cnt++;
    }

    return bit_cnt;
}

INLINE BOOL_T is_bit_position_valid(UWORD8 bit_pos)
{
    if(bit_pos < MAX_BMAP_SZ)
        return BTRUE;

    return BFALSE;
}

/*chenq add for itm trace*/
#define itm_debug_s_flag 's'
#define itm_debug_g_flag 'g'

#define itm_debug_reg_flag "01"
#define itm_debug_plevel_flag "02"
#define itm_debug_plog_flag "03"

#define itm_debug_all_reg_flag "ff"
#define itm_debug_user_ind_flag "fe"
#define itm_debug_pa_flag "00"
#define itm_debug_ce_flag "01"
#define itm_debug_cm_flag "02"
#define itm_debug_py_flag "03"
#define itm_debug_sys_flag "04"
#define itm_debug_rf_flag "05"
#define itm_debug_ada_flag "06"

#define itm_debug_plevel_all "00"

//#define itm_debug_plog_all "00"

#define itm_debug_plog_sharemem "00"
#define itm_debug_plog_mactxrx "01"
#define itm_debug_plog_spisdiodma "02"
#define itm_debug_plog_macfsm_mib "03"
#define itm_debug_plog_host6820info "04"

/*sub-type of ShareMemInfo*/
#define itm_debug_plog_sharemem_tx_pkt  0
#define itm_debug_plog_sharemem_rx_ptk  1
#define itm_debug_plog_sharemem_tx_buf  2
#define itm_debug_plog_sharemem_rx_buf  3
#define itm_debug_plog_sharemem_buf_use 4
/*sub-type of MacTxRxStatistics*/
#define itm_debug_plog_mactxrx_reg      0
#define itm_debug_plog_mactxrx_frame    1
#define itm_debug_plog_mactxrx_rx_size  2
#define itm_debug_plog_mactxrx_isr      3
/*sub-type of SpiSdioDmaState*/
#define itm_debug_plog_spisdiodma_spisdio 0
#define itm_debug_plog_spisdiodma_dma     1
#define itm_debug_plog_spisdiodma_isr     2
/*sub-type of MacFsmMibState*/
#define itm_debug_plog_macfsm_mib_fsm       0
#define itm_debug_plog_macfsm_mib_assoc     1
#define itm_debug_plog_macfsm_mib_Enc_auth  2
#define itm_debug_plog_macfsm_mib_wps       3
#define itm_debug_plog_macfsm_mib_ps        4//PowerSave
#define itm_debug_plog_macfsm_mib_wd        5//WiFi-Direct
#define itm_debug_plog_macfsm_mib_txrx_path 6

#define itm_debug_plog_macfsm_mib_mibapp    0x10000
#define itm_debug_plog_macfsm_mib_mibprtcl  0x10001
#define itm_debug_plog_macfsm_mib_mibmac    0x10002
/*sub-type of Host6820Info*/
//no add

/* set value */
#define print_on  1
#define print_off 0
#define counter_start 1
#define counter_end   0

/*flag of ShareMemInfo*/
extern int g_debug_print_tx_pkt_on;
extern int g_debug_print_rx_ptk_on;
extern int g_debug_print_tx_buf_on;
extern int g_debug_print_rx_buf_on;
extern int g_debug_buf_use_info_start;
/*flag of MacTxRxStatistics*/
extern int g_debug_txrx_reg_info_start;
extern int g_debug_txrx_frame_info_start;
extern int g_debug_rx_size_info_start;
extern int g_debug_isr_info_start;
/*flag of SpiSdioDmaState*/
extern int g_debug_print_spisdio_bus_on;
extern int g_debug_print_dma_do_on;
extern int g_debug_spisdiodma_isr_info_start;
/*flag of MacFsmMibState*/
extern int g_debug_print_fsm_on;
extern int g_debug_print_assoc_on;
extern int g_debug_print_Enc_auth_on;
extern int g_debug_print_wps_on; 
extern int g_debug_print_ps_on;//PowerSave
extern int g_debug_print_wd_on;//WiFi-Direct
extern int g_debug_print_txrx_path_on;
/*flag of Host6820Info*/
//no add

extern void Reset_itm_trace_flag(void);
extern void ShareMemInfo(int type,int flag,int value,char * reserved2ext);
extern void MacTxRxStatistics(int type,int flag,char * reserved2ext);
extern void SpiSdioDmaState(int type,int flag,int value,char * reserved2ext);
extern void MacFsmMibState(int type,int value,char * reserved2ext);
extern void Host6820Info(int type,char * reserved2ext);

/*print fsm */
#define PRINT_FSM(args...)  if(g_debug_print_fsm_on == 1)\
							{\
								PRINTD(args);\
								print_symbol("at %s\n", (unsigned long)__builtin_return_address(0));\
							}
/*print scan auth assoc Proc*/
#define PRINT_AssocProc(args...)    if(g_debug_print_assoc_on == 1)\
								{\
									PRINTD(args);\
								}
/*print wpa wpa2 EncProc*/
#define PRINT_EncProc(args...)  if(g_debug_print_Enc_auth_on == 1)\
							{\
								PRINTD(args);\
							}
/*print wps */
#define PRINT_WPS(args...)  if(g_debug_print_wps_on == 1)\
							{\
								PRINTD(args);\
							}
/*print power save*/
#define PRINT_PS(args...)   if(g_debug_print_ps_on == 1)\
							{\
								PRINTD(args);\
							}
/*print wifi-direct*/
#define PRINT_WD(args...)   if(g_debug_print_wd_on == 1)\
							{\
								PRINTD(args);\
							}
/*print data rx tx*/
#define PRINT_DATA_PKT(args...)  if(g_debug_print_txrx_path_on == 1)\
							{\
								PRINTD(args);\
							}

#define PRINT_MIB(args...)  if(g_debug_print_txrx_path_on == 1)\
							{\
								PRINTD(args);\
							}

/*chenq add end*/

/*leon liu added judgement for WEXT*/
#ifndef CONFIG_TROUT_WEXT
//#define wireless_send_event(dev, event, wrqu, buf)	
#endif

#endif /* COMMON_H */
