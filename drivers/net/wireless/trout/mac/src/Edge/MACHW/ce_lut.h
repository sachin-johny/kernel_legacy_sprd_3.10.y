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
/*  File Name         : ce_lut.h                                             */
/*                                                                           */
/*  Description       : This file contains definitions and inline utility    */
/*                      functions required by the MAC CE LUT interface       */
/*                                                                           */
/*  List of Functions : enable_ce                                            */
/*                      disable_ce                                           */
/*                      machw_ce_update_key                                  */
/*                      machw_ce_read_key                                    */
/*                      set_machw_ce_macaddr                                 */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef CE_LUT_H
#define CE_LUT_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "mh.h"
#include "spi_interface.h"
/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

typedef enum {TX_GTK_NUM  = 0,
              PTK_NUM     = 1,
              RX_GTK0_NUM = 2,
              RX_GTK1_NUM = 3
} KEY_NUMBER_T;


typedef enum {AUTH_KEY = 0,
              SUPP_KEY = 1
} KEY_ORIGIN_T;

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define LUT_KEY_PRGM_MASK   0x02
#define LUT_ADDR_PRGM_MASK  0x01

#define LUT_WRITE_OPERN     0x000
#define LUT_READ_OPERN      0x100
#define LUT_REMOVE_OPERN    0x200

/* Time (in units of 10us) to wait for CE-LUT update operation to complete */
#define CE_LUT_UPDATE_TIMEOUT       20000 /* 20ms */
#define NUM_CE_LUT_UPDATE_ATTEMPTS  2

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void   initialize_lut(void);
extern void   get_machw_ce_pn_val(UWORD8* val);
extern BOOL_T machw_ce_del_key(UWORD8 sta_index);
extern BOOL_T machw_ce_add_key(UWORD8 key_id, KEY_NUMBER_T key_num,
                               UWORD8 sta_index, UWORD8 mask,
                               UWORD8 cipher_type, UWORD8 *key, UWORD8 *addr,
                               KEY_ORIGIN_T key_origin,UWORD8 *mic_key);
extern BOOL_T machw_ce_init_rx_bcmc_pn_val(UWORD8 tid, UWORD8 sta_index,
                                           UWORD8 *pn_val);

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/* This function enables the CE in the PA */
INLINE void enable_ce(void)
{
    /* Enable CE */

    //chenq mod
    //rMAC_PA_CON |= REGBIT11;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PA_CON ) | REGBIT11,
        (UWORD32)rMAC_PA_CON);
}

/* This function disables the CE in the PA */
INLINE void disable_ce(void)
{
    /* Enable CE */
    //chenq mod
    //rMAC_PA_CON &= ~(REGBIT11);
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PA_CON ) & (~(REGBIT11)),
        (UWORD32)rMAC_PA_CON);
}

/* Function checks if CE enabled  */
INLINE BOOL_T is_ce_enabled(void)
{
    //chenq mod
    //if(rMAC_PA_CON & REGBIT11)
    if( host_read_trout_reg( (UWORD32)rMAC_PA_CON ) & REGBIT11)
        return BTRUE;

    return BFALSE;
}

/* Functions to enable/disable clock gating for CE */
INLINE void enable_machw_ce_clkgating(void)
{
#ifndef DISABLE_CE_CLKGATING
    //chenq mod
    //rMAC_CE_CONFIG &= ~REGBIT3;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_CE_CONFIG ) & (~REGBIT3) ,
        (UWORD32)rMAC_CE_CONFIG );
#endif /* DISABLE_CE_CLKGATING */
}

INLINE void disable_machw_ce_clkgating(void)
{
#ifndef DISABLE_CE_CLKGATING
    //chenq mod
    //rMAC_CE_CONFIG |= REGBIT3;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_CE_CONFIG ) | (REGBIT3) ,
        (UWORD32)rMAC_CE_CONFIG );
#endif /* DISABLE_CE_CLKGATING */
}

/* This function updates the LUT entry for the given index */
/* This feature is redundant and same as write key         */
/* Thus it calls the ADD KEY API                           */
INLINE BOOL_T machw_ce_update_key(UWORD8 key_id, KEY_NUMBER_T key_num,
        UWORD8 sta_index, UWORD8 mask, UWORD8 cipher_type,
        UWORD8 *key, UWORD8 *sta_addr, KEY_ORIGIN_T key_origin,UWORD8* mic_key)
{
    return machw_ce_add_key(key_id, key_num, sta_index, mask, cipher_type,
                key, sta_addr,key_origin,mic_key);
}

/* This function reads a key for an given key index */
/* The addresses are *not* updated for the given key index. Only the key */
/* for the given key index is updated. If the STA at the given key index */
/* uses WEP, key id is used and updated at key_num = 0, else all keys    */
/* for the given key index are updated                                   */
INLINE BOOL_T machw_ce_read_key(UWORD8 key_num, UWORD8 key_id,
                                UWORD8 sta_index, UWORD8 *key)
{
    UWORD32 temp = 0;

	/*junbinwang add for ce config clock gating 20131009.bug 218429*/
	disable_machw_ce_clkgating();

    //chenq mod
    //temp = convert_to_le(rMAC_CE_LUT_OPERN);
    temp = convert_to_le(host_read_trout_reg( (UWORD32)rMAC_CE_LUT_OPERN));

    /* Update the Key number */
    temp = (temp & 0xFFFFFF3F) | (((key_num & 0x03) << 6) & 0x000000C0);

    /* Update the LUT Operation */
    temp = (temp & 0xFFFFFCFF) | LUT_READ_OPERN;

    /* Update the Key ID */
    temp = (temp & 0xFFFFF3FF) | (((key_id & 0x03) << 10) & 0x00000C00);

    /* Update the LUT index */
    temp = (temp & 0xFF83FFFF) | (((sta_index & 0x1F) << 18) & 0x007C0000);

    /* Update the ADDR/KEY Programing mask */
    temp = temp & 0xFFFFCFFF;

    /*Set LUT_EN bit*/
    temp = temp | BIT0;

    //chenq mod
    //rMAC_CE_LUT_OPERN = convert_to_le(temp);
    host_write_trout_reg( convert_to_le(temp), 
                         (UWORD32)rMAC_CE_LUT_OPERN );

    /* Poll for the LUT_EN Bit to be reset */
    //chenq mod
    //while(rMAC_CE_LUT_OPERN & REGBIT0)
    while(host_read_trout_reg( (UWORD32)rMAC_CE_LUT_OPERN ) & REGBIT0)
    {
        key_num = key_num;
    }

    //chenq mod
    //temp = convert_to_le(rMAC_CE_LUT_OPERN);
    temp = convert_to_le( host_read_trout_reg( (UWORD32)rMAC_CE_LUT_OPERN) );

    //temp   = convert_to_le(rMAC_CE_KEY_FIRST);
    temp   = convert_to_le(host_read_trout_reg( (UWORD32)rMAC_CE_KEY_FIRST) );
    key[0] = (temp & 0xFF000000) >> 24;
    key[1] = (temp & 0x00FF0000) >> 16;
    key[2] = (temp & 0x0000FF00) >> 8;
    key[3] = (temp & 0x000000FF);
    key += 4;

    //temp   = convert_to_le(rMAC_CE_KEY_SECOND);
    temp   = convert_to_le(host_read_trout_reg( (UWORD32)rMAC_CE_KEY_SECOND));
    key[0] = (temp & 0xFF000000) >> 24;
    key[1] = (temp & 0x00FF0000) >> 16;
    key[2] = (temp & 0x0000FF00) >> 8;
    key[3] = (temp & 0x000000FF);
    key += 4;

    //temp   = convert_to_le(rMAC_CE_KEY_THIRD);
    temp   = convert_to_le(host_read_trout_reg( (UWORD32)rMAC_CE_KEY_THIRD));
    key[0] = (temp & 0xFF000000) >> 24;
    key[1] = (temp & 0x00FF0000) >> 16;
    key[2] = (temp & 0x0000FF00) >> 8;
    key[3] = (temp & 0x000000FF);
    key += 4;

    //temp   = convert_to_le(rMAC_CE_KEY_FOURTH);
    temp   = convert_to_le(host_read_trout_reg( (UWORD32)rMAC_CE_KEY_FOURTH));
    key[0] = (temp & 0xFF000000) >> 24;
    key[1] = (temp & 0x00FF0000) >> 16;
    key[2] = (temp & 0x0000FF00) >> 8;
    key[3] = (temp & 0x000000FF);
    key += 4;

	/*junbinwang add for ce config clock gating 20131009.bug 218429*/
	enable_machw_ce_clkgating();
    return BTRUE;
}

/* The MAC address is set in the 2 registers with the lower 32 bits set in   */
/* rMAC_ADDR_LO and higher 16 bits in rMAC_ADDR_HI                           */
INLINE void set_machw_ce_macaddr(UWORD8* mac_addr)
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
    //rMAC_CE_MAC_ADDR_MSB = convert_to_le(mac_h);
    //rMAC_CE_MAC_ADDR_LSB = convert_to_le(mac_l);
    
    host_write_trout_reg( convert_to_le(mac_h), 
                          (UWORD32)rMAC_CE_MAC_ADDR_MSB );
    host_write_trout_reg( convert_to_le(mac_l), 
                          (UWORD32)rMAC_CE_MAC_ADDR_LSB );

}

/* This function sets the Group Key Type register */
INLINE void set_machw_ce_grp_key_type(UWORD32 val)
{
    //chenq mod
    //rMAC_CE_RX_GRP_CIPHER_TYPE = convert_to_le(val);
    host_write_trout_reg( convert_to_le(val), 
                          (UWORD32)rMAC_CE_RX_GRP_CIPHER_TYPE );
}

/* This function gets the value of Group Key Type register */
INLINE UWORD32 get_machw_ce_grp_key_type(void)
{
    //chenq mod
    //return convert_to_le(rMAC_CE_RX_GRP_CIPHER_TYPE);
    return convert_to_le(host_read_trout_reg( 
                            (UWORD32)rMAC_CE_RX_GRP_CIPHER_TYPE));
}

/* CE Control Register                                                       */
/* ------------------------------------------------------------------------- */
/* | 31 - 3   | 2               | 1                       | 0              | */
/* ------------------------------------------------------------------------- */
/* | Reserved | Replay Detection Enable | TKIP MIC Enable | FSM Soft Reset | */
/* ------------------------------------------------------------------------- */
/*                                                                           */
/* TKIP MIC Enable          0 - Feature is disabled                          */
/*                          1 - Feature is enabled                           */
/* Replay Detection Enable  0 - Replay detection feature is disabled         */
/*                          1 - Replay detection feature is enabled          */
/* FSM Soft Reset           S/w should write a '1' and then a '0' - Not Used */

INLINE void enable_machw_ce_replay_det(void)
{
    //chenq mod
    //rMAC_CE_CONTROL |= REGBIT2;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_CE_CONTROL ) | (REGBIT2) ,
        (UWORD32)rMAC_CE_CONTROL );
}

INLINE void disable_machw_ce_replay_det(void)
{
    //chenq mod
    //rMAC_CE_CONTROL &= ~REGBIT2;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_CE_CONTROL ) & (~REGBIT2) ,
        (UWORD32)rMAC_CE_CONTROL );
}

INLINE void enable_machw_ce_tkip_mic(void)
{
    //chenq mod
    //rMAC_CE_CONTROL |= REGBIT1;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_CE_CONTROL ) | (REGBIT1) ,
        (UWORD32)rMAC_CE_CONTROL );
}

INLINE void disable_machw_ce_tkip_mic(void)
{
    //chenq mod
    //rMAC_CE_CONTROL &= ~REGBIT1;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_CE_CONTROL ) & (~REGBIT1) ,
        (UWORD32)rMAC_CE_CONTROL );
}

/* This function reads the TKIP Replay Count from the register */
INLINE UWORD32 get_ce_tkip_replay_fail_cnt(void)
{
    //chenq mod
    //return (convert_to_le(rMAC_CE_TKIP_REPLAY_FAIL_CNT));
    return (convert_to_le(host_read_trout_reg( 
                            (UWORD32)rMAC_CE_TKIP_REPLAY_FAIL_CNT)));
}

/* This function reads the CCMP Replay Count from the register */
INLINE UWORD32 get_ce_ccmp_replay_fail_cnt(void)
{
    //chenq mod
    //return (convert_to_le(rMAC_CE_CCMP_REPLAY_FAIL_CNT));
    return (convert_to_le(host_read_trout_reg( 
                            (UWORD32)rMAC_CE_CCMP_REPLAY_FAIL_CNT)));
}

/* This function configures CE in BSS-AP mode */
INLINE void set_machw_ce_ap_mode(void)
{
    //chenq mod
    //rMAC_CE_CONFIG &= MASK_INVERSE(2, 1);
    //rMAC_CE_CONFIG |= REGBIT2;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_CE_CONFIG ) & (MASK_INVERSE(2, 1)) ,
        (UWORD32)rMAC_CE_CONFIG );
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_CE_CONFIG ) | (REGBIT2) ,
        (UWORD32)rMAC_CE_CONFIG );
#ifdef DISABLE_CE_CLKGATING
    /* In Access Point mode due to a Hw bug, CE abort occurs after Beacon Tx.*/
    /* This affects the Hw LUT controller and results in Sw hang sfter LUT   */
    /* operation. To fix this CE clock gating is disabled in AP mode         */
    /* The hang issue can exist in IBSS mode also, but as IBSS+Security is   */
    /* not supported, CE clock gating is currently not disabled in IBSS mode */

    //chenq mod
    //rMAC_CE_CONFIG |= REGBIT3;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_CE_CONFIG ) | (REGBIT3) ,
        (UWORD32)rMAC_CE_CONFIG );

#endif /* DISABLE_CE_CLKGATING */
}

/* This function configures CE in BSS-STA mode */
INLINE void set_machw_ce_bss_sta_mode(void)
{
    //chenq mod
    //rMAC_CE_CONFIG &= MASK_INVERSE(2, 1);
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_CE_CONFIG ) & (MASK_INVERSE(2, 1)) ,
        (UWORD32)rMAC_CE_CONFIG );
#ifdef DISABLE_CE_CLKGATING
    /* Disable Clock-Gating in STA mode. Enabling this was leading to hang */
    /* in CE-LUT interface under heavy traffic conditions                  */

    //chenq mod
    //rMAC_CE_CONFIG |= REGBIT3;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_CE_CONFIG ) | (REGBIT3) ,
        (UWORD32)rMAC_CE_CONFIG );
#endif /* DISABLE_CE_CLKGATING */
}

/* This function configures CE in IBSS-STA mode */
INLINE void set_machw_ce_ibss_sta_mode(void)
{
    //chenq mod
    //rMAC_CE_CONFIG &= MASK_INVERSE(2, 1);
    //rMAC_CE_CONFIG |= REGBIT1;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_CE_CONFIG ) & (MASK_INVERSE(2, 1)) ,
        (UWORD32)rMAC_CE_CONFIG );
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_CE_CONFIG ) | (REGBIT1) ,
        (UWORD32)rMAC_CE_CONFIG );
#ifdef DISABLE_CE_CLKGATING
    /* Disable Clock-Gating in STA mode. Enabling this was leading to hang */
    /* in CE-LUT interface under heavy traffic conditions                  */

    //rMAC_CE_CONFIG |= REGBIT3;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_CE_CONFIG ) | (REGBIT3) ,
        (UWORD32)rMAC_CE_CONFIG );
#endif /* DISABLE_CE_CLKGATING */
}

INLINE void enable_machw_ce_features(void)
{
    enable_ce();
    enable_machw_ce_tkip_mic();
}

INLINE void disable_machw_ce_features(void)
{
    disable_ce();
    disable_machw_ce_tkip_mic();
    disable_machw_ce_replay_det();
}

/* This function flushes all CE-LUT entries in MAC H/w */
INLINE void flush_ce_lut(void)
{
    UWORD8 sta_index = 0;
	/*junbin.wang add 20131206*/
	UWORD8 data_0[6] = {0,0,0,0,0,0};
    UWORD8 data_1[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    UWORD8 data_2[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

    /* If Cipher Engine was used last time, then clear all the entries */
    for(sta_index =0; sta_index < NUM_STA_SUPPORTED; sta_index++)
            machw_ce_del_key(sta_index);
    
    //chenq add for flash GTK 2013-11-06
    #if 0
	/*cipher_type = 0, wep*/
	machw_ce_add_key((UWORD8)0, RX_GTK0_NUM, (UWORD8)0,
          (UWORD8)0, (UWORD8)0, data_1,
          data_0, SUPP_KEY,
          data_2);

    machw_ce_add_key((UWORD8)0, RX_GTK1_NUM, (UWORD8)0,
          (UWORD8)0, (UWORD8)0, data_1,
          data_0, SUPP_KEY,
          data_2);
	#else
	/*cipher_type = 1, wpa*/
    machw_ce_add_key((UWORD8)0, RX_GTK0_NUM, (UWORD8)0,
          (UWORD8)0, (UWORD8)1, data_1,
          data_0, SUPP_KEY,
          data_2);

    machw_ce_add_key((UWORD8)0, RX_GTK1_NUM, (UWORD8)0,
          (UWORD8)0, (UWORD8)1, data_1,
          data_0, SUPP_KEY,
          data_2);


	#endif
}

#endif /* CE_LUT_H */
