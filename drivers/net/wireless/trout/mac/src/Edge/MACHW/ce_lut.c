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
/*  File Name         : ce_lut.c                                             */
/*                                                                           */
/*  Description       : This file contains the utility functions required    */
/*                      by the MAC CE LUT interface                          */
/*                                                                           */
/*  List of Functions : initialize_lut                                       */
/*                      machw_ce_del_key                                     */
/*                      machw_ce_add_key                                     */
/*                      get_machw_ce_pn_val                                  */
/*                      machw_ce_init_rx_bcmc_pn_val                         */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "ce_lut.h"
#include "spi_interface.h"
/*****************************************************************************/
/*                                                                           */
/*  Function Name    : machw_ce_del_key                                      */
/*                                                                           */
/*  Description      : This function deletes a key from the LUT.             */
/*                                                                           */
/*  Inputs           : sta_index                                             */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function deletes keys from the LUT, for the      */
/*                     addresses entries that are marked valid. Hence only   */
/*                     reception from these will be discarded So the WEP     */
/*                     keys cannot be deleted once programmed, but can be    */
/*                     over written                                          */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : BTRUE  - CE LUT Update Successful                     */
/*                   : BFALSE - CE LUT Update Failed                         */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

BOOL_T machw_ce_del_key(UWORD8 sta_index)
{
    UWORD8  cnt     = 0;
    UWORD32 temp    = 0;
    BOOL_T  ustatus = BTRUE;

	/*junbinwang add for ce config clock gating 20131009.bug 218429*/
	disable_machw_ce_clkgating();

    for(cnt = 0; cnt < NUM_CE_LUT_UPDATE_ATTEMPTS; cnt++)
    {
        ustatus = BTRUE;

        //chenq mod
        //temp = convert_to_le(rMAC_CE_LUT_OPERN);
        temp = convert_to_le(host_read_trout_reg( 
                                (UWORD32)rMAC_CE_LUT_OPERN));

        /* Update the LUT Operation */
        temp = (temp & 0xFFFFFCFF) | LUT_REMOVE_OPERN;

        /* Update the Key ID to zero */
        temp = (temp & 0xFFFFF3FF);

        /* Update the LUT index */
        temp = (temp & 0xFF83FFFF) | (((sta_index & 0x1F) << 18) & 0x007C0000);

        /* Update the ADDR/KEY Programing mask to zero */
        temp = temp & 0xFFFFCFFF;

        //chenq mod
        //rMAC_CE_LUT_OPERN = convert_to_le(temp);
        host_write_trout_reg( convert_to_le(temp), 
                              (UWORD32)rMAC_CE_LUT_OPERN );

        //rMAC_CE_LUT_OPERN |= REGBIT0;
        host_write_trout_reg(
            host_read_trout_reg( (UWORD32)rMAC_CE_LUT_OPERN ) | (REGBIT0) ,
            (UWORD32)rMAC_CE_LUT_OPERN );

        temp = 0;
        /* Poll for the LUT_EN Bit to be reset */

        //chenq mod
        //while(rMAC_CE_LUT_OPERN & REGBIT0)
        while(host_read_trout_reg( (UWORD32)rMAC_CE_LUT_OPERN) & REGBIT0)
        {
            add_calib_delay(1);
            temp++;

           /* Wait for sometime for the CE-LUT update operation to complete */
            if(temp > CE_LUT_UPDATE_TIMEOUT)
            {
#ifdef DEBUG_MODE
            PRINTD("HwEr: CeLutDelFail\n\r");
            g_mac_stats.celutdelfail++;
#endif /* DEBUG_MODE */

                ustatus = BFALSE;
                break;
            }
        }

        if(ustatus == BTRUE)
            break;
    }

    if(ustatus == BFALSE)
    {
        raise_system_error(CE_UPDATE_FAIL);
    }
	/*junbinwang add for ce config clock gating 20131009.bug 218429*/
	enable_machw_ce_clkgating();
    return ustatus;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : machw_ce_add_key                                      */
/*                                                                           */
/*  Description      : This function adds the LUT entry for the given index. */
/*                                                                           */
/*  Inputs           : 1) Key identifier                                     */
/*                     2) Key number                                         */
/*                     3) Key index                                          */
/*                     4) Mask                                               */
/*                     5) Cipher type                                        */
/*                     6) Pointer to the key                                 */
/*                     7) Pointer to the STA address                         */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function adds the LUT entry for the given index  */
/*                     If the cipher type is wep, only then the field mask   */
/*                     is used by the CE to either program the entry.        */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : BTRUE  - CE LUT Update Successful                     */
/*                   : BFALSE - CE LUT Update Failed                         */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

BOOL_T machw_ce_add_key(UWORD8 key_id, KEY_NUMBER_T key_num, UWORD8 sta_index,
                        UWORD8 mask, UWORD8 cipher_type, UWORD8 *cip_key,
                        UWORD8 *sta_addr, KEY_ORIGIN_T key_origin,
                        UWORD8 *mic_key)
{
    UWORD8  cnt     = 0;
    UWORD8 *key     = NULL;
    UWORD32 temp    = 0;
    BOOL_T  ustatus = BTRUE;


	/*junbinwang add for ce config clock gating 20131009.bug 218429*/
	disable_machw_ce_clkgating();
    /* Enable replay detection before adding key to ensure LUT clearing */
    enable_machw_ce_replay_det();

    for(cnt = 0; cnt < NUM_CE_LUT_UPDATE_ATTEMPTS; cnt++)
    {

        ustatus = BTRUE;

        //chenq mod
        //temp = convert_to_le(rMAC_CE_LUT_OPERN);
        temp = convert_to_le(host_read_trout_reg( (UWORD32)rMAC_CE_LUT_OPERN));

        /* Update the Cipher Suite Type */
        temp = (temp & 0xFFFFFFC7) | (((cipher_type & 0x07) << 3) & 0x38);

        /* Update the Key number */
        temp = (temp & 0xFFFFFF3F) | (((key_num & 0x03) << 6) & 0x000000C0);

        /* Update the LUT Operation */
        temp = (temp & 0xFFFFFCFF) | LUT_WRITE_OPERN;

        /* Update the Key ID */
        temp = (temp & 0xFFFFF3FF) | (((key_id & 0x03) << 10) & 0x00000C00);

        /* Update the LUT index */
        temp = (temp & 0xFF83FFFF) | (((sta_index & 0x1F) << 18) & 0x007C0000);

        /* Update the Authentication/Supplicant bit */
        temp = (temp & 0xFF7FFFFF) | (key_origin << 23);

        /* Update the ADDR/KEY Programing mask */
        temp = (temp & 0xFFFFCFFF) | (((mask & 0x03) << 12) & 0x00003000);

        //chenq mod
        //rMAC_CE_LUT_OPERN = convert_to_le(temp);
        host_write_trout_reg( convert_to_le(temp), 
                              (UWORD32)rMAC_CE_LUT_OPERN );

        /* Set the cipher key */
        key  = cip_key;
        temp = (0xFF & key[3]) | ((0xFF & key[2]) << 8) |
                ((0xFF & key[1]) << 16) | ((0xFF & key[0]) << 24);
        key += 4;
        //chenq mod
        //rMAC_CE_KEY_FIRST = convert_to_le(temp);
        host_write_trout_reg( convert_to_le(temp), 
                              (UWORD32)rMAC_CE_KEY_FIRST );

        temp = (0xFF & key[3]) | ((0xFF & key[2]) << 8) |
                ((0xFF & key[1]) << 16) | ((0xFF & key[0]) << 24);
        key += 4;
        //chenq mod
        //rMAC_CE_KEY_SECOND = convert_to_le(temp);
        host_write_trout_reg( convert_to_le(temp), 
                              (UWORD32)rMAC_CE_KEY_SECOND );

        temp = (0xFF & key[3]) | ((0xFF & key[2]) << 8) |
                ((0xFF & key[1]) << 16) | ((0xFF & key[0]) << 24);
        key += 4;
        //chenq mod
        //rMAC_CE_KEY_THIRD = convert_to_le(temp);
        host_write_trout_reg( convert_to_le(temp), 
                              (UWORD32)rMAC_CE_KEY_THIRD );

        temp = (0xFF & key[3]) | ((0xFF & key[2]) << 8) |
                ((0xFF & key[1]) << 16) | ((0xFF & key[0]) << 24);
        key += 4;
        //chenq mod
        //rMAC_CE_KEY_FOURTH = convert_to_le(temp);
        host_write_trout_reg( convert_to_le(temp), 
                             (UWORD32)rMAC_CE_KEY_FOURTH );

        /* Set Micheal Key */
        if(mic_key != NULL)
        {
            key  = mic_key;
            temp = (0xFF & key[0]) | ((0xFF & key[1]) << 8) |
                    ((0xFF & key[2]) << 16) | ((0xFF & key[3]) << 24);
            key += 4;
            //chenq mod
            //rMAC_CE_TKIP_MIC_KEY_Q4 = convert_to_le(temp);
            host_write_trout_reg( convert_to_le(temp), 
                                  (UWORD32)rMAC_CE_TKIP_MIC_KEY_Q4 );

            temp = (0xFF & key[0]) | ((0xFF & key[1]) << 8) |
                    ((0xFF & key[2]) << 16) | ((0xFF & key[3]) << 24);
            key += 4;
            //chenq mod
            //rMAC_CE_TKIP_MIC_KEY_Q3 = convert_to_le(temp);
            host_write_trout_reg( convert_to_le(temp), 
                                  (UWORD32)rMAC_CE_TKIP_MIC_KEY_Q3 );

            temp = (0xFF & key[0]) | ((0xFF & key[1]) << 8) |
                    ((0xFF & key[2]) << 16) | ((0xFF & key[3]) << 24);
            key += 4;
            //chenq mod    
            //rMAC_CE_TKIP_MIC_KEY_Q2 = convert_to_le(temp);
            host_write_trout_reg( convert_to_le(temp), 
                                  (UWORD32)rMAC_CE_TKIP_MIC_KEY_Q2 );

            temp = (0xFF & key[0]) | ((0xFF & key[1]) << 8) |
                    ((0xFF & key[2]) << 16) | ((0xFF & key[3]) << 24);
            key += 4;
            //chenq mod
            //rMAC_CE_TKIP_MIC_KEY_Q1 = convert_to_le(temp);
            host_write_trout_reg( convert_to_le(temp), 
                                  (UWORD32)rMAC_CE_TKIP_MIC_KEY_Q1 );
       }

        /* Set the STA Address */
        temp = (sta_addr[1] | ((sta_addr[0] << 8) & 0x0000FF00));
        //chenq mod
        //rMAC_CE_STA_ADDR_MSB = convert_to_le(temp);
        host_write_trout_reg( convert_to_le(temp), 
                              (UWORD32)rMAC_CE_STA_ADDR_MSB );

        temp = (sta_addr[5]
            | ((sta_addr[4] << 8) & 0x0000FF00)
            | ((sta_addr[3] << 16) & 0x00FF0000)
            | ((sta_addr[2] << 24) & 0xFF000000));

        //chenq mod
        //rMAC_CE_STA_ADDR_LSB = convert_to_le(temp);

        //rMAC_CE_LUT_OPERN |= REGBIT0;

        host_write_trout_reg( convert_to_le(temp), (UWORD32)rMAC_CE_STA_ADDR_LSB );
        host_write_trout_reg(host_read_trout_reg( (UWORD32)rMAC_CE_LUT_OPERN)
                                                    | (REGBIT0), 
                             (UWORD32)rMAC_CE_LUT_OPERN );

        temp = 0;

        /* Poll for the LUT_EN Bit to be reset */

        //chenq mod
        //while(rMAC_CE_LUT_OPERN & REGBIT0)
        while(host_read_trout_reg( (UWORD32)rMAC_CE_LUT_OPERN) & REGBIT0)
        {
            add_calib_delay(1);
            temp++;

           /* Wait for sometime for the CE-LUT update operation to complete */
            if(temp > CE_LUT_UPDATE_TIMEOUT)
            {
#ifdef DEBUG_MODE
                PRINTD("HwEr:CeLutAddFail\n\r");
                g_mac_stats.celutaddfail++;
#endif /* DEBUG_MODE */

                ustatus = BFALSE;
                break;
            }
        }

        if(ustatus == BTRUE)
            break;
    }

    if(is_ce_enabled() == BFALSE)
    {
        enable_machw_ce_features();
    }

    if(ustatus == BFALSE)
        raise_system_error(CE_UPDATE_FAIL);
	/*junbinwang add for ce config clock gating 20131009.bug 218429*/
	enable_machw_ce_clkgating();
    return ustatus;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : machw_ce_init_rx_bcmc_pn_val                          */
/*                                                                           */
/*  Description      : This function adds the LUT entry for the given index. */
/*                                                                           */
/*  Inputs           : 1) TID                                                */
/*                     2) Station index                                      */
/*                     3) Pointer to the PN/TSC Value                        */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function initializes BC/MC Rx PN/TSC value       */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : BTRUE  - CE LUT Update Successful                     */
/*                   : BFALSE - CE LUT Update Failed                         */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

BOOL_T machw_ce_init_rx_bcmc_pn_val(UWORD8 tid, UWORD8 lut_index, UWORD8 *pn)
{
    UWORD8  cnt     = 0;
    UWORD32 temp    = 0;
    BOOL_T  ustatus = BTRUE;



	/*junbinwang add for ce config clock gating 20131009.bug 218429*/
	disable_machw_ce_clkgating();
    for(cnt = 0; cnt < NUM_CE_LUT_UPDATE_ATTEMPTS; cnt++)
    {

        ustatus = BTRUE;

        if(pn != NULL)
        {
            temp = (pn[0] | (pn[1] << 8) | (pn[2] << 16) | (pn[3] << 24));
            //chenq mod
            //rMAC_CE_RX_BC_PN_LSB = convert_to_le(temp);
            host_write_trout_reg( convert_to_le(temp), 
                                  (UWORD32)rMAC_CE_RX_BC_PN_LSB );

            temp = (pn[4] | (pn[5] << 8));
            //chenq mod
            //rMAC_CE_RX_BC_PN_MSB = convert_to_le(temp);
            host_write_trout_reg( convert_to_le(temp), 
                                  (UWORD32)rMAC_CE_RX_BC_PN_MSB );

        }
        else
        {
            //chenq mod
            //rMAC_CE_RX_BC_PN_LSB = 0;
            //rMAC_CE_RX_BC_PN_MSB = 0;
            host_write_trout_reg( 0, (UWORD32)rMAC_CE_RX_BC_PN_LSB );
            host_write_trout_reg( 0, (UWORD32)rMAC_CE_RX_BC_PN_MSB );
        }

        //chenq mod
        //temp = convert_to_le(rMAC_CE_LUT_OPERN);
        temp = convert_to_le(host_read_trout_reg( (UWORD32)rMAC_CE_LUT_OPERN  ));

        /* Update the LUT index */
        temp = (temp & 0xFF83FFFF) | (((lut_index & 0x1F) << 18) & 0x007C0000);

        /* Update the TID */
        //temp = (temp & 0xF0FFFFFF) | (((tid & 0xF) << 24) & 0x0F000000);
        temp = (temp & 0xE1FFFFFF) | (((tid & 0xF) << 25) & 0x1E000000);

        //chenq mod
        //rMAC_CE_LUT_OPERN = convert_to_le(temp);
        //rMAC_CE_LUT_OPERN |= REGBIT24;
        host_write_trout_reg( convert_to_le(temp), 
					          (UWORD32)rMAC_CE_LUT_OPERN );

        host_write_trout_reg(
            host_read_trout_reg( (UWORD32)rMAC_CE_LUT_OPERN ) | (REGBIT24) ,
            (UWORD32)rMAC_CE_LUT_OPERN );

        temp = 0;

        /* Wait for the update to be complete */

        //chenq mod
        //while(rMAC_CE_LUT_OPERN & REGBIT24)
        while(host_read_trout_reg( (UWORD32)rMAC_CE_LUT_OPERN) & REGBIT24)
        {
            add_calib_delay(1);
            temp++;

            /* Wait for sometime for the CE-LUT update operation to complete */
            if(temp > CE_LUT_UPDATE_TIMEOUT)
            {
#ifdef DEBUG_MODE
                PRINTD("HwEr:CeLutPnInitFail\n\r");
                g_mac_stats.celutpnfail++;
#endif /* DEBUG_MODE */

                ustatus = BFALSE;
                break;
            }
        }

        if(ustatus == BTRUE)
            break;
    }

    if(ustatus == BFALSE)
        raise_system_error(CE_UPDATE_FAIL);
	/*junbinwang add for ce config clock gating 20131009.bug 218429*/
	enable_machw_ce_clkgating();
    return ustatus;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : get_machw_ce_pn_val                                   */
/*                                                                           */
/*  Description      : This function reads the CE PN Value from MAC HW       */
/*                                                                           */
/*  Inputs           : Pointer to the location where value to be read to     */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void get_machw_ce_pn_val(UWORD8* val)
{
    UWORD32 pn_h = 0;
    UWORD32 pn_l = 0;

    /* Read the value from the registers */

    //chenq mod
    //pn_h = convert_to_le(rMAC_CE_GTK_PN_MSB);
    //pn_l = convert_to_le(rMAC_CE_GTK_PN_LSB);
    pn_h = convert_to_le(host_read_trout_reg( (UWORD32)rMAC_CE_GTK_PN_MSB));
    pn_l = convert_to_le(host_read_trout_reg( (UWORD32)rMAC_CE_GTK_PN_LSB));

    /* Write the values */
    val[5] = (pn_h & 0x0000FF00) >> 8;
    val[4] = (pn_h & 0x000000FF);
    val[3] = (pn_l & 0xFF000000) >> 24;
    val[2] = (pn_l & 0x00FF0000) >> 16;
    val[1] = (pn_l & 0x0000FF00) >> 8;
    val[0] = (pn_l & 0x000000FF);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : initialize_lut                                        */
/*                                                                           */
/*  Description      : This function initializes the CE LUT                  */
/*                                                                           */
/*  Inputs           : None                                                  */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void initialize_lut(void)
{
	TROUT_FUNC_ENTER;
    /* CE Register Initialization */
#ifdef IBSS_BSS_STATION_MODE
	if(mget_DesiredBSSType() == INDEPENDENT)
	{
	    set_machw_ce_ibss_sta_mode();
	}
	else
	{
	    set_machw_ce_bss_sta_mode();
	}
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
	set_machw_ce_ap_mode();
#endif /* BSS_ACCESS_POINT_MODE */

	set_machw_ce_macaddr(mget_StationID());
	TROUT_FUNC_EXIT;
}
