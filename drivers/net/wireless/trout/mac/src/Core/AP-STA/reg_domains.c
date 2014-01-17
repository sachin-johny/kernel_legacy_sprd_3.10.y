/*****************************************************************************/
/*                                                                           */
/*                     Ittiam 802.11 MAC SOFTWARE                            */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2011                               */
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
/*  File Name         : reg_domains.c                                        */
/*                                                                           */
/*  Description       : This file contains the functions for Regulatory      */
/*                      Domains support.                                     */
/*                                                                           */
/*  List of Functions : init_supp_reg_domain                                 */
/*                      update_curr_rc_max_tx_pow                            */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "reg_domains.h"
#include "core_mode_if.h"

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

UWORD8 g_current_reg_domain = NO_REG_DOMAIN;
UWORD8 g_current_reg_class  = UNKNOWN_REG_CLASS;
// config default channel number, caisf 20130506
UWORD8 g_scan_req_channel_no = 14; 

// 20120830 caisf mod, merged ittiam mac v1.3 code
#if 0
curr_tx_power_info_t g_curr_tx_power = {{0, }, };
#else
UWORD8 g_new_reg_class      = INVALID_REG_CLASS;

curr_tx_power_info_t g_default_tx_power  = {{0, }, };
curr_tx_power_info_t g_user_tx_power     = {{0, }, };
curr_tx_power_info_t g_reg_max_tx_power  = {{0, }, };
#endif

/*****************************************************************************/
/*                                                                           */
/*  Function Name : init_supp_reg_domain                                     */
/*                                                                           */
/*  Description   : This function initializes the supported Regulatory       */
/*                  Domain configuration.                                    */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The regulatory domain information table is initialized   */
/*                  as per support available in the device. This is RF       */
/*                  dependent and therefore RF I/f function is used.         */
/*                  The current Regulatory Domain is then initialized based  */
/*                  on the mode of operation (Core Mode-IF function)         */
/*                  Called from "initialize_macsw" to initialize the info    */
/*                  whenever operation starts.                               */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void init_supp_reg_domain(void)
{
	TROUT_FUNC_ENTER;
	init_supp_reg_domain_rf();
	init_current_reg_domain();
	TROUT_FUNC_EXIT;
}

// 20120830 caisf add, merged ittiam mac v1.3 code
/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_curr_rc_max_tx_pow                                */
/*                                                                           */
/*  Description   : This function updates current Regulatory Class Max Tx    */
/*                  power                                                    */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function is called whenever current regulatory class*/
/*                  is configured. This function is also called from TPC     */
/*                  (11h protocol) whenever local Max power changes.         */
/*                  Structure 'g_reg_max_tx_power' power levels a, b, n and  */
/*                  n40 are updated with the minimum of RC Max RF Tx         */
/*                  Power and Regulatory Max power.                          */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void update_curr_rc_max_tx_pow(void)
{
    UWORD8 gaincode   = 0;
    UWORD8 dbm        = 0;
    UWORD8 freq       = get_current_start_freq();
    UWORD8 reg_mx_pow = 0;
    UWORD8 chan       = mget_CurrentChannel();
    UWORD8 reg_class  = get_current_reg_class();
    reg_class_info_t *rc_info = NULL;

    rc_info    = get_reg_class_info(freq, reg_class, get_current_reg_domain());

    if(rc_info == NULL)
    {
        PRINTD("MultiDomain Warning: NULL Reg Class Info\n\r");
        return;
    }

    reg_mx_pow = get_rc_max_reg_tx_pow(rc_info);

    dbm = MIN(get_rc_max_rf_tx_pow_a(rc_info), reg_mx_pow);

    /* Convert the TX power dBm to appropriate Gain code value */
    gaincode = convert_dbm_to_regval(dbm, freq, chan);

    /* Set the current TX power dBm and Gain code values */
    g_reg_max_tx_power.pow_a.dbm      = dbm;
    g_reg_max_tx_power.pow_a.gaincode = gaincode;

    dbm = MIN(get_rc_max_rf_tx_pow_b(rc_info), reg_mx_pow);

     /* Convert the TX power dBm to appropriate Gain code value */
    gaincode = convert_dbm_to_regval(dbm, freq, chan);

    /* Set the current TX power dBm and Gain code values */
    g_reg_max_tx_power.pow_b.dbm      = dbm;
    g_reg_max_tx_power.pow_b.gaincode = gaincode;

    dbm = MIN(get_rc_max_rf_tx_pow_n(rc_info), reg_mx_pow);

     /* Convert the TX power dBm to appropriate Gain code value */
    gaincode = convert_dbm_to_regval(dbm, freq, chan);

    /* Set the current TX power dBm and Gain code values */
    g_reg_max_tx_power.pow_n.dbm      = dbm;
    g_reg_max_tx_power.pow_n.gaincode = gaincode;

}
