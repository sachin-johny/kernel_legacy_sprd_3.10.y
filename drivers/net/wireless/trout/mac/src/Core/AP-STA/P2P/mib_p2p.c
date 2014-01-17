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
/*  File Name         : mib_p2p.c                                            */
/*                                                                           */
/*  Description       : This file contains the P2P MIB structure and         */
/*                      initialisation                                       */
/*                                                                           */
/*  List of Functions : initialize_mac_mib_p2p                               */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/
#ifdef MAC_P2P
/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "mib_p2p.h"
#include "phy_hw_if.h"

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/
p_mib_p2p_t P_MIB_P2P = {0,};
BOOL_T g_p2p_opp_ps = BFALSE;// 20120709 caisf add, merged ittiam mac v1.2 code

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : initialize_mac_mib_p2p                                */
/*                                                                           */
/*  Description      : This function initializes all private MIB entries     */
/*                     for P2P                                               */
/*                                                                           */
/*  Inputs           : None                                                  */
/*                                                                           */
/*  Globals          : P_MIB_P2P                                             */
/*                                                                           */
/*  Processing       : All the MIB entries are initialized to their default  */
/*                     values.                                               */
/*                                                                           */
/*  Outputs          : The various entries of the global MIB structure are   */
/*                     set to default values.                                */
/*                                                                           */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void initialize_private_mib_p2p(void)
{
	TROUT_FUNC_ENTER;
    P_MIB_P2P.p2penable       = BFALSE;
    P_MIB_P2P.p2pdiscover     = BTRUE;
    P_MIB_P2P.p2ppersistgrp   = BFALSE;
    P_MIB_P2P.persistReconn   = BFALSE;
    P_MIB_P2P.p2pautoGO       = BFALSE;
    P_MIB_P2P.GOintrabss      = BTRUE;
    if(BFALSE == g_reset_mac_in_progress)
        P_MIB_P2P.devlmt      = BFALSE;
    P_MIB_P2P.inframang       = BFALSE;
    P_MIB_P2P.invitproc       = BTRUE;
    P_MIB_P2P.crsconnect      = BFALSE;
    P_MIB_P2P.GO              = BFALSE;
    P_MIB_P2P.grpformation    = BFALSE;
    P_MIB_P2P.grplmt          = BFALSE;
    P_MIB_P2P.p2p_listen_chan = CHANNEL1;
    P_MIB_P2P.p2p_oper_chan   = CHANNEL1;
    P_MIB_P2P.GOintentval     = 0;
    P_MIB_P2P.CTwindow        = 0;
    P_MIB_P2P.config_timeout  = 255;  /* 2.55 Sec */
    P_MIB_P2P.p2p_listen_mode = 0;// 20120709 caisf add, merged ittiam mac v1.2 code
    mem_set(P_MIB_P2P.p2ptrgtid, 0, 6);
    mem_set(P_MIB_P2P.p2pinvtid, 0, 6);
	TROUT_FUNC_EXIT;
}

//chenq for trace mib
void mib_p2p_print(void)
{
	PRINT_MIB("/************************************************************/\n");
	PRINT_MIB("/* mib in p2p                                               */\n");
	PRINT_MIB("/************************************************************/\n");
	PRINT_MIB("P_MIB_P2P.p2penable       = %s\n",(P_MIB_P2P.p2penable     == BTRUE) ? "BTRUE" : "BFALSE");
    PRINT_MIB("P_MIB_P2P.p2pdiscover     = %s\n",(P_MIB_P2P.p2pdiscover   == BTRUE) ? "BTRUE" : "BFALSE");
    PRINT_MIB("P_MIB_P2P.p2ppersistgrp   = %s\n",(P_MIB_P2P.p2ppersistgrp == BTRUE) ? "BTRUE" : "BFALSE");
	PRINT_MIB("P_MIB_P2P.persistReconn   = %s\n",(P_MIB_P2P.persistReconn == BTRUE) ? "BTRUE" : "BFALSE");
	PRINT_MIB("P_MIB_P2P.p2pautoGO       = %s\n",(P_MIB_P2P.p2pautoGO     == BTRUE) ? "BTRUE" : "BFALSE");
	PRINT_MIB("P_MIB_P2P.GOintrabss      = %s\n",(P_MIB_P2P.GOintrabss    == BTRUE) ? "BTRUE" : "BFALSE");
	PRINT_MIB("P_MIB_P2P.devlmt          = %s\n",(P_MIB_P2P.devlmt        == BTRUE) ? "BTRUE" : "BFALSE");
	PRINT_MIB("P_MIB_P2P.inframang       = %s\n",(P_MIB_P2P.inframang     == BTRUE) ? "BTRUE" : "BFALSE");
	PRINT_MIB("P_MIB_P2P.invitproc       = %s\n",(P_MIB_P2P.invitproc     == BTRUE) ? "BTRUE" : "BFALSE");
	PRINT_MIB("P_MIB_P2P.crsconnect      = %s\n",(P_MIB_P2P.crsconnect    == BTRUE) ? "BTRUE" : "BFALSE");
	PRINT_MIB("P_MIB_P2P.GO              = %s\n",(P_MIB_P2P.GO            == BTRUE) ? "BTRUE" : "BFALSE");
	PRINT_MIB("P_MIB_P2P.grpformation    = %s\n",(P_MIB_P2P.grpformation  == BTRUE) ? "BTRUE" : "BFALSE");
	PRINT_MIB("P_MIB_P2P.grplmt          = %s\n",(P_MIB_P2P.grplmt        == BTRUE) ? "BTRUE" : "BFALSE");
	PRINT_MIB("P_MIB_P2P.p2p_listen_chan = CHANNEL%d\n",P_MIB_P2P.p2p_listen_chan+1);
	PRINT_MIB("P_MIB_P2P.p2p_oper_chan   = CHANNEL%d\n",P_MIB_P2P.p2p_oper_chan+1);
	PRINT_MIB("P_MIB_P2P.GOintentval     = 0x%02x\n",P_MIB_P2P.GOintentval);
	PRINT_MIB("P_MIB_P2P.CTwindow        = 0x%02x\n",P_MIB_P2P.CTwindow);
	PRINT_MIB("P_MIB_P2P.config_timeout  = 0x%02x\n",P_MIB_P2P.config_timeout);
	PRINT_MIB("P_MIB_P2P.p2ptrgtid       = %02x:%02x:%02x:%02x:%02x:%02x\n",P_MIB_P2P.p2ptrgtid[0]
																			,P_MIB_P2P.p2ptrgtid[1]
																			,P_MIB_P2P.p2ptrgtid[2]
																			,P_MIB_P2P.p2ptrgtid[3]
																			,P_MIB_P2P.p2ptrgtid[4]
																			,P_MIB_P2P.p2ptrgtid[5]);
	PRINT_MIB("P_MIB_P2P.p2pinvtid       = %02x:%02x:%02x:%02x:%02x:%02x\n",P_MIB_P2P.p2pinvtid[0]
																			,P_MIB_P2P.p2pinvtid[1]
																			,P_MIB_P2P.p2pinvtid[2]
																			,P_MIB_P2P.p2pinvtid[3]
																			,P_MIB_P2P.p2pinvtid[4]
																			,P_MIB_P2P.p2pinvtid[5]);
	PRINT_MIB("\n");
}

#endif /* MAC_P2P */
