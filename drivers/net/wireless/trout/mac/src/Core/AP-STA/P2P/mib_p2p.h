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
/*  File Name         : mib_p2p.h                                            */
/*                                                                           */
/*  Description       : This file contains the definitions and structures    */
/*                      related to MIB for P2P protocol                      */
/*                                                                           */
/*  List of Functions : Access functions for all P2P related MIB             */
/*                      parameters.                                          */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_P2P

#ifndef MIB_P2P_H
#define MIB_P2P_H
// 20120709 caisf mod, merged ittiam mac v1.2 code
/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "mib.h"

/* P2P MIB Structure - Common for GO and Client mode */
typedef struct
{
    UWORD8 p2penable;
    UWORD8 p2pdiscover;
    UWORD8 p2ppersistgrp;
    UWORD8 persistReconn;
    UWORD8 p2pautoGO;
    UWORD8 GOintrabss;
    UWORD8 devlmt;
    UWORD8 inframang;
    UWORD8 invitproc;
    UWORD8 crsconnect;
    UWORD8 GO;
    UWORD8 grpformation;
    UWORD8 grplmt;
    UWORD8 p2p_listen_chan;
    UWORD8 p2p_oper_chan;
    UWORD8 GOintentval;
    UWORD8 CTwindow;
    UWORD16 p2p_pref_configmethod; 
    UWORD8 config_timeout;
    UWORD8 p2p_listen_mode;
    UWORD8 p2ptrgtid[MAC_ADDRESS_LEN];
    UWORD8 p2pinvtid[MAC_ADDRESS_LEN];
} p_mib_p2p_t;

/*****************************************************************************/
/* External Global Variables                                                 */
/*****************************************************************************/
extern p_mib_p2p_t P_MIB_P2P;
extern BOOL_T g_p2p_opp_ps;

/*****************************************************************************/
/* External funtion declaration                                              */
/*****************************************************************************/
extern void initialize_private_mib_p2p(void);

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/* This funtion returns whether the p2p device supports service discovery or */
/* not.                                                                      */
INLINE BOOL_T mget_serv_discovery(void)
{
    return BFALSE; /* Havent been included yet.                              */
}


/* This funtion returns the p2p device capability to support concurrent      */
/* operation with WLAN.                                                      */
INLINE BOOL_T mget_conc_op(void)
{
    return BFALSE; /* Is not included yet */
}


/* This funtion sets the capability of the p2p interface of the p2p device   */
/* to be managed by the WLAN.                                                */
INLINE void mset_p2p_infra_managed(BOOL_T val)
{
    P_MIB_P2P.inframang = val;
}

/* This funtion returns the capability of the p2p interface of the p2p device*/
/* to be managed by the WLAN.                                                */
INLINE BOOL_T mget_p2p_infra_managed(void)
{
    return BFALSE; /* This feature is not being supported now */
}


/* This funtion sets the device limit of the p2p device   */
INLINE void mset_p2p_dev_lmt(BOOL_T val)
{
    P_MIB_P2P.devlmt = val;
}

/* This funtion returns the device limit of the p2p device */
INLINE BOOL_T mget_p2p_dev_lmt(void)
{
    return (BOOL_T)(P_MIB_P2P.devlmt) ;
}


/* This funtion sets the capability of the p2p device to process the p2p     */
/* invitaion procedure signalling.                                           */
INLINE void mset_p2p_invit_proc(BOOL_T val)
{
    P_MIB_P2P.invitproc = val;
}

/* This funtion returns the capability of the p2p device to process the p2p  */
/* invitaion procedure signalling.                                           */
INLINE BOOL_T mget_p2p_invit_proc(void)
{
    return (BOOL_T)(P_MIB_P2P.invitproc);
}

/* This funtion sets the MIB GO with the required value. It is BTRUE of the  */
/* device is the GO else BFALSE.                                             */
INLINE void mset_p2p_GO(BOOL_T val)
{
    P_MIB_P2P.GO = val;
}

/* This funtion returns if the device is Group Owner or not  */
INLINE BOOL_T mget_p2p_GO(void)
{
    return (BOOL_T)(P_MIB_P2P.GO);
}

/* This funtion sets the capability of the GO to add additional client */
INLINE void mset_p2p_grp_lmt(UWORD8 val)
{
    P_MIB_P2P.grplmt = val;
}

/* This funtion returns the capability of the GO to add additional client */
INLINE UWORD8 mget_p2p_grp_lmt(void)
{
    return (BOOL_T)(P_MIB_P2P.grplmt);
}


/* This funtion sets the capability of the GO to host cross connection       */
/* between P2P group and WLAN.                                               */
INLINE void mset_crs_connect(BOOL_T val)
{
    P_MIB_P2P.crsconnect = val;
}

/* This funtion returns the capability of the GO to host cross connection    */
/* between P2P group and WLAN.                                               */
INLINE BOOL_T mget_crs_connect(void)
{
    return BFALSE;
}



/* This funtion sets  whether the p2p device is operating as the GO in the */
/* provisioning phase of the group formation or not.                       */
INLINE void mset_grp_formation(BOOL_T val)
{
    P_MIB_P2P.grpformation = val;
}

/* This funtion returns whether the p2p device is operating as the GO in the */
/* provisioning phase of the group formation.                                */
INLINE BOOL_T mget_grp_formation(void)
{
    return (BOOL_T)(P_MIB_P2P.grpformation);
}

/* This function returns the pointer to the interface address of the P2P     */
/* device                                                                    */
INLINE UWORD8 *get_p2p_if_address(void)
{
    return mget_StationID();
}

/* This funtion sets the CTwindow.                                          */
INLINE void mset_p2p_CTW(UWORD8 val)
{
    P_MIB_P2P.CTwindow = val;
}

/* This funtion returns the CTwindow.                                        */
INLINE UWORD8 mget_p2p_CTW(void)
{
    return P_MIB_P2P.CTwindow;
}

/* This function returns the pointer to the group BSSID */
INLINE UWORD8 *get_p2p_grp_bssid(void)
{
    return mget_bssid();
}

/* This function returns the status for revinvoke persistent group */
INLINE BOOL_T get_reinvoke_persist_grp(void)
{
    return BFALSE;
}

/* This funtion sets the config timeout of the GO. */
INLINE void set_config_timeout(UWORD8 val)
{
    P_MIB_P2P.config_timeout = val;
}

/* This funtion returns the config timeout of the GO. */
INLINE UWORD8 get_config_timeout(void)
{
    return P_MIB_P2P.config_timeout;
}

/* This funtion sets the p2p feature enable value. */
INLINE void mset_p2p_enable(BOOL_T val)
{
    TROUT_DBG4("P2P: caisf p2p enable = %d\n", val);
    P_MIB_P2P.p2penable = val;
}

/* This function gets the current P2P feature enable status. */
INLINE BOOL_T mget_p2p_enable(void)
{
    return (BOOL_T)(P_MIB_P2P.p2penable);
}


/* This function sets the device to listen state. */
INLINE void mset_p2p_discoverable(BOOL_T val)
{
    P_MIB_P2P.p2pdiscover = val;
}

/* This function gets the state of the device if it is in listen state. */
INLINE BOOL_T mget_p2p_discoverable(void)
{
    return (BOOL_T)(P_MIB_P2P.p2pdiscover);
}

/* This function sets the status of the persistent group functionality,      */
/* whether the device would try to form a persistent group or not.           */
INLINE void mset_p2p_persist_grp(BOOL_T val)
{
    P_MIB_P2P.p2ppersistgrp = val;
}

/* This function gets the status of the persistent group functionality,      */
/* whether the device would try to form a persistent group or not.           */
INLINE BOOL_T mget_p2p_persist_grp(void)
{
    return (BOOL_T)(P_MIB_P2P.p2ppersistgrp);
}


/* This function sets the value for persistent reconnection */
INLINE void mset_presistReconn(BOOL_T val)
{
     P_MIB_P2P.persistReconn = val;
}

/* This function returns the value for persistent reconnection */
INLINE BOOL_T mget_presistReconn(void)
{
     return (BOOL_T)(P_MIB_P2P.persistReconn);
}


/* This function enables intra-BSS functionality in the GO. */
INLINE void mset_p2p_intra_bss(BOOL_T val)
{
    P_MIB_P2P.GOintrabss = val;
}

/* This function gets the status of intra-BSS functionality in GO. */
INLINE BOOL_T mget_p2p_intra_bss(void)
{
    return (BOOL_T)(P_MIB_P2P.GOintrabss);
}

/* This function returns the listen channel */
INLINE UWORD8 mget_p2p_listen_chan(void)
{
    return P_MIB_P2P.p2p_listen_chan;
}

/* This function sets the listen channel */
INLINE void mset_p2p_listen_chan(UWORD8 val)
{
    P_MIB_P2P.p2p_listen_chan = val;
}

/* This function returns the operating channel */
INLINE UWORD8 mget_p2p_oper_chan(void)
{
    return P_MIB_P2P.p2p_oper_chan;
}

/* This function sets the operating channel */
INLINE void mset_p2p_oper_chan(UWORD8 val)
{
    P_MIB_P2P.p2p_oper_chan = val;
}

/* This function sets the GO Intent Value.  */
INLINE void mset_p2p_GO_intent_val(UWORD8 val)
{
	/* Input Sanity Check */
    if(val > 15)
        return;

    P_MIB_P2P.GOintentval = val;
}

/* This function gets the GO Intent Value */
INLINE UWORD8 mget_p2p_GO_intent_val(void)
{
    return P_MIB_P2P.GOintentval;
}


/* This funtion gets the P2P device address */
INLINE UWORD8 *mget_p2p_dev_addr(void)
{
    return mget_StationID();
}

/* This funtion sets the P2P device address */
INLINE void mset_p2p_dev_addr(UWORD8 *addr)
{
    mset_StationID(addr);
}


/* This funtion sets the target P2P device id to which it needs to connect   */
INLINE void mset_p2p_trgt_dev_id(UWORD8 *inp)
{
    mac_addr_cpy(P_MIB_P2P.p2ptrgtid, inp);
}

/* This funtion returns the target P2P device id to which it needs to        */
/* connect.                                                                  */
INLINE UWORD8 *mget_p2p_trgt_dev_id(void)
{
    return P_MIB_P2P.p2ptrgtid;
}

/* This funtion sets the P2P device id of the device which the user wants to */
/* invite into its group                                                     */
INLINE void mset_p2p_invit_dev_id(UWORD8 *inp)
{
    if(inp)
        TROUT_DBG4("P2P: invit dev: %x:%x:%x:%x:%x:%x",inp[0],inp[1],inp[2],inp[3],inp[4],inp[5]);
    memcpy(P_MIB_P2P.p2pinvtid, inp, 6);
}

/* This funtion returns the P2P device id of the device which the user wants */
/* invite into its group                                                     */
INLINE UWORD8 *mget_p2p_invit_dev_id(void)
{
    return P_MIB_P2P.p2pinvtid;
}

/* This function sets Auto GO mode. In STA mode this can be enabled to       */
/* become a P2P GO automatically (i.e. switch to AP mode). In AP mode this   */
/* can be disabled to return to P2P Client mode (i.e. switch back to Client  */
/* mode).                                                                    */
INLINE void mset_p2p_auto_go(BOOL_T val)
{
	/* Input Sanity Check */
    if((val > 1) )
        return;

    if((BTRUE == mget_p2p_enable()))
        P_MIB_P2P.p2pautoGO = val;
    else
        P_MIB_P2P.p2pautoGO = BFALSE;    
}

/* This function gets the status of autonomous GO functionality.  */
INLINE BOOL_T mget_p2p_auto_go(void)
{
    return (BOOL_T)(P_MIB_P2P.p2pautoGO);
}

/* This function  returns the value of preferred config method */
INLINE UWORD16 mget_p2p_pref_config_method(void)
{
    return P_MIB_P2P.p2p_pref_configmethod;
}

/* This function  sets the value of preferred config method */
INLINE void mset_p2p_pref_config_method(UWORD16 val)
{
    P_MIB_P2P.p2p_pref_configmethod = val;
}

/* This function  returns the listen  mode of the device    */
INLINE UWORD8 mget_p2p_listen_mode(void)
{
    return (BOOL_T)P_MIB_P2P.p2p_listen_mode;
}

/* This function  sets the listen  mode of the device    */
INLINE void mset_p2p_listen_mode(UWORD8 val)
{
    if(val > 1)
        return;
    if(BTRUE == mget_p2p_enable())
    	P_MIB_P2P.p2p_listen_mode = val;
}


/* This function returns if opportunistic power save is being used or not */
INLINE BOOL_T get_opp_ps(void)
{
    return (BOOL_T)(g_p2p_opp_ps);
}
#endif /* MIB_P2P_H */
#endif /* MAC_P2P */
