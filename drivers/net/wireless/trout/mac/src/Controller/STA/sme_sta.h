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
/*  File Name         : sme_sta.h                                            */
/*                                                                           */
/*  Description       : This file contains all declarations and functions    */
/*                      related to SME functions specific to STA mode.       */
/*                                                                           */
/*  List of Functions : initiate_join                                        */
/*                      initiate_auth                                        */
/*                      initiate_asoc                                        */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE

#ifndef SME_STA_H
#define SME_STA_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "fsm_sta.h"
#include "management_sta.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

extern BOOL_T       g_wps_scan_req_from_user; // caisf add for fix wps scan bug. 1121

extern UWORD32 g_auth_retry_cnt;
extern UWORD32 g_assoc_retry_cnt;


#define MAX_SCAN_TIME       1200
#define MIN_SCAN_TIME       10
#define PROBE_DELAY_TIME    10
#define JOIN_TIMEOUT        3000
#define START_TIMEOUT       3000
#define USER_INPUT_TIMEOUT  30000

//zhangzhao add 2013-01-12
#define MAX_NR_SCAN_TIMES   3

/* Macros defining the different types of Scan Supported */
#define DEFAULT_SCAN        0
#define USER_SCAN           BIT0
#define OBSS_PERIODIC_SCAN  BIT1
#define OBSS_ONETIME_SCAN   BIT2
/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

typedef enum {SITE_SURVEY_1CH    = 0,
              SITE_SURVEY_ALL_CH = 1,
              SITE_SURVEY_OFF    = 2
} SITE_SURVEY_T;

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void send_mlme_rsp_to_host_sta(mac_struct_t *mac, UWORD8 type,
                                      UWORD8 *msg);
extern void prepare_mlme_scan_req(scan_req_t *scan_req);
extern void prepare_mlme_join_req(join_req_t *join_req, bss_dscr_t *bss_dscr);
extern void prepare_mlme_start_req(start_req_t *start_req);
extern void prepare_mlme_auth_req(auth_req_t *auth_req);
extern void prepare_mlme_asoc_req(asoc_req_t *asoc_req);

extern void initiate_scan(mac_struct_t *mac);
extern void initiate_start(mac_struct_t *mac);
extern UWORD8 check_scan_match(scan_rsp_t *scan_rsp);

//chenq add for check link ap info change 2013-06-08
extern BOOL_T check_cur_link_ap_info_change(bss_dscr_t * bss_dscr);
//chenq add end

extern void set_start_scan_req_sta(UWORD8 val);
extern void initiate_scan_procedure(mac_struct_t *mac);
extern void prepare_scan_response(scan_rsp_t *scan_rsp);

extern void start_obss_scan(void);
extern void end_obss_scan(mac_struct_t *mac);

extern void start_obss_scan_timer(void);
extern void stop_obss_scan_timer(void);

#ifndef OS_LINUX_CSL_TYPE
extern void obss_scan_to_fn(ALARM_HANDLE_T* alarm, ADDRWORD_T data);
#else /* OS_LINUX_CSL_TYPE */
extern void obss_scan_to_fn(ADDRWORD_T data);
#endif /* OS_LINUX_CSL_TYPE */
extern void handle_start_scan_req_sta(void);

extern unsigned int scan_need_goon(void);
extern void init_scan_limit(void);
extern void inc_scan_limit(void);





/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/
/* This function prepares the join request message and calls the MAC API to  */
/* start joining.                                                            */
INLINE void initiate_join(mac_struct_t *mac, bss_dscr_t *bss_dscr)
{
    join_req_t join_req = {{INFRASTRUCTURE,0,},};

    prepare_mlme_join_req(&join_req, bss_dscr);

    mlme_join_req(mac, (UWORD8*)&join_req);
}

/* This function prepares the auth request message and calls the MAC API to  */
/* start authenticating.                                                     */
INLINE void initiate_auth(mac_struct_t *mac)
{
    auth_req_t auth_req = {{0,},};

    prepare_mlme_auth_req(&auth_req);

    mlme_auth_req(mac, (UWORD8*)&auth_req);
}

/* This function prepares the asoc request message and calls the MAC API to  */
/* start associating.                                                        */
INLINE void initiate_asoc(mac_struct_t *mac)
{
    asoc_req_t asoc_req = {{0,},};

    prepare_mlme_asoc_req(&asoc_req);

    mlme_asoc_req(mac, (UWORD8*)&asoc_req);
}

#endif /* SME_STA_H */

#endif /* IBSS_BSS_STATION_MODE */
