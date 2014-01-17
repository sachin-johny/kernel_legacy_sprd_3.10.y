/*****************************************************************************/
/*                                                                           */
/*                           Ittiam WPS SOFTWARE                             */
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
/*  File Name         : ap_management_wps.h                                  */
/*                                                                           */
/*  Description       : This file contains all the data type definitions for */
/*                      WPS AP                                               */
/*                                                                           */
/*  List of Functions : None                                                 */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/
#ifdef BSS_ACCESS_POINT_MODE
#ifdef INT_WPS_SUPP

#ifndef AP_MGMT_WPS_H
#define AP_MGMT_WPS_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "wps_reg_if.h"
#include "management_ap.h"
#include "frame.h"

/*****************************************************************************/
/* Constant Macros                                                           */
/*****************************************************************************/

#define MAX_ENR_BACKLIST       4
#define MAX_FAIL_ENR_LIST      MAX_ENR_BACKLIST
#define WPS_BLACKLIST_LIMIT    3

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/
/* Structure used to list down the Enrollees which have failed WPS protocol  */
typedef struct
{
	UWORD8 fail_cnt;
	UWORD8 mac_addr[MAC_ADDRESS_LEN];
} wps_fail_enr_t;

/* This structure maintains a list of Enrollees that are blacklisted because */
/* of repetitive Registration Protocol Failure                               */
typedef struct
{
	UWORD8 blacklist_cnt;
	UWORD8 mac_addr[MAX_ENR_BACKLIST][MAC_ADDRESS_LEN];
} wps_enr_blacklist_t;

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/
/* Enumeration used to return after checking ASSOC-REQ */
typedef enum
{
	WPS_CHECK_SUCCESS = 0,
	WPS_CHECK_FAILURE = 1,
	WPS_CHECK_FURTHER = 2
} WPS_ASSOC_CHECK_T;

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/

extern UWORD8 g_fail_enr_cnt;
extern wps_fail_enr_t g_fail_enr_list[MAX_FAIL_ENR_LIST];
extern wps_enr_blacklist_t g_enr_blacklist;

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/
extern UWORD16 set_wps_wsc_info_element_ap(UWORD8 *data, UWORD16 index,
                                            TYPESUBTYPE_T frame_type);
extern void wps_handle_probe_req_ap(UWORD8 *msa, UWORD8 *sa, UWORD16 rx_len,
                                     TYPESUBTYPE_T frame_type);
extern STATUS_T wps_handle_eap_rx_ap(asoc_entry_t *ae, UWORD8 *msa,
                                      UWORD16 rx_len);
extern WPS_ASSOC_CHECK_T check_wps_capabilities_ap(asoc_entry_t *ae,
                                                   UWORD8 *msa,
                                                   UWORD16 rx_len);
extern void wps_update_beacon_wsc_ie_ap(void);
extern void wps_update_failed_enr_list(UWORD8* mac_addr);

/*****************************************************************************/
/* Inline functions                                                          */
/*****************************************************************************/

/* This function checks if the given MAC address is the Enrollee Blacklist   */
INLINE BOOL_T is_enr_blacklisted(UWORD8 *mac_addr)
{
	UWORD8 i = 0;
	BOOL_T retval = BFALSE;

	for(i = 0; i < g_enr_blacklist.blacklist_cnt; i++)
	{
		if(BTRUE == mac_addr_cmp(mac_addr, g_enr_blacklist.mac_addr[i]))
		{
			retval = BTRUE;
			break;
		}
	}

	return retval;
}

/* This function clears the Enrollee Blacklist by setting the count to zero */
INLINE void wps_clear_black_list(void)
{
	g_enr_blacklist.blacklist_cnt = 0;
}

/* This function resets the Failed Enrollee count */
INLINE void wsp_clear_fail_enr_list(void)
{
	g_fail_enr_cnt = 0;
}

/* This function shifts all the entries by one position in Failed Enrollee   */
/* list starting from the specifed lower index upto the higher index         */
INLINE void sort_fail_enr_list(UWORD8 lower_indx, UWORD8 upper_indx)
{
	UWORD8 i = 0;

	for(i = lower_indx; i < upper_indx; i++)
		memcpy((UWORD8*)&g_fail_enr_list[i], (UWORD8*)&g_fail_enr_list[i+1],
		       sizeof(wps_fail_enr_t));
}

#endif /* AP_MGMT_WPS_H */
#endif /* INT_WPS_SUPP */
#endif /* BSS_ACCESS_POINT_MODE */
