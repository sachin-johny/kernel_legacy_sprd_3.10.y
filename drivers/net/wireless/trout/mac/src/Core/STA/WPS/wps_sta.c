/*****************************************************************************/
/*                                                                           */
/*                     Ittiam 802.11 MAC SOFTWARE                            */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2010                               */
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
/*  File Name         : wps_sta.c                                            */
/*                                                                           */
/*  Description       : This file contains all functions related to WPS      */
/*                      feature in BSS STA mode.                             */
/*                                                                           */
/*  List of Functions : allocate_wps_handle_mem                              */
/*                      free_wps_handle_mem                                  */
/*                      sys_start_wps                                        */
/*                      end_wps_enrollee                                     */
/*                      rec_wps_cred                                         */
/*                      wps_timeout_fn                                       */
/*                      start_wps_scan                                       */
/*                      prepare_wps_config                                   */
/*                      update_rcv_wps_cred                                  */
/*                      join_wps_cred                                        */
/*                      wps_cred_join_timeout_fn                             */
/*                      handle_wps_cred_join_timeout                         */
/*                      set_wsc_info_element                                 */
/*                      handle_mlme_rsp_wps_sta                              */
/*                      handle_wps_event_sta                                 */
/*                      process_wps_pkt_sta                                  */
/*                      handle_wps_scan_rsp_sta                              */
/*                      handle_wps_scan_complete_sta                         */
/*                      start_wps_cred_rejoin_timer                          */
/*                      send_wps_status                                      */
/*                      send_wps_cred_list                                   */
/*                      set_wps_cred_list_enr                                */
/*                      get_wps_cred_list_enr                                */
/*                      set_wps_dev_mode_sta                                 */
/*                      join_wps_cred_from_eeprom                            */
/*                      read_wps_cred_from_eeprom                            */
/*                      write_wps_cred_to_eeprom                             */
/*                      wps_mem_alloc                                        */
/*                      wps_mem_free                                         */
/*                      wps_frame_mem_alloc                                  */
/*                      wps_frame_mem_free                                   */
/*                      save_cred_to_pers_mem                                */
/*                      read_cred_from_pers_mem                              */
/*                      init_wps_sta_globals                                 */
/*                      wps_allow_sys_restart_sta                            */
/*                      send_1x_fail_deauth                                  */
/*                      handle_host_non_wps_cred_req                         */
/*                      wps_create_alarm                                     */
/*                      wps_delete_alarm                                     */
/*                      wps_start_alarm                                      */
/*                      wps_stop_alarm                                       */
/*                                                                           */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef INT_WPS_SUPP
#ifdef IBSS_BSS_STATION_MODE

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "imem_if.h"
#include "wps_sta.h"
#include "wps_enr_if.h"
#include "host_if.h"
#include "iconfig.h"
#include "controller_mode_if.h"
#include "mac_init.h"
#include "phy_prot_if.h"
#include "qmu_if.h"

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

BOOL_T  g_config_write_from_host             = BFALSE;
UWORD8 g_wps_dev_mode  = WPS_STANDALONE_DEVICE;
UWORD8 g_wps_cred_list[MAX_WPS_CRED_LIST_SIZE] = {0};
wps_internal_params_t g_wps_params = {0};
wps_enrollee_t        *g_wps_enrollee_ptr = 0;
WPS_CRED_USAGE_STATUS_T  g_wps_use_creds = WPS_CRED_NOT_USED;
ALARM_HANDLE_T *g_wps_cred_join_timer = 0;
UWORD16 g_wps_scratch_mem_idx[MAX_NESTED_SCRATCH_MEM_ALLOC] = {0};
UWORD8 g_wps_cur_scratch_indx = 0;
wps_enr_config_struct_t *g_wps_config_ptr = 0;


/*****************************************************************************/
/* Static Global Variables                                                   */
/*****************************************************************************/

#ifdef WPS_DATA_MEM_GLOBAL
//static wps_enrollee_t g_wps_enrollee_buf;
#endif /* WPS_DATA_MEM_GLOBAL */

static UWORD8 g_wps_curr_cred        = 0;
static UWORD8 g_wps_proc_cnt         = 0;
static UWORD8 g_cred_saved_in_eeprom = 0;


/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/

#ifndef MAC_P2P
static void prepare_wps_config(wps_enr_config_struct_t *config_ptr,
                               WPS_PROT_TYPE_T prot_type);
#endif /* MAC_P2P */
static UWORD8 update_rcv_wps_cred(wps_cred_t *cred, UWORD8 *cred_buff,
                                  BOOL_T ver2_cap_ap);
static void join_wps_cred(void);

#ifndef OS_LINUX_CSL_TYPE
static void wps_cred_join_timeout_fn(HANDLE_T* alarm, ADDRWORD_T data);
#else /* OS_LINUX_CSL_TYPE */
static void wps_cred_join_timeout_fn(ADDRWORD_T data);
#endif /* OS_LINUX_CSL_TYPE */

static void handle_wps_cred_join_timeout(misc_event_msg_t *misc);

static void read_wps_cred_from_eeprom(UWORD8 *cred_list);
static void write_wps_cred_to_eeprom(UWORD8 *cred, UWORD8 cred_len);
static void read_cred_from_pers_mem(wps_store_cred_t *store_cred);
static void send_wps_cred_list(void);
static void send_1x_fail_deauth(void);

/*****************************************************************************/
/*                                                                           */
/*  Function Name : allocate_wps_handle_mem                                  */
/*                                                                           */
/*  Description   : This function allocates memory for the WPS structures.   */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_wps_enrollee_ptr                                       */
/*                  g_wps_config_ptr                                         */
/*                                                                           */
/*  Processing    : This function allocates buffers for the WPS enrollee and */
/*                  configuration structures. Note that these buffers are    */
/*                  locked after allocation to prevent freeing during resets */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : STATUS_T; WPS Structure allocation status                */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

STATUS_T allocate_wps_handle_mem(void)
{
#if 0  //wxb modify
#ifndef WPS_DATA_MEM_GLOBAL
    if(NULL != g_wps_enrollee_ptr)
    {
        unlock_the_buffer(g_wps_enrollee_ptr);
        mem_free(g_local_mem_handle, g_wps_enrollee_ptr);
        g_wps_enrollee_ptr = NULL;
    }
#endif /* WPS_DATA_MEM_GLOBAL */

    if(NULL != g_wps_config_ptr)
    {
        unlock_the_buffer(g_wps_config_ptr);
        mem_free(g_local_mem_handle, g_wps_config_ptr);
        g_wps_config_ptr = NULL;
    }

#ifdef WPS_DATA_MEM_GLOBAL
    g_wps_enrollee_ptr = (wps_enrollee_t *)(&g_wps_enrollee_buf);
#else /* WPS_DATA_MEM_GLOBAL */
    g_wps_enrollee_ptr = (wps_enrollee_t *)mem_alloc(g_local_mem_handle,
                                                       sizeof(wps_enrollee_t));

    if(NULL == g_wps_enrollee_ptr)
    {
        raise_system_error(NO_LOCAL_MEM);
        return FAILURE;
    }
#endif /* WPS_DATA_MEM_GLOBAL */

    g_wps_config_ptr = (wps_enr_config_struct_t *)mem_alloc(g_local_mem_handle,
                                              sizeof(wps_enr_config_struct_t));
    if(NULL == g_wps_config_ptr)
    {
        mem_free(g_local_mem_handle, g_wps_enrollee_ptr);
        raise_system_error(NO_LOCAL_MEM);
        return FAILURE;
    }

    /* Lock the buffers */
#ifndef WPS_DATA_MEM_GLOBAL
    lock_the_buffer(g_wps_enrollee_ptr);
#endif /* WPS_DATA_MEM_GLOBAL */
    lock_the_buffer(g_wps_config_ptr);
#else /* if 0 */
#if 1
	printk("%s -kmalloc g_wps_enrollee_ptr and g_wps_config_ptr\n", __FUNCTION__);
	if(NULL != g_wps_enrollee_ptr)
	{
	        kfree(g_wps_enrollee_ptr);
	        g_wps_enrollee_ptr = NULL;
	}

	g_wps_enrollee_ptr = (wps_enrollee_t *)kmalloc(sizeof(wps_enrollee_t), GFP_KERNEL);

	if(NULL == g_wps_enrollee_ptr)
	{
	        raise_system_error(NO_LOCAL_MEM);
	        return FAILURE;
	}

	if(NULL != g_wps_config_ptr)
	{
	        kfree(g_wps_config_ptr);
	        g_wps_config_ptr = NULL;
	}

	g_wps_config_ptr = (wps_enr_config_struct_t *)kmalloc(sizeof(wps_enr_config_struct_t), GFP_KERNEL);//wxb add

	if(NULL == g_wps_config_ptr)
	{
	        raise_system_error(NO_LOCAL_MEM);
	        return FAILURE;
	}

	printk("%s -kmalloc complete\n", __FUNCTION__);
#else

	g_wps_enrollee_ptr = (wps_enrollee_t *)(&g_wps_enrollee_buf);
	g_wps_config_ptr = (wps_enr_config_struct_t *)(&g_wps_config_buf);
	memset(g_wps_enrollee_ptr, 0, sizeof(wps_enrollee_t));
	memset(g_wps_config_ptr, 0, sizeof(wps_enr_config_struct_t));

	printk("%s -- g_wps_enrollee_ptr: %p\n", __FUNCTION__, g_wps_enrollee_ptr);
	printk("%s -- g_wps_config_ptr: %p\n", __FUNCTION__, g_wps_config_ptr);
#endif /* if 1 */

#endif /* if 0 */

    return SUCCESS;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : free_wps_handle_mem                                      */
/*                                                                           */
/*  Description   : This function frees the memory for the WPS structures.   */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_wps_enrollee_ptr                                       */
/*                  g_wps_config_ptr                                         */
/*                                                                           */
/*  Processing    : This function unlocks and frees the buffers that were    */
/*                  allocated for the WPS enrollee and configuration         */
/*                  structures.                                              */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void free_wps_handle_mem(void)
{
#if 0  //wxb modify
#ifndef WPS_DATA_MEM_GLOBAL
    if(NULL != g_wps_enrollee_ptr)
    {
        unlock_the_buffer(g_wps_enrollee_ptr);
        mem_free(g_local_mem_handle, g_wps_enrollee_ptr);
    }

#endif /* WPS_DATA_MEM_GLOBAL */

    if(NULL != g_wps_config_ptr)
    {
        unlock_the_buffer(g_wps_config_ptr);
        mem_free(g_local_mem_handle, g_wps_config_ptr);
    }
#else /* if 0 */

	printk("%s -free g_wps_enrollee_ptr and g_wps_config_ptr\n", __FUNCTION__);
	if(NULL != g_wps_enrollee_ptr)
	{
		kfree(g_wps_enrollee_ptr);
	}

	if(NULL != g_wps_config_ptr)
	{
	       kfree(g_wps_config_ptr);
	}

#endif /* if 0 */

    g_wps_enrollee_ptr = NULL;
    g_wps_config_ptr   = NULL;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : sys_start_wps                                            */
/*                                                                           */
/*  Description   : This function starts the WPS Enrollee.                   */
/*                                                                           */
/*  Inputs        : 1) WPS Protocol type (PIN/PBC)                           */
/*                                                                           */
/*  Globals       : g_wps_config_ptr                                         */
/*                                                                           */
/*  Processing    : This function prepares the configuration input for WPS   */
/*                  and calls the WPS API to start the enrollee. If the      */
/*                  status returned is FAILURE, the WPS mode is set to       */
/*                  DISABLED and FAILURE returned.      The WPS mode needs   */
/*                  to be set to WPS_ENABLED initially so that MAC SW        */
/*                  is aware that WPS enrollee is running when the WPS       */
/*                  enrollee function (wps_start_enrollee) is called.        */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : STATUS_T; WPS API return status                          */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

STATUS_T sys_start_wps(WPS_PROT_TYPE_T prot_type)
{
    STATUS_T ret_status = FAILURE;
    /* Reset the length and number of credentials in the global credential  */
    /* list buffer                                                          */
    g_wps_cred_list[WPS_CRED_LIST_LEN_OFF] = 0;
    g_wps_cred_list[WPS_CRED_LIST_LEN_OFF+1] = 0;
    g_wps_cred_list[WPS_CRED_LIST_CNT_OFF] = 0;
    g_wps_use_creds = WPS_CRED_NOT_USED;

    set_wps_mode(WPS_ENABLED);
    if(NULL != g_wps_cred_join_timer)
    {
        /* Delete the WPS credential join timer                              */
        delete_alarm(&g_wps_cred_join_timer);
        g_wps_cred_join_timer = NULL;
    }
    free_wps_handle_mem(); //wxb add
    if(allocate_wps_handle_mem() == SUCCESS)
    {
        /* Prepare the WPS configuration structure */
        prepare_wps_config(g_wps_config_ptr, prot_type);
        ret_status = wps_start_enrollee(g_wps_enrollee_ptr, g_wps_config_ptr);
        if(FAILURE == ret_status)
        {
            free_wps_handle_mem();
        }
    }
    if(FAILURE == ret_status)
    {
        set_wps_mode(WPS_DISABLED);
        g_wps_params.wps_prot = WPS_PROT_NONE;
        disconnect_station(0);
        send_wps_status(ERR_WPS_INIT_FAIL, NULL, 0);
    }
    return ret_status;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : end_wps_enrollee                                         */
/*                                                                           */
/*  Description   : This function handles the completion of WPS Enrollee     */
/*                  function.                                                */
/*                                                                           */
/*  Inputs        : 1) WPS status code                                       */
/*                                                                           */
/*  Globals       : g_wps_cred                                               */
/*                  g_wps_curr_cred                                          */
/*                  g_wps_proc_cnt                                           */
/*                  g_wps_enrollee_ptr                                       */
/*                                                                           */
/*  Processing    : This function takes appropriate action based on the WPS  */
/*                  Enrollee function status code. In case of system error   */
/*                  the system is restarted. If the protocol type is PIN the */
/*                  WPS enrollee function is invoked to try to start again.  */
/*                  In any other failure case the station is disconnected.   */
/*                  In case the WPS enrollee function completed successfully */
/*                  the buffers are freed and a function is called so to     */
/*                  initiate join to the current WPS credential.             */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void end_wps_enrollee(WPS_STATUS_T status_code)
{
    set_wps_mode(WPS_DISABLED);
    /* Clear all the existing networks                                       */
    g_bss_dscr_set_index = 0;

    /* Clear the scratch memory index, incase not done already */
    if(g_wps_cur_scratch_indx != 0)
    {
        UWORD8 i = 0;

        restore_scratch_mem_idx(g_wps_scratch_mem_idx[0]);

        for(i = 0; i < MAX_NESTED_SCRATCH_MEM_ALLOC; i++)
        {
            g_wps_scratch_mem_idx[i] = 0;
        }
        g_wps_cur_scratch_indx = 0;
    }

    if(status_code != REG_PROT_SUCC_COMP)
    {
        send_1x_fail_deauth();

        /* Reset the length and number of credentials in the global          */
        /* credential list buffer                                            */
        g_wps_cred_list[WPS_CRED_LIST_LEN_OFF] = 0;
        g_wps_cred_list[WPS_CRED_LIST_LEN_OFF+1] = 0;
        g_wps_cred_list[WPS_CRED_LIST_CNT_OFF] = 0;

        /* In case of system error restart and initiate WPS start in case of */
        /* WPS protocol type PIN. Disconnect the station otherwise.          */
        if(status_code == ERR_SYSTEM)
        {
            restart_mac(&g_mac,0);

            if(g_wps_config_ptr->prot_type == PIN)
            {
                send_wps_status(status_code, NULL, 0);
                sys_start_wps(g_wps_config_ptr->prot_type);
                g_wps_proc_cnt++;
                return;
            }
        }

        disconnect_station(0);
        send_wps_status(status_code, NULL, 0);
    }
    else
    {
        disconnect_station(0);
        send_wps_status(status_code, NULL, 0);
        if(get_wps_dev_mode_enr() == WPS_HOST_MANAGED_DEVICE)
        {
            send_wps_cred_list();
        }
        else
        {
            /* If WPS Enrollee function completed successfully free the      */
            /* enrollee pointer, set the current WPS credential and initiate */
            /* join to this in Standalone mode                               */
            g_wps_curr_cred = 0;
            g_cred_saved_in_eeprom = 0;
            join_wps_cred();
        }
    }
    g_wps_params.wps_prot = WPS_PROT_NONE;
    free_wps_handle_mem();
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : rec_wps_cred                                             */
/*                                                                           */
/*  Description   : This function receives the WPS Enrollee credentials.     */
/*                                                                           */
/*  Inputs        : 1) Pointer to the credentials                            */
/*                                                                           */
/*  Globals       : g_wps_cred                                               */
/*                                                                           */
/*  Processing    : This function updates the received WPS credential to a   */
/*                  temporary buffer in the format that is required to be    */
/*                  maintained internally (WPS Credential List format).      */
/*                  If this function fails, do nothing and return. Otherwise */
/*                  for Standalone mode, allocate a global WPS credential    */
/*                  list buffer (if not available already) and reset the     */
/*                  number of credentials to 0. Now read the number of       */
/*                  credentials as saved in the WPS credential list and      */
/*                  append the received credential to the end if the max     */
/*                  limit has not been reached. Update the number of         */
/*                  credentials in the list buffer. For Host Managed mode    */
/*                  send the received credentials to the host.               */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : BTRUE - If credential record successful else BFALSE      */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

BOOL_T rec_wps_cred(wps_cred_t *cred, BOOL_T ver2_cap_ap)
{
    /* WPS TBD - Decide if you want to allocate this from local memory       */
    UWORD8  temp_cred_buff[MAX_CRED_SIZE] = {0};
    UWORD8  cred_len = 0;
    UWORD16 temp_u16 = 0;

    /* Update the received WPS credential parameters in the required format  */
    /* in the temporary credentials buffer                                   */
    cred_len = update_rcv_wps_cred(cred, temp_cred_buff, ver2_cap_ap );

    /* Length 0 indicates that the received WPS credential could not be      */
    /* parsed successfully and failed some validation check. This credential */
    /* is ignored.                                                           */
    if(cred_len == 0)
    {
        return BFALSE;
    }

    /* Save the received credential in the following format                  */
    /* --------------------------------------------------------------------- */
    /* | Length |Num Creds | Cred 1 Length | Value |.| Cred n Length | Value|*/
    /* --------------------------------------------------------------------- */
    /* |  2b    |1b (n)    | 1b (c1)       | c1 b  |.| 1b (cn)       | cn b |*/
    /* --------------------------------------------------------------------- */
    /* If there is enough space in the credential list buffer to store the   */
    /* new credential then store it else discard it.                         */
    /* Storage required is cred_len+1 thats why check is <=. Also cCheck is  */
    /* not vary accurate when length stored length is zero, however in that  */
    /* case there will anyway be enough space to store one credential        */
    temp_u16 = ((UWORD16)g_wps_cred_list[WPS_CRED_LIST_LEN_OFF+1]) << 8;
    temp_u16 |= g_wps_cred_list[WPS_CRED_LIST_LEN_OFF];
    if(0 == temp_u16)
    {
        temp_u16 = 1;
    }
    temp_u16 += WPS_CRED_LIST_CNT_OFF;
    if((MAX_WPS_CRED_LIST_SIZE - temp_u16) <= cred_len)
    {
        return BTRUE;
    }
    /* Copy the received credential to the offset in the required format     */
    g_wps_cred_list[temp_u16++] = cred_len;
    memcpy(&g_wps_cred_list[temp_u16], temp_cred_buff, cred_len);
    temp_u16 = temp_u16 + cred_len - WPS_CRED_LIST_CNT_OFF;

    /* Increment the number of credentials being stored and save the         */
    /* same in the WPS credential list structure                             */
    g_wps_cred_list[WPS_CRED_LIST_CNT_OFF]++;
    /* Update the length of the credential list                              */
    g_wps_cred_list[WPS_CRED_LIST_LEN_OFF] = (UWORD8)(temp_u16 & 0xFF);
    g_wps_cred_list[WPS_CRED_LIST_LEN_OFF+1] = (UWORD8)(temp_u16 >> 8);

    return BTRUE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : wps_timeout_fn                                           */
/*                                                                           */
/*  Description   : This function handles the WPS Enrollee timeouts.         */
/*                                                                           */
/*  Inputs        : 1) Timer data                                            */
/*                                                                           */
/*  Globals       : g_event_mem_handle                                       */
/*                                                                           */
/*  Processing    : This function handles WPS timeout and fires a High       */
/*                  priority event to handle WPS timeout. It creates a high  */
/*                  priority event with event data set to NULL, event        */
/*                  info set to the timer input data and event name set to   */
/*                  WPS_TIMEOUT_EVENT. Call post_event with the pointer to   */
/*                  the event created and event ID as HIGH_PRI_EVENT_QID.    */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

#ifndef OS_LINUX_CSL_TYPE
void wps_timeout_fn(HANDLE_T* alarm, ADDRWORD_T data)
#else /* OS_LINUX_CSL_TYPE */
void wps_timeout_fn(ADDRWORD_T data)
#endif /* OS_LINUX_CSL_TYPE */
{
    misc_event_msg_t *misc = 0;
    TROUT_DBG4("WPS: timeout!");

    misc = (misc_event_msg_t*)event_mem_alloc(MISC_EVENT_QID);

    if(NULL == misc)
    {
        raise_system_error(NO_EVENT_MEM);
        return;
    }

    misc->data = 0;
    misc->info = data;
    misc->name = WPS_TIMEOUT_EVENT;
    post_event((UWORD8*)misc, MISC_EVENT_QID);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : start_wps_scan                                           */
/*                                                                           */
/*  Description   : This function starts the WLAN scan in WPS mode.          */
/*                                                                           */
/*  Inputs        : 1) Indicate whether deauth with 1x failure result code is*/
/*                     required                                              */
/*                                                                           */
/*  Globals       : g_reset_mac                                              */
/*                                                                           */
/*  Processing    : The BSS Type, Scan type are set to desired values. The   */
/*                  start scan request function is called and MAC is         */
/*                  restarted. The global WPS enrollee pointer is returned   */
/*                  to the WPS enrollee module.                              */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None                                                     */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void start_wps_scan(BOOL_T deauth_1x_req)
{
    if(BTRUE == deauth_1x_req)
    {
        send_1x_fail_deauth();
    }

    set_site_survey(SITE_SURVEY_OFF);
    /* Disconnect the station if connected                                   */
    disconnect_station(0);
    mset_DesiredBSSType(INFRASTRUCTURE);

    mset_scan_type(ACTIVE_SCAN);

    g_reset_req_from_user = DO_RESET;
    g_wps_scan_req_from_user = BTRUE;
    set_start_scan_req(USER_SCAN);

    return;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : prepare_wps_config                                       */
/*                                                                           */
/*  Description   : This function prepares the configuration for the WPS     */
/*                  Enrollee.                                                */
/*                                                                           */
/*  Inputs        : 1) Pointer to the configuration structure                */
/*                  2) WPS Protocol type (PIN/PBC)                           */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function gets the configuration inputs required by  */
/*                  WPS Enollee and updates the given configuration          */
/*                  structure with the same.                                 */
/*                                                                           */
/*  Outputs       : Updates the structure pointed by the given config_ptr    */
/*                                                                           */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void prepare_wps_config(wps_enr_config_struct_t *config_ptr,
                        WPS_PROT_TYPE_T prot_type)
{
    UWORD8 *str_wid_val = 0;

    memset(config_ptr, 0, sizeof(wps_enr_config_struct_t)); //wxb add

    config_ptr->prot_type = prot_type;
    config_ptr->dev_pass_id = get_wps_pass_id();

    config_ptr->wps_state = WPS_NOT_CONFIGURED;
    config_ptr->rf_bands = get_rf_band();

    str_wid_val = get_wps_pin();
    config_ptr->dev_pass_len = str_wid_val[0];
    config_ptr->dev_pass     = &str_wid_val[1];

    config_ptr->mac_address = mget_StationID();

    config_ptr->config_meth = get_wps_config_method();

    /* 0x1- ESS, 0x2 - IBSS */
    if(mget_DesiredBSSType() == INDEPENDENT)
    {
        config_ptr->con_type_flag = 0x02;
    }
    else
    {
        config_ptr->con_type_flag = 0x01;
    }

    /* Category ID, OUI and sub category ID in sequence */
    str_wid_val = get_prim_dev_type();
    config_ptr->prim_dev_cat_id = &str_wid_val[2];

#ifdef MAC_P2P
    str_wid_val = get_req_dev_type();
    if(REQ_DEV_TYPE_LEN == ((str_wid_val[1] << 8) + str_wid_val[0]))
    {   /* The requested device type is set only if it is valid */
        g_wps_config_ptr->req_dev_cat_id = &str_wid_val[2];
    }
#endif /* MAC_P2P */

#ifdef WPS_1_0_SEC_SUPP
    config_ptr->enc_type_flags  =
        (WPS_NONE | WPS_WEP | WPS_AES | WPS_TKIP | WPS_AES_TKIP);
    config_ptr->auth_type_flags =
        (WPS_OPEN | WPS_WPA_PSK | WPS_SHARED | WPS_WPA2_PSK);
#else  /* WPS_1_0_SEC_SUPP */
    config_ptr->enc_type_flags  = (WPS_NONE | WPS_TKIP | WPS_AES );
    config_ptr->auth_type_flags = (WPS_OPEN | WPS_WPA_PSK | WPS_WPA2_PSK);
#endif /* WPS_1_0_SEC_SUPP */

    str_wid_val = get_manufacturer();
    config_ptr->manufacturer_len = str_wid_val[0];
    config_ptr->manufacturer     = &str_wid_val[1];

    str_wid_val = get_model_name();
    config_ptr->model_name_len = str_wid_val[0];
    config_ptr->model_name     = &str_wid_val[1];

    str_wid_val = get_model_num();
    config_ptr->model_num_len = str_wid_val[0];
    config_ptr->model_num     = &str_wid_val[1];

    str_wid_val = get_dev_name();
    config_ptr->dev_name_len = str_wid_val[0];
    config_ptr->dev_name     = &str_wid_val[1];

    config_ptr->os_version = get_dev_os_version();

    str_wid_val = get_serial_number();
    config_ptr->serial_num_len = str_wid_val[0];
    config_ptr->serial_num     = &str_wid_val[1];

    /* Set the Version2 subelement to indicate WSC 2.0 capable */
    config_ptr->version2.id = WPS_SUB_ELEM_VER2;
    config_ptr->version2.len = WPS_VER2_LEN;
    config_ptr->version2.val[0] = WPS_VER2_VAL;

    /* Pointer to timer callback function */
    config_ptr->timer_cb_fn_ptr = wps_timeout_fn;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_rcv_wps_cred                                      */
/*                                                                           */
/*  Description   : This function updates the received WPS credenials from   */
/*                  the WPS enrollee credential structure to the given       */
/*                  buffer in the required format.                           */
/*                                                                           */
/*  Inputs        : 1) Pointer to the WPS credentials structure              */
/*                  2) Buffer in which the credential needs to be saved      */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function prepares the credential as a list of WIDs  */
/*                  and saves the same in the given buffer. During the       */
/*                  update it also performs validity checks on the various   */
/*                  incoming parameters and stops the update in case any     */
/*                  issue is found.                                          */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : UWORD8, Length of the credential updated in the buffer   */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
UWORD8 update_rcv_wps_cred(wps_cred_t *cred, UWORD8 *temp_cred_buff,
                           BOOL_T ver2_cap_ap)
{
    UWORD8 index = 0;
    UWORD8 auth_type  = 0;
    UWORD8 sec_mode   = 0;
    UWORD8 klen  = cred->net_key_len[0];

    if((cred->ssid_len) > (MAX_SSID_LEN - 1))
    {
        /* Exception - Length of SSID exceeds maximum SSID length */
        return 0;
    }
    if(NULL == g_wps_config_ptr)
    {
        return 0;
    }
    /*************************************************************************/
    /* Check for common capabilities between received and configured values  */
    /* of "Authentication Type Flags" and "Encryption Type Flags". If there  */
    /* is no common capability in any one  of these then discard credential  */
    /*************************************************************************/
    if(((cred->enc_type & g_wps_config_ptr->enc_type_flags &
        WPS_ENCR_TYPE_FLAGS_MASK) == 0) ||
        ((cred->auth_type & g_wps_config_ptr->auth_type_flags &
        WPS_AUTH_TYPE_FLAGS_MASK) == 0))
    {
        return 0;
    }

    if((cred->enc_type != WPS_NONE) && (klen == 0))
    {
        /* Exception - For non-NONE encryption, Key length must be non-zero */
        return 0;
    }

    /* Set the security mode. The security mode is in the following format   */
    /* BIT0   - Encryption On       (ON = 1, OFF = 0)                        */
    /* BIT1:2 - WEP Encryption      (WEP-40 = 01, WEP-104 = 11               */
    /* BIT3:4 - Authentication Type (WPA = 01, WPA2 = 10, Mixed = 11         */
    /* BIT5:6 - Encryption Type     (AES = 01, TKIP = 10, AES+TKIP = 11      */
    switch(cred->enc_type)
    {
    case WPS_NONE:
    {
        sec_mode = SEC_MODE_NONE;
    }
    break;
    case WPS_WEP:
#ifdef WPS_1_0_SEC_SUPP
    {
        UWORD8 i          = 0;
        for(i = 1; i < MAX_NW_KEY_PER_CRED; i++)
        {
            UWORD8 temp = cred->net_key_len[i];

            if((temp != 0) && (temp != klen))
            {
                /* Exception - Length of all keys, if present must be same */
                return 0;
            }
        }

        if((klen == (WEP40_KEY_SIZE * 2)) || (klen == WEP40_KEY_SIZE))
        {
            sec_mode = SEC_MODE_WEP40;
        }
        else if((klen == (WEP104_KEY_SIZE * 2)) || (klen == WEP104_KEY_SIZE))
        {
            sec_mode = SEC_MODE_WEP104;
        }
        else
        {
            /* Exception - Key length violation for WEP-40 or WEP-104 */
            return 0;
        }
    }
    break;
#else  /* WPS_1_0_SEC_SUPP */
        /* WEP Encryption is not supported for WSC 2.0 */
        return 0;
#endif /* WPS_1_0_SEC_SUPP */
    case WPS_AES:
    case WPS_AES_TKIP:
    {
        sec_mode = ((SEC_MODE_CT_AES) << SEC_MODE_CT_OFFSET) + BIT0;
    }
    break;
    case WPS_TKIP:
    {
#ifdef WPS_1_0_SEC_SUPP
        sec_mode = ((SEC_MODE_CT_TKIP) << SEC_MODE_CT_OFFSET) + BIT0;
#else  /* WPS_1_0_SEC_SUPP */
        /* Version 2 capable AP cannot give TKIP credential */
        if(ver2_cap_ap == BTRUE)
            return 0;

        /* In case of WSC 1.0 AP, set the Encryption to Mixed mode */
        sec_mode = ((SEC_MODE_CT_AES_TKIP) << SEC_MODE_CT_OFFSET) + BIT0;
#endif /* WPS_1_0_SEC_SUPP */
    }
    break;
#ifdef WPS_1_0_SEC_SUPP
    case WPS_AES_TKIP_MIXED:
    {
        sec_mode = ((SEC_MODE_CT_AES_TKIP) << SEC_MODE_CT_OFFSET) + BIT0;
    }
    break;
#endif /* WPS_1_0_SEC_SUPP */
    default:
    {
        /* Exception - Unknown Encryption Type */
        return 0;
    }
    }

    if(cred->auth_type == WPS_OPEN)
    {
        if((cred->enc_type != WPS_NONE) && (cred->enc_type != WPS_WEP))
        {
            /* Exception - Open Auth Type valid only for NONE or WEP */
            return 0;
        }

        auth_type = CONFIG_OPEN_AUTH;
    }
    else if(cred->auth_type == WPS_SHARED)
    {
#ifdef WPS_1_0_SEC_SUPP
        if(cred->enc_type != WPS_WEP)
        {
            /* Exception - Shared Auth Type valid only for WEP */
            return 0;
        }

        auth_type = CONFIG_SHARED_AUTH;
#else  /* WPS_1_0_SEC_SUPP */
            return 0;
#endif /* WPS_1_0_SEC_SUPP */
    }
    else
    {
        if((cred->enc_type == WPS_NONE) || (cred->enc_type == WPS_WEP))
        {
            /* Exception - NONE or WEP not valid for any Auth Type other     */
            /* than Open or Shared                                           */
            return 0;
        }

        switch(cred->auth_type)
        {
        case WPS_WPA_PSK:
        {
#ifdef WPS_1_0_SEC_SUPP
            auth_type = CONFIG_OTHER_AUTH;
            sec_mode |= (SEC_MODE_AUTH_WPA) << SEC_MODE_AUTH_OFFSET;
#else  /* WPS_1_0_SEC_SUPP */
        /* Version 2 capable AP cannot give WPA-Only credential */
        if(ver2_cap_ap == BTRUE)
            return 0;

        sec_mode |= (SEC_MODE_AUTH_MIXED) << SEC_MODE_AUTH_OFFSET;
#endif /* WPS_1_0_SEC_SUPP */
        }
        break;
        case WPS_WPA2_PSK:
        case WPS_WPA_WPA2_PSK:
        {
            auth_type = CONFIG_OTHER_AUTH;
            sec_mode |= (SEC_MODE_AUTH_WPA2) << SEC_MODE_AUTH_OFFSET;
        }
        break;
#ifdef WPS_1_0_SEC_SUPP
        case WPS_WPA_WPA2_MIXED:
        {
            auth_type = CONFIG_OTHER_AUTH;
            sec_mode |= (SEC_MODE_AUTH_MIXED) << SEC_MODE_AUTH_OFFSET;
        }
        break;
#endif /* WPS_1_0_SEC_SUPP */
        case WPS_WPA:
        {
#ifdef WPS_1_0_SEC_SUPP
            auth_type = (CONFIG_OTHER_AUTH) | SEC_MODE_1X_AUTH_BIT;
            sec_mode |= (SEC_MODE_AUTH_WPA) << SEC_MODE_AUTH_OFFSET;
#else  /* WPS_1_0_SEC_SUPP */
        /* Version 2 capable AP cannot give WPA-Only credential */
        if(ver2_cap_ap == BTRUE)
            return 0;

        auth_type = (CONFIG_OTHER_AUTH) | SEC_MODE_1X_AUTH_BIT;
        sec_mode |= (SEC_MODE_AUTH_MIXED) << SEC_MODE_AUTH_OFFSET;
#endif /* WPS_1_0_SEC_SUPP */
        }
        break;
        case WPS_WPA2:
        {
            auth_type = (CONFIG_OTHER_AUTH) | SEC_MODE_1X_AUTH_BIT;
            sec_mode |= (SEC_MODE_AUTH_WPA2) << SEC_MODE_AUTH_OFFSET;
        }
        break;
        default:
        {
            /* Exception - Unknown Auth Type */
            return 0;
        }
        }
    }

    /* The credential is saved in the following format                       */
    /* --------------------------------------------------------------------- */
    /* Name                     Length (Bytes)  Value                        */
    /* --------------------------------------------------------------------- */
    /* SSID Parameter WID         2         WID_SSID                         */
    /* wid len                    1                                          */
    /* wid val                    0 to 32                                    */
    /* 11i Mode Parameter WID     2         WID_11I_MODE                     */
    /* wid len                    1                                          */
    /* wid val                    1                                          */
    /* Auth Type Parameter WID    2         WID_AUTH_TYPE                    */
    /* wid len                    1                                          */
    /* wid val                    1                                          */
    /* WEP Key ID Parameter WID   2         WID_KEY_ID (WEP Only)            */
    /* wid len                    1                                          */
    /* wid val                    1                                          */
    /* WEP Key Parameter WID      2         WID_WEP_KEY_VALUE (WEP Only)     */
    /* wid_wep_key_val (Len)      1         X                                */
    /* wid_wep_key_val (Val)      X                                          */
    /* PSK Parameter WID          2         WID_11I_PSK                      */
    /* wid_802_11i_psk_val (Len)  1         X                                */
    /* wid_802_11i_psk_val (Val)  X                                          */
    /* --------------------------------------------------------------------- */

    /* SSID WID-Length-Value */
    temp_cred_buff[index++] = WID_SSID & 0xFF;
    temp_cred_buff[index++] = (WID_SSID & 0xFF00) >> 8;
    temp_cred_buff[index++] = cred->ssid_len;
    memcpy(temp_cred_buff + index, cred->ssid, cred->ssid_len);
    index += cred->ssid_len;

    /* 802.11i Mode WID-Length-Value */
    temp_cred_buff[index++] = WID_11I_MODE & 0xFF;
    temp_cred_buff[index++] = (WID_11I_MODE & 0xFF00) >> 8;
    temp_cred_buff[index++] = 1;
    temp_cred_buff[index++] = sec_mode;

    /* Auth Type WID-Length-Value */
    temp_cred_buff[index++] = WID_AUTH_TYPE & 0xFF;
    temp_cred_buff[index++] = (WID_AUTH_TYPE & 0xFF00) >> 8;
    temp_cred_buff[index++] = 1;
    temp_cred_buff[index++] = auth_type;

    if(sec_mode == SEC_MODE_NONE) /* No encryption */
    {
        /* Do nothing */
    }
#ifdef WPS_1_0_SEC_SUPP
    else if((sec_mode == SEC_MODE_WEP40) || (sec_mode == SEC_MODE_WEP104))
    {
        UWORD8 i          = 0;
        for(i = 0; i < MAX_NW_KEY_PER_CRED; i ++)
        {
            UWORD8 key_id  = cred->net_key_index[i];
            UWORD8 key_len = cred->net_key_len[i];
            UWORD8 *key    = cred->net_key[i];

            /* If key length is 0 it indicates that no valid key is present */
            if(key_len == 0)
                break;

            /* Key ID should be in the range 1 to NUM_DOT11WEPDEFAULTKEYVALUE*/
            /* Also as index used of 0 to NUM_DOT11WEPDEFAULTKEYVALUE,       */
            /* subtract 1 from key id                                        */
            if((0 == key_id) || (NUM_DOT11WEPDEFAULTKEYVALUE < key_id))
            {
                return 0;
            }
            key_id--;

            temp_cred_buff[index++] = WID_KEY_ID & 0xFF;
            temp_cred_buff[index++] = (WID_KEY_ID & 0xFF00) >> 8;
            temp_cred_buff[index++] = 1;
            temp_cred_buff[index++] = key_id;

            temp_cred_buff[index++] = WID_WEP_KEY_VALUE & 0xFF;
            temp_cred_buff[index++] = (WID_WEP_KEY_VALUE & 0xFF00) >> 8;

            /* If WEP Key Len is 5 or 13 then parse the keys as ASCII keys */
            if((key_len == WEP40_KEY_SIZE) || (key_len == WEP104_KEY_SIZE))
            {
                UWORD8 i = 0;

                /* Key length is twice the ASCII value */
                temp_cred_buff[index++] = (key_len * 2);

                /* Unpack each byte of the key and convert it into ASCII     */
                /* character                                                 */
                for(i=0; i < key_len; i++)
                {
                    UWORD8 upper_nibble = (key[i] >> 4);
                    UWORD8 lower_nibble = (key[i] & 0x0F);

                    temp_cred_buff[index++] = hex_2_char(upper_nibble);
                    temp_cred_buff[index++] = hex_2_char(lower_nibble);
                }

            }
            else
            {
                temp_cred_buff[index++] = key_len;
                memcpy(temp_cred_buff + index, key, key_len);
                index += key_len;
            }
        }
    }
#endif /* WPS_1_0_SEC_SUPP */
    else /* Non-WEP */
    {
        UWORD8 key_len = cred->net_key_len[0];
        UWORD8 *key    = cred->net_key[0];

        temp_cred_buff[index++] = WID_11I_PSK & 0xFF;
        temp_cred_buff[index++] = (WID_11I_PSK & 0xFF00) >> 8;
        temp_cred_buff[index++] = key_len;
        memcpy(temp_cred_buff + index, key, key_len);
        index += key_len;
    }

    return index;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : join_wps_cred                                            */
/*                                                                           */
/*  Description   : This function configures the MAC to join a particular    */
/*                  network based on the credential received.                */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_wps_cred                                               */
/*                  g_wps_curr_cred                                          */
/*                  g_wps_use_creds                                          */
/*                                                                           */
/*  Processing    : This function configures the MAC to join a particular    */
/*                  network based on the current credentials in the list of  */
/*                  credentials that is stored. The saved credential list is */
/*                  traversed till the given credential is reached and these */
/*                  are used to configure the MAC using the set functions    */
/*                  defined as part of configuration interface. Thereafter   */
/*                  MAC is restarted and a join timer is started.            */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void join_wps_cred(void)
{
    UWORD8  num_cred      = 0;
    UWORD8  cred_skipped  = 0;
    UWORD16 curr_cred_idx = 0;

    /* Get the number of credentials if there is a WPS credential buffer */
    num_cred = g_wps_cred_list[WPS_CRED_LIST_CNT_OFF];

    /* If there are no credentials stored take appropriate action and return */
    if(0 == num_cred)
    {
        /* Change to disabled state */
        disconnect_station(0);
        /* Send an error message to the host */
        send_wps_status(CRED_JOIN_LIST_NULL, NULL, 0);
        return;
    }

    /* If the current credential is greater than the number of credentials   */
    /* that are stored reset the current credential to 0. This is an         */
    /* exception and should never occur                                      */
    if(g_wps_curr_cred > num_cred)
    {
        g_wps_curr_cred = 0;
    }

    /* Skip credentials in the list until the current credential is reached  */
    curr_cred_idx = WPS_CRED_LIST_CRED_START_OFF;

    while(cred_skipped < g_wps_curr_cred)
    {
        curr_cred_idx += (1 + g_wps_cred_list[curr_cred_idx]);
        cred_skipped++;
    }
    g_wps_use_creds = WPS_CRED_BEING_PROGRAMMED;

    /* Configure MAC with the current credentials */
    process_write(&g_wps_cred_list[curr_cred_idx + 1],
                  g_wps_cred_list[curr_cred_idx]);

    /* Initiate join to the configured network */
    restart_mac(&g_mac, 0);
    g_wps_use_creds = WPS_CRED_PROGRAMMED;

    /* If a WPS credential join timer is not present, create and start the   */
    /* same for maximum network join time (includes scan to key handshake)   */
    if(NULL == g_wps_cred_join_timer)
    {
        g_wps_cred_join_timer = create_alarm(wps_cred_join_timeout_fn,
                                             JOIN_WPS_CRED_LIST, NULL);
        if(NULL == g_wps_cred_join_timer)
        {
            raise_system_error(NO_LOCAL_MEM);
            return;
        }
    }

    start_alarm(g_wps_cred_join_timer, MAX_WPS_NW_JOIN_TIME);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : wps_cred_join_timeout_fn                                 */
/*                                                                           */
/*  Description   : This function is the callback function for the WPS       */
/*                  credential join timer.                                   */
/*                                                                           */
/*  Inputs        : 1) Input data                                            */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function creates a miscellaneous event with name    */
/*                  WPS_CRED_JOIN_TIMEOUT, info as the Timer input data and  */
/*                  queues it in the High Priority queue.                    */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

#ifndef OS_LINUX_CSL_TYPE
void wps_cred_join_timeout_fn(HANDLE_T* alarm, ADDRWORD_T data)
#else /* OS_LINUX_CSL_TYPE */
void wps_cred_join_timeout_fn(ADDRWORD_T data)
#endif /* OS_LINUX_CSL_TYPE */
{
    misc_event_msg_t *misc = 0;
    TROUT_DBG4("WPS: cred join timeout fn!");

    misc = (misc_event_msg_t*)event_mem_alloc(MISC_EVENT_QID);

    if(NULL == misc)
    {
        raise_system_error(NO_EVENT_MEM);
        return;
    }

    misc->data = 0;
    misc->info = data;
    misc->name = WPS_CRED_JOIN_TIMEOUT;
    post_event((UWORD8*)misc, MISC_EVENT_QID);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : handle_wps_cred_join_timeout                             */
/*                                                                           */
/*  Description   : This function handles the WPS credential join timeout.   */
/*                                                                           */
/*  Inputs        : 1) Pointer to the WPS credential join timeout event      */
/*                                                                           */
/*  Globals       : g_wps_cred_join_timer                                    */
/*                  g_wps_cred_list                                          */
/*                  g_wps_curr_cred                                          */
/*                  g_wps_use_creds                                          */
/*                                                                           */
/*  Processing    : This function deletes the WPS credential join timer. It  */
/*                  then checks if the STA has been able to connect to the   */
/*                  network with required credentials successfully.          */
/*                  In case the timer data indicates that this was as a      */
/*                  result of a disconnect and reconnect, no action needs to */
/*                  be taken if the STA has been able to connect. If not a   */
/*                  message must be sent to the host with appropriate status */
/*                  in the Host Managed mode.                                */
/*                  In case the timer data indicates that this was initiated */
/*                  by a WPS join, if the STA has been able to connect, the  */
/*                  current credential with which the STA successfully       */
/*                  connected is saved in EEPROM (in Standalone mode) or is  */
/*                  sent to the host (in Host Managed mode). Also the WPS    */
/*                  credential list buffer is freed. In case the STA has not */
/*                  been able to connect, it continues to try to join with   */
/*                  the next credential in the list. If the list is over it  */
/*                  restarts from the first one. In such a case it also      */
/*                  sends a message to the host with appropriate status in   */
/*                  Host Managed mode.                                       */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void handle_wps_cred_join_timeout(misc_event_msg_t *misc)
{
    BOOL_T sta_connected = BFALSE;
    TROUT_DBG4("WPS: handle wps cred join timeout!\n");

	TROUT_FUNC_ENTER;
    /* Delete the WPS credential join timer */
    delete_alarm(&g_wps_cred_join_timer);
    g_wps_cred_join_timer = NULL;

    if((WPS_ENABLED == get_wps_mode()) ||
        (WPS_CRED_PROGRAMMED != g_wps_use_creds))
    {
		TROUT_FUNC_EXIT;
        return;
    }
    /* Check if the STA is connected currently (includes any key handshake) */
    sta_connected = is_sta_connected();

    switch(misc->info)
    {
    case JOIN_WPS_CRED_LIST:
    {
        if(sta_connected == BTRUE)
        {
            UWORD8  cred_skipped  = 0;
            UWORD8  curr_cred_len = 0;
            UWORD16 curr_cred_idx = 0;
            UWORD8  *curr_cred    = 0;

            /* Get pointer to the current credential that the STA has been   */
            /* able to connect with                                          */
            curr_cred_idx = WPS_CRED_LIST_CRED_START_OFF;

            while(cred_skipped < g_wps_curr_cred)
            {
                curr_cred_idx += (1 + g_wps_cred_list[curr_cred_idx]);
                cred_skipped++;
            }

            /* Points to the start of the current credential element in the  */
            /* list. The first byte is the credential length. Following      */
            /* bytes is the actual credential (WIDs)                         */
            curr_cred     = &g_wps_cred_list[curr_cred_idx];
            curr_cred_len = g_wps_cred_list[curr_cred_idx];

            /* ------------------------------------------------------------- */
            /* Credential format (to store in EEPROM or message to host)     */
            /* ------------------------------------------------------------- */
            /* | Credential Value                                          | */
            /* ------------------------------------------------------------- */
            /* | cred_len bytes (WID_SSID, ... WID_BSSID)                  | */
            /* ------------------------------------------------------------- */

            /* Based on the device mode update EEPROM with this credential   */
            /* or send the credential to the host with required status.      */
            if(get_wps_dev_mode_enr() == WPS_STANDALONE_DEVICE)
            {
                if(0 == g_cred_saved_in_eeprom)
                {
                    write_wps_cred_to_eeprom(&curr_cred[1], curr_cred_len);
                    g_cred_saved_in_eeprom = 1;
                }
            }
            else /* HOST_MANAGED_DEVICE */
            {
                send_wps_status(CRED_JOIN_SUCCESS, &curr_cred[1],
                                curr_cred_len);
            }
        }
        else
        {
            UWORD8 num_cred = g_wps_cred_list[WPS_CRED_LIST_CNT_OFF];

            /* Increment the current credential count */
            g_wps_curr_cred++;

            /* If the current credential is greater than the number of       */
            /* credentials that are stored reset the current credential to 0 */
            /* and attempt to join the list from the beginning but inform    */
            /* the host that joining has failed in case of Host Managed mode */
            if(g_wps_curr_cred > num_cred)
            {
                g_wps_curr_cred  = 0;

                if(get_wps_dev_mode_enr() == WPS_HOST_MANAGED_DEVICE)
                    send_wps_status(CRED_JOIN_FAILURE, NULL, 0);
            }

            /* Try to join the next credential in the list */
            join_wps_cred();
        }
    }
    break;

    case REJOIN_WPS_CRED:
    {
        if(sta_connected == BFALSE)
        {
            if(get_wps_dev_mode_enr() == WPS_HOST_MANAGED_DEVICE)
                send_wps_status(CRED_JOIN_FAILURE, NULL, 0);
        }
    }
    break;

    default:
    {
        /* Unknown timeout event. Do nothing. */
    }
    break;
    }
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_wsc_info_element                                     */
/*                                                                           */
/*  Description   : This function sets the WSC IE in the given frame.        */
/*                                                                           */
/*  Inputs        : 1) Pointer to the frame                                  */
/*                  2) Index at which the WSC IE needs to be set             */
/*                  3) Frame type                                            */
/*                                                                           */
/*  Globals       : g_wps_enrollee                                           */
/*                                                                           */
/*  Processing    : This function checks if WPS mode is ENABLED and WSC IE   */
/*                  is enabled. If so based on the compile-time setting and  */
/*                  frame type the WPS Enrollee function is called to set    */
/*                  this WSC IE.                                             */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : UWORD16, Length of the WSC IE                             */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD16 set_wsc_info_element(UWORD8 *data, UWORD16 index, UWORD8 frm_type)
{
    UWORD16 ret_val = 0;
    /* If user has disabled WSC IE, return 0 */
    if(WPS_ENABLED == get_wps_mode())
    {
        if (NULL != g_wps_enrollee_ptr) //wxb add if
        {
           ret_val = wps_gen_wsc_ie(g_wps_enrollee_ptr, data+index, frm_type);
        }
    } /* if(WPS_ENABLED == get_wps_mode()) */
    return ret_val;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : handle_mlme_rsp_wps_sta                                  */
/*                                                                           */
/*  Description   : This function handles the MLME response for WPS STA.     */
/*                                                                           */
/*  Inputs        : 1) MLME Response type                                    */
/*                  2) MLME Response message                                 */
/*                                                                           */
/*  Globals       : g_wps_enrollee_ptr                                       */
/*                                                                           */
/*  Processing    : This function calls the WPS Enrollee function to handle  */
/*                  association completion with the appropriate status.      */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : UWORD8, Length of the WSC IE                             */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void handle_mlme_rsp_wps_sta(UWORD8 rsp_type, UWORD8 *rsp_msg)
{
    UWORD8 status = 0;

    /* Initialize the status to Timeout */
    status = WPS_ASSOC_TIMEOUT;

    /* Only for Association response update the status to SUCCESS if the     */
    /* Association response result code is SUCCESS_MLMESTATUS indicating     */
    /* Successful association and FAILURE if the result code is neither      */
    /* SUCCESS_MLMESTATUS nor TIMEOUT, indicating that association failed.   */
    /* For all other cases this function will be called in case of Timeouts  */
    /* and hence the default TIMEOUT status shall be sent.                   */
    if(MLME_ASOC_RSP == rsp_type)
    {
        asoc_rsp_t *asoc_rsp = (asoc_rsp_t *)rsp_msg;
        UWORD8     result    = asoc_rsp->result_code;

        if(result == SUCCESS_MLMESTATUS) /* Successful association */
            status = WPS_ASSOC_SUCCESS;
        else if(result != TIMEOUT) /* Failed to associate (not Timeout) */
            status = WPS_ASSOC_FAILURE;
    }

    /* Call the WPS association complete function with the status */
    wps_assoc_comp(g_wps_enrollee_ptr, (WPS_ASSOC_STATUS_T)status);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : handle_wps_event_sta                                     */
/*                                                                           */
/*  Description   : This function handles WPS events in STA mode.            */
/*                                                                           */
/*  Inputs        : 1) Event                                                 */
/*                                                                           */
/*  Globals       : g_wps_enrollee_ptr                                       */
/*                                                                           */
/*  Processing    : This function calls the required WPS event handling      */
/*                  functions (internal WPS SW function or WPS Enrollee      */
/*                  function) with the required inputs. Based on if the      */
/*                  event type is known and processed by WPS, the status is  */
/*                  returned.                                                */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : BOOL_T; BTRUE if the event is handled; Else BFALSE       */
/*                                                                           */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

BOOL_T handle_wps_event_sta(UWORD32 event)
{
    BOOL_T           event_handled = BFALSE;
    misc_event_msg_t *wps_event    = (misc_event_msg_t *)event;

    TROUT_FUNC_ENTER;
    switch(wps_event->name)
    {
    case WPS_CRED_JOIN_TIMEOUT:
    {
        handle_wps_cred_join_timeout(wps_event);
        event_handled = BTRUE;
    }
    break;
    case WPS_TIMEOUT_EVENT:
    {
        if(WPS_ENABLED == get_wps_mode())
        {
            wps_handle_event(g_wps_enrollee_ptr, wps_event->info);
            event_handled = BTRUE;
        }
    }
    break;
    default:
    {
        /* Unknown WPS event type. Do nothing. */
    }
    break;
    }

    TROUT_FUNC_EXIT;
    return event_handled;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : process_wps_pkt_sta                                      */
/*                                                                           */
/*  Description   : This function processes WPS EAP messages.                */
/*                                                                           */
/*  Inputs        : 1) Pointer to the received frame                         */
/*                  2) Length of the received frame                          */
/*                                                                           */
/*  Globals       : g_wps_enrollee_ptr                                       */
/*                                                                           */
/*  Processing    : This function calls the WPS Enrollee function to process */
/*                  an EAP message.                                          */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void process_wps_pkt_sta(UWORD8 *rx_data, UWORD16 rx_len)
{
    BOOL_T b_beacon_filter, b_tbtt_masked;
    /* Mask the TBTT interrupt and disable beacon reception by SW this is   */
    /* done to avoid link loss that occurs due to numerious TBTT events     */
    /* queued during the long processing time taken by WPS                  */
    b_beacon_filter = is_bcn_filter_on();
    b_tbtt_masked   = is_machw_tbtt_int_masked();
    mask_machw_tbtt_int();
    enable_machw_beacon_filter();
    wps_process_eap_msg(g_wps_enrollee_ptr, rx_data, rx_len);
    if(BFALSE == b_beacon_filter)
    {
        disable_machw_beacon_filter();
    }
    if(BFALSE == b_tbtt_masked)
    {
        unmask_machw_tbtt_int();
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : handle_wps_scan_rsp_sta                                  */
/*                                                                           */
/*  Description   : This function handles scan response frames in WPS mode   */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC structure                          */
/*                  2) Pointer to the beacon/probe response frame            */
/*                  3) Length of the received frame                          */
/*                  4) RSSI of the received frame                            */
/*                                                                           */
/*  Globals       : g_wps_enrollee_ptr                                       */
/*                                                                           */
/*  Processing    : This function checks if the channel is within the        */
/*                  specified channels. If not nothing is done. Otherwise    */
/*                  the WPS enrollee function to process the probe response  */
/*                  is called. Based on the status returned by this function */
/*                  the BSS descriptor set is updated with relevant info     */
/*                  and join may be initiated.                               */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void handle_wps_scan_rsp_sta(UWORD8 *mac, UWORD8 *msa, UWORD16 rx_len,
                             WORD8 rssi)
{
    UWORD8  wps_action  = 0;

	TROUT_FUNC_ENTER;
#if 0 /* TBD - channel enable check is not done in 11n code yet */
    UWORD8  rx_channel  = 0;
    UWORD32 channel_bit = 0;

    /* Check if the channel of received frame is within the channels that    */
    /* have been specified. If not, no further processing is required.       */
    rx_channel = get_current_channel(msa, rx_len) - 1;
    channel_bit = (0x00000001 << rx_channel);

    if(!(get_enable_ch() & channel_bit))
        return;
#endif /* TBD - channel enable check is not done in 11n code yet */

    /* Call the WPS enrollee function to process the received frame */
    wps_action = wps_process_scan_rsp(g_wps_enrollee_ptr, msa, rx_len);

    switch(wps_action)
    {
    case SKIP_FRAME:
    {
        /* Do nothing. Skip this frame and do not update the BSS descriptor */
    }
    break;
    case KEEP_FRAME:
    {
        /* Update the BSS descriptor with this (the only) entry */
        g_bss_dscr_set_index = 0;
        update_bss_dscr_set(msa, rx_len, rssi, g_bss_dscr_set_index);
        g_bss_dscr_set[g_bss_dscr_set_index].cap_info &= (~PRIVACY);
    }
    break;
    case JOIN_NW:
    {
        scan_rsp_t scan_rsp_buf = {0};
        scan_rsp_t *scan_rsp = &scan_rsp_buf;

        /* Update the BSS descriptor with this (the only) entry */
        g_bss_dscr_set_index = 0;
        update_bss_dscr_set(msa, rx_len, rssi, g_bss_dscr_set_index);
        g_bss_dscr_set[g_bss_dscr_set_index].cap_info &= (~PRIVACY);
        g_bss_dscr_set_index = 1;

        /* Initiate join to the only BSS descriptor stored */
        set_join_req_sta(0);

        /* Set the MAC state to SCAN_COMP, prepare and send a scan response */
        set_mac_state(SCAN_COMP);

        prepare_scan_response(scan_rsp);

        /* Send the response to host */
        send_mlme_rsp_to_host(((mac_struct_t *)mac), MLME_SCAN_RSP,
                              (UWORD8 *)scan_rsp);
    }
    break;
    default:
    {
        raise_system_error(SYSTEM_ERROR);
    }
    break;
    }
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : handle_wps_scan_complete_sta                             */
/*                                                                           */
/*  Description   : This function handles scan completion in WPS mode        */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC structure                          */
/*                                                                           */
/*  Globals       : g_wps_enrollee_ptr                                       */
/*                                                                           */
/*  Processing    : This function calls the WPS enrollee function to process */
/*                  scan completion. Based on the status returned by this    */
/*                  function the station either restarts scanning or         */
/*                  initiates a join.                                        */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void handle_wps_scan_complete_sta(UWORD8 *mac)
{
    UWORD8 wps_action = 0;

	TROUT_FUNC_ENTER;
    /* Call the WPS enrollee function to process scan completion */
    wps_action = wps_scan_complete(g_wps_enrollee_ptr);

    switch(wps_action)
    {
    case CONT_SCAN:
    {
        g_channel_index = 0;
        scan_channel(g_channel_list[g_channel_index]);
    }
    break;
    case JOIN_NW:
    {
        scan_rsp_t scan_rsp_buf = {0};
        scan_rsp_t *scan_rsp = &scan_rsp_buf;

        /* Initiate join to the only BSS descriptor stored */
        g_bss_dscr_set_index = 1;
        set_join_req_sta(0);

        /* Set the MAC state to SCAN_COMP, prepare and send a scan response */
        set_mac_state(SCAN_COMP);

        /* Update the scan response message with required fields */
        prepare_scan_response(scan_rsp);

        /* Send the response to host */
        send_mlme_rsp_to_host(((mac_struct_t *)mac), MLME_SCAN_RSP,
                              (UWORD8 *)scan_rsp);
    }
    break;
    default:
    {
        raise_system_error(SYSTEM_ERROR);
    }
    break;
    }
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : start_wps_cred_rejoin_timer                              */
/*                                                                           */
/*  Description   : This function starts a network rejoin timer.             */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_wps_cred_join_timer                                    */
/*                  g_wps_use_creds                                          */
/*                                                                           */
/*  Processing    : This function checks if a WPS credential join timer is   */
/*                  already present. If so, do nothing (since it implies     */
/*                  that the timer is already running and on timeout the     */
/*                  appropriate action shall be taken). If not, create the   */
/*                  timer with input data set to REJOIN_WPS_CRED and start   */
/*                  the same for MAX_WPS_NW_JOIN_TIME.                       */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void start_wps_cred_rejoin_timer(void)
{
    if((WPS_ENABLED == get_wps_mode()) ||
        (WPS_CRED_PROGRAMMED != g_wps_use_creds))
    {
        return;
    }

    /* If a WPS credential join timer is not present, create and start the   */
    /* same for maximum network join time (includes scan to key handshake)   */
    if(NULL == g_wps_cred_join_timer)
    {
        g_wps_cred_join_timer = create_alarm(wps_cred_join_timeout_fn,
                                             REJOIN_WPS_CRED, NULL);
        if(NULL == g_wps_cred_join_timer)
        {
            raise_system_error(NO_LOCAL_MEM);
            return;
        }
    }
    start_alarm(g_wps_cred_join_timer, MAX_WPS_NW_JOIN_TIME);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : send_wps_cred_list                                       */
/*                                                                           */
/*  Description   : This function sends the WPS Credential list as an        */
/*                  Information packet to host                               */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_wps_cred_list                                          */
/*                                                                           */
/*  Processing    : This function prepares an Information type configuration */
/*                  packet with WID_WPS_CRED_LIST that includes the          */
/*                  credential list. The same is then sent to the host.      */
/*                  Required buffers are allocated and freed.                */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void send_wps_cred_list(void)
{
    UWORD8       *info_msg          = 0;
    UWORD8       *info_buf          = 0;
    UWORD16      wps_status_msg_len = 0;
    UWORD16      cred_len           = 0;

    info_buf = (UWORD8*)pkt_mem_alloc(MEM_PRI_CONFIG);

    if(info_buf == NULL)
    {
        return;
    }

    info_msg = info_buf + get_config_pkt_hdr_len(get_host_config_if_type());

    cred_len = (UWORD16)g_wps_cred_list[WPS_CRED_LIST_LEN_OFF] +
               ((UWORD16)g_wps_cred_list[WPS_CRED_LIST_LEN_OFF+1] << 8);
    wps_status_msg_len = MSG_DATA_OFFSET + WID_LENGTH_OFFSET + 2 + cred_len;

    /* Prepare the WPS Credential list message to be sent to the host        */

    /* WID_WPS_CRED_LIST (Binary WID) Message Format                         */
    /* --------------------------------------------------------------------- */
    /* | Message Type | ID | Length | WID Type | WID Length | WID Data     | */
    /* --------------------------------------------------------------------- */
    /* | 1            | 1  | 2      | 2        | 2          | cred_len     | */
    /* --------------------------------------------------------------------- */
    info_msg[0] = 'I';
    info_msg[1] = g_info_id++;
    info_msg[2] = (wps_status_msg_len) & 0xFF;
    info_msg[3] = ((wps_status_msg_len) & 0xFF00) >> 8;
    info_msg[4] = WID_WPS_CRED_LIST & 0xFF;
    info_msg[5] = (WID_WPS_CRED_LIST >> 8) & 0xFF;

    /* Copy the data including the length field                              */
    memcpy((info_msg + 6), g_wps_cred_list, (cred_len + 2));

    /* Send the information message to the host                              */
    send_host_rsp(info_buf, wps_status_msg_len, get_host_config_if_type());
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_wps_cred_list_enr                                    */
/*                                                                           */
/*  Description   : This function sets the WID_WPS_CRED_LIST                 */
/*                                                                           */
/*  Inputs        : 1) Pointer to the WPS credentials list                   */
/*                                                                           */
/*  Globals       : g_wps_enrollee_ptr                                       */
/*                  g_wps_cred_list                                          */
/*                  g_wps_curr_cred                                          */
/*                  g_config_write_from_host                                 */
/*                                                                           */
/*  Processing    : This function is valid only in Host Managed mode. If the */
/*                  WPS credential list buffer is not yet allocated, the     */
/*                  same is allocated. If WPS is ENABLED, the Enrollee is    */
/*                  stopped, mode DISABLED and enrollee pointer freed. The   */
/*                  incoming WPS credentials are copied to the buffer        */
/*                  allocated (no format change required as they are already */
/*                  in the format required - WPS_CRED_LIST format). The      */
/*                  current credential index is reset to 0 and the function  */
/*                  join_wps_cred is called.                                 */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void set_wps_cred_list_enr(UWORD8 *wps_cred_list)
{
    UWORD16 wps_cred_list_len;
    BOOL_T   b_temp = g_config_write_from_host;
    if((get_wps_dev_mode_enr() != WPS_HOST_MANAGED_DEVICE) ||
        (BTRUE == g_reset_mac_in_progress))
    {
        return;
    }
    g_config_write_from_host = BFALSE;

    /* Reset the number of credentials in the global credential list buffer  */
    g_wps_cred_list[WPS_CRED_LIST_CNT_OFF] = 0;
    if(NULL != g_wps_cred_join_timer)
    {
        /* Delete the WPS credential join timer                              */
        delete_alarm(&g_wps_cred_join_timer);
        g_wps_cred_join_timer = NULL;
    }
    g_wps_use_creds = WPS_CRED_NOT_USED;
    disable_wps_mode();
    disconnect_station(0);

    /* get length of the Credential list Programmed                          */
    wps_cred_list_len  = wps_cred_list[0];
    wps_cred_list_len |= ((UWORD16)wps_cred_list[1] << 8) & 0xFF00;
    wps_cred_list_len &= WID_BIN_DATA_LEN_MASK;
    wps_cred_list_len += 2;
    if(wps_cred_list_len > MAX_WPS_CRED_LIST_SIZE)
    {
        /* Length exception                                                  */
        send_wps_status(REC_INVALID_CRED_JOIN_LIST, NULL, 0);
        g_config_write_from_host = b_temp;
        return;
    }
    /* --------------------------------------------------------------------- */
    /* Format of the credentials list buffer (g_wps_cred_list) which is the  */
    /* same as the incoming WID_WPS_CRED_LIST Value format. Hence no format  */
    /* conversion is required. It can be directly copied.                    */
    /* --------------------------------------------------------------------- */
    /* | Length |Num Creds | Credential 1 Len | Credential 1 Value          |*/
    /* --------------------------------------------------------------------- */
    /* |  2b    |1b (n)    | 1 byte (cred_len)| cred_len(WID_SSID.WID_BSSID)|*/
    /* ----------------------------------------------------------------------*/
    memcpy(g_wps_cred_list, wps_cred_list, wps_cred_list_len);
    /* Initiate join from the first credential in the list                   */
    g_wps_curr_cred = 0;
    if(0 != g_wps_cred_list[WPS_CRED_LIST_CNT_OFF])
    {
        join_wps_cred();
    }
    else
    {
        send_wps_status(REC_INVALID_CRED_JOIN_LIST, NULL, 0);
    }
    g_reset_mac = BFALSE;
    g_config_write_from_host = b_temp;
    return;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : get_wps_cred_list_enr                                    */
/*                                                                           */
/*  Description   : This function returns the WID_WPS_CRED_LIST              */
/*                                                                           */
/*  Inputs        : 1) None                                                  */
/*                                                                           */
/*  Globals       : g_wps_cred_list                                          */
/*                                                                           */
/*  Processing    : This function checks if there are credentials stored in  */
/*                  g_wps_cred_list, if credentials are present it returns   */
/*                  pointer to g_wps_cred_list, else it return NULL          */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : Pointer to credential list                               */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8* get_wps_cred_list_enr(void)
{
    if(((0 == g_wps_cred_list[WPS_CRED_LIST_LEN_OFF]) &&
        (0 == g_wps_cred_list[WPS_CRED_LIST_LEN_OFF+1])) ||
        (0 == g_wps_cred_list[WPS_CRED_LIST_CNT_OFF]))
    {
        return NULL;
    }
    return g_wps_cred_list;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_wps_dev_mode_enr                                     */
/*                                                                           */
/*  Description   : This function set the WPS device mode (Host managed or   */
/*                  standalone device mode)                                  */
/*                                                                           */
/*  Inputs        : WPS device mode                                          */
/*                                                                           */
/*  Globals       : g_wps_params                                             */
/*                  g_config_write_from_host                                 */
/*                                                                           */
/*  Processing    : This function sets the wps device mode                   */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None                                                     */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void set_wps_dev_mode_enr(UWORD8 val)
{
    BOOL_T   b_temp = g_config_write_from_host;
    if(((WPS_STANDALONE_DEVICE != val) && (WPS_HOST_MANAGED_DEVICE != val))
        || (val == get_wps_dev_mode_enr()) || (BTRUE == g_reset_mac_in_progress))
    {
        return;
    }
    g_config_write_from_host = BFALSE;

    disable_wps_mode();
    disconnect_station(0);
    /* Reset the length and number of credentials in the global credential   */
    /* list buffer                                                           */
    g_wps_cred_list[WPS_CRED_LIST_LEN_OFF] = 0;
    g_wps_cred_list[WPS_CRED_LIST_LEN_OFF+1] = 0;
    g_wps_cred_list[WPS_CRED_LIST_CNT_OFF] = 0;
    g_wps_dev_mode = val;
    if(NULL != g_wps_cred_join_timer)
    {
        /* Delete the WPS credential join timer                              */
        delete_alarm(&g_wps_cred_join_timer);
        g_wps_cred_join_timer = NULL;
    }
    if(WPS_STANDALONE_DEVICE == val)
    {
	// 20120709 caisf add, merged ittiam mac v1.2 code
#ifdef MAC_P2P
        if(BFALSE == mget_p2p_enable())
#endif /* MAC_P2P */
        join_wps_cred_from_eeprom();
    }
    g_config_write_from_host = b_temp;
    return;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : join_wps_cred_from_eeprom                                */
/*                                                                           */
/*  Description   : This function gets the WPS credential from EEPROM and    */
/*                  initiates a join with the same.                          */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_wps_enrollee_ptr                                       */
/*                  g_wps_cred_list                                          */
/*                  g_wps_curr_cred                                          */
/*                                                                           */
/*  Processing    : This function is valid only in Standalone mode. If the   */
/*                  WPS credential list buffer is not yet allocated, the     */
/*                  same is allocated. If WPS is ENABLED, the Enrollee is    */
/*                  stopped, mode DISABLED and enrollee pointer freed. The   */
/*                  WPS credentials are read from EEPROM in the format       */
/*                  required (WPS_CRED_LIST format). The current credential  */
/*                  index is reset to 0 and the function join_wps_cred is    */
/*                  called.                                                  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void join_wps_cred_from_eeprom(void)
{
	TROUT_FUNC_ENTER;
	
    if(get_wps_dev_mode_enr() != WPS_STANDALONE_DEVICE)
    {
		TROUT_FUNC_EXIT;
        return;
    }

    /* Reset the length and number of credentials in the global credential  */
    /* list buffer                                                          */
    g_wps_cred_list[WPS_CRED_LIST_LEN_OFF] = 0;
    g_wps_cred_list[WPS_CRED_LIST_LEN_OFF+1] = 0;
    g_wps_cred_list[WPS_CRED_LIST_CNT_OFF] = 0;

    /* Read value from EEPROM to this buffer in the WID_WPS_CRED_LIST format */
    read_wps_cred_from_eeprom(g_wps_cred_list);

    disable_wps_mode();

    /* Try to join from the first credential */
    g_wps_curr_cred = 0;

    join_wps_cred();
    
	TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : read_wps_cred_from_eeprom                                */
/*                                                                           */
/*  Description   : This function reads the WPS credentials from EEPROM to   */
/*                  the given buffer in the required format.                 */
/*                                                                           */
/*  Inputs        : 1) Pointer to the WPS credentials list buffer            */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function reads the WPS credential (currently one)   */
/*                  saved in the EEPROM and updates the given buffer with    */
/*                  the same in the format of WID_WPS_CRED_LIST.             */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void read_wps_cred_from_eeprom(UWORD8 *cred_list)
{
    wps_store_cred_t eeprom_data;
    wps_store_cred_t *cred        = 0;
    UWORD8           index        = 0;
    BOOL_T           cred_valid   = BTRUE;
    UWORD8           ssid_len     = 0;

    //caisf add
    memset(&eeprom_data,0,sizeof(wps_store_cred_t));

    cred = &eeprom_data;
    /* Read credential from Persistent memory                                */
    read_cred_from_pers_mem(cred);

    ssid_len = strlen((WORD8 *)cred->ssid);

    /* Check if credential is valid, if not then make credential count =0    */
    /* and return. SSID Length, Key length, Security mode and  Number of keys*/
    /* are checked                                                           */
    if(((0 == ssid_len) || (MAX_SSID_LEN <= ssid_len)) ||
      (SEC_MODE_MASK < cred->sec_mode))
    {
        cred_list[0] = 0;
        return;
    }
    switch(cred->sec_mode)
    {
#ifdef WPS_1_0_SEC_SUPP
        case SEC_MODE_WEP40:
        {
            if((WEP40_KEY_SIZE != cred->key_len) ||
              (0 == cred->key.wep_key.num_key) ||
              (NUM_DOT11WEPDEFAULTKEYVALUE < cred->key.wep_key.num_key))
            {
                cred_valid   = BFALSE;
            }
            break;
        }
        case SEC_MODE_WEP104:
        {
            if((WEP104_KEY_SIZE != cred->key_len) ||
              (0 == cred->key.wep_key.num_key) ||
              (NUM_DOT11WEPDEFAULTKEYVALUE < cred->key.wep_key.num_key))
            {
                cred_valid   = BFALSE;
            }
            break;
        }
#endif /* WPS_1_0_SEC_SUPP */
        case SEC_MODE_NONE:
        {
            break;
        }
        default: /* Non-WEP */
        {
            if((MAX_PSK_PASS_PHRASE_LEN < cred->key_len) ||
              (0 == cred->key_len))
            {
                cred_valid   = BFALSE;
            }
            break;
        }
    }
    if(BFALSE == cred_valid)
    {
        cred_list[0] = 0;
        return;
    }

    /* --------------------------------------------------------------------- */
    /* Format in EEPROM                                                      */
    /* --------------------------------------------------------------------- */
    /* | wps_store_cred_t                                                  | */
    /* --------------------------------------------------------------------- */

    /* --------------------------------------------------------------------- */
    /* Format in the buffer to which the credentials need to be written      */
    /* --------------------------------------------------------------------- */
    /* | Length |Num Creds |Credential 1 Len |Credential 1 Value           | */
    /* --------------------------------------------------------------------- */
    /* |  2b    |1b (n)    |1 byte (cred_len)|cred_len (WID_SSID.WID_BSSID)| */
    /* --------------------------------------------------------------------- */

    /* Set the number of credentials to 1 */
    cred_list[WPS_CRED_LIST_CNT_OFF] = 1;

    /* Skip the length field. To be filled in the end */
    index = WPS_CRED_LIST_CRED_START_OFF+1;

    /* Update the WPS credential structure to the WPS credential list buffer */

    /* SSID WID-Length-Value */
    cred_list[index++] = WID_SSID & 0xFF;
    cred_list[index++] = (WID_SSID & 0xFF00) >> 8;
    cred_list[index++] = ssid_len;
    memcpy(cred_list + index, cred->ssid, ssid_len);
    index += ssid_len;

    /* 802.11i Mode WID-Length-Value */
    cred_list[index++] = WID_11I_MODE & 0xFF;
    cred_list[index++] = (WID_11I_MODE & 0xFF00) >> 8;
    cred_list[index++] = 1;
    cred_list[index++] = cred->sec_mode;

    /* Auth Type WID-Length-Value */
    cred_list[index++] = WID_AUTH_TYPE & 0xFF;
    cred_list[index++] = (WID_AUTH_TYPE & 0xFF00) >> 8;
    cred_list[index++] = 1;
    cred_list[index++] = cred->auth_type;

    if(cred->sec_mode == SEC_MODE_NONE) /* No encryption */
    {
        /* Do nothing */
    }
#ifdef WPS_1_0_SEC_SUPP
    else if((cred->sec_mode == SEC_MODE_WEP40) ||
            (cred->sec_mode == SEC_MODE_WEP104))
    {
		UWORD8        i        = 0;
        UWORD8        key_len  = cred->key_len;
        wps_wep_key_t *wep_key = &(cred->key.wep_key);
        UWORD8        num_key  = wep_key->num_key;

        for(i = 0; i < num_key; i ++)
        {
            UWORD8 key_id  = wep_key->key_id[i];
            UWORD8 *key    = wep_key->key[i];
            UWORD8 cnt     = 0;

            cred_list[index++] = WID_KEY_ID & 0xFF;
            cred_list[index++] = (WID_KEY_ID & 0xFF00) >> 8;
            cred_list[index++] = 1;
            cred_list[index++] = key_id;

            cred_list[index++] = WID_WEP_KEY_VALUE & 0xFF;
            cred_list[index++] = (WID_WEP_KEY_VALUE & 0xFF00) >> 8;
            /* In EEPROM the WEP keys are stored as hex characters, convert  */
            /* them to string                                                */
            cred_list[index++] = 2 * key_len;
            for(cnt = 0; cnt < key_len; cnt++)
            {
                cred_list[index++] = hex_2_char(((key[cnt] & 0xF0) >> 4));
                cred_list[index++] = hex_2_char((key[cnt] & 0x0F));
            }
        }
    }
#endif /* WPS_1_0_SEC_SUPP */
    else /* Non-WEP */
    {
        UWORD8 key_len = cred->key_len;
        UWORD8 *key    = cred->key.psk;

        cred_list[index++] = WID_11I_PSK & 0xFF;
        cred_list[index++] = (WID_11I_PSK & 0xFF00) >> 8;
        cred_list[index++] = key_len;
        memcpy(cred_list + index, key, key_len);
        index += key_len;
    }

    /* Set the length field */
    cred_list[WPS_CRED_LIST_CRED_START_OFF] =
        index-(WPS_CRED_LIST_CRED_START_OFF+1);
    index -= WPS_CRED_LIST_CNT_OFF;
    cred_list[WPS_CRED_LIST_LEN_OFF] = (UWORD8)(index & 0xFF);
    cred_list[WPS_CRED_LIST_LEN_OFF+1] = (UWORD8)(index >> 8);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : write_wps_cred_to_eeprom                                 */
/*                                                                           */
/*  Description   : This function writes a single WPS credential from the    */
/*                  given buffer to the EEPROM in the required format.       */
/*                                                                           */
/*  Inputs        : 1) Pointer to the WPS credential buffer                  */
/*                  2) Length of the WPS credential list                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function reads the WPS credential (one or multiple) */
/*                  from the given buffer in the format of WID_WPS_CRED_LIST */
/*                  and writes it to the EEPROM in the required format.      */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void write_wps_cred_to_eeprom(UWORD8 *cred, UWORD8 cred_len)
{
    wps_store_cred_t store_cred = {0};
    UWORD8           *temp      = 0;

    /* --------------------------------------------------------------------- */
    /* Format in the buffer in which 1 credential is stored                  */
    /* --------------------------------------------------------------------- */
    /* | Credential Value                                                  | */
    /* --------------------------------------------------------------------- */
    /* | cred_len bytes (WID_SSID, ... WID_BSSID)                          | */
    /* --------------------------------------------------------------------- */

    /* --------------------------------------------------------------------- */
    /* Format in EEPROM                                                      */
    /* --------------------------------------------------------------------- */
    /* | wps_store_cred_t                                                  | */
    /* --------------------------------------------------------------------- */

    store_cred.auth_type = mget_auth_type();
    store_cred.sec_mode  = get_802_11I_mode();
    strcpy((char*)store_cred.ssid, mget_DesiredSSID());

    if(store_cred.sec_mode == SEC_MODE_NONE) /* No encryption */
    {
        /* Do nothing */
    }
#ifdef WPS_1_0_SEC_SUPP
    else if((store_cred.sec_mode == SEC_MODE_WEP40) ||
            (store_cred.sec_mode == SEC_MODE_WEP104)) /* WEP */
    {
        UWORD8        i           = 0;
        UWORD8        cnt         = 0;
        UWORD8        curr_key_id = 0;
        wps_wep_key_t *wep_key    = &(store_cred.key.wep_key);

        /* Read the current Default WEP Key ID */
        curr_key_id = mget_WEPDefaultKeyID();

        if(store_cred.sec_mode == SEC_MODE_WEP40)
            store_cred.key_len = WEP40_KEY_SIZE;
        else
            store_cred.key_len = WEP104_KEY_SIZE;

        wep_key->num_key = NUM_DOT11WEPDEFAULTKEYVALUE;

        for(i = curr_key_id + 1; i < NUM_DOT11WEPDEFAULTKEYVALUE; i++)
        {
            mset_WEPDefaultKeyID(i);
            temp = mget_WEPDefaultKeyValue();
            wep_key->key_id[cnt] = i;
            memcpy(wep_key->key[cnt++], temp, store_cred.key_len);
        }

        for(i = 0; i < MIN(NUM_DOT11WEPDEFAULTKEYVALUE,(curr_key_id + 1)); i++)
        {
            mset_WEPDefaultKeyID(i);
            temp = mget_WEPDefaultKeyValue();
            wep_key->key_id[cnt] = i;
            memcpy(wep_key->key[cnt++], temp, store_cred.key_len);
        }

        /* Restore the Default WEP Key ID */
        mset_WEPDefaultKeyID(curr_key_id);
    }
#endif /* WPS_1_0_SEC_SUPP */
    else /* Non-WEP */
    {
        temp = get_RSNAConfigPSKPassPhrase();
        store_cred.key_len = temp[0];
        memcpy(store_cred.key.psk, &temp[1], store_cred.key_len);
    }

    /* Save credential to Persistent memory                                  */
    save_cred_to_pers_mem(&store_cred);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : wps_mem_alloc                                            */
/*                                                                           */
/*  Description   : This function allocates memory for WPS Library from      */
/*                  the scratch memory. This function can be called in a     */
/*                  nested manner upto MAX_NESTED_SCRATCH_MEM_ALLOC level    */
/*                                                                           */
/*  Inputs        : 1) Size of buffer to be allocated                        */
/*                                                                           */
/*  Globals       : g_wps_cur_scratch_indx                                   */
/*                  g_wps_scratch_mem_idx                                    */
/*                                                                           */
/*  Processing    : This function allocates the required sized memory from   */
/*                  the scratch memory.                                      */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : void *, Pointer to the buffer                            */
/*                                                                           */
/*  Issues        : Buffers allocated using this method must be freed in     */
/*                  appropriate order i.e. reverse of the allocations.       */
/*                  For example, if buffers a, b, c, d are allocated using   */
/*                  this function, the freeing should be done in the         */
/*                  following order - d, c, b, a.                            */
/*                                                                           */
/*****************************************************************************/

void* wps_mem_alloc(UWORD16 size)
{
    UWORD8 *buffer = NULL;

    /* Scratch memory index should always be 0 for a successful mem_alloc */
    if((g_wps_cur_scratch_indx >= MAX_NESTED_SCRATCH_MEM_ALLOC) ||
       (g_wps_scratch_mem_idx[g_wps_cur_scratch_indx] != 0))
    {
        PRINTD("Nested WPS Memory Allocation Exception\n");
        return 0;
    }

    g_wps_scratch_mem_idx[g_wps_cur_scratch_indx] = get_scratch_mem_idx();

    buffer = (UWORD8 *)scratch_mem_alloc(size);

    /* Restore and reset scratch memory index if allocation failed */
    if(buffer == NULL)
    {
        restore_scratch_mem_idx(g_wps_scratch_mem_idx[g_wps_cur_scratch_indx]);
        g_wps_scratch_mem_idx[g_wps_cur_scratch_indx] = 0;
    }
    else
    {
        g_wps_cur_scratch_indx++;
    }

    return buffer;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : wps_mem_free                                             */
/*                                                                           */
/*  Description   : This function frees memory for WPS.                      */
/*                                                                           */
/*  Inputs        : 1) Pointer to the buffer to be freed                     */
/*                                                                           */
/*  Globals       : g_wps_cur_scratch_indx                                   */
/*                  g_wps_scratch_mem_idx                                    */
/*                                                                           */
/*  Processing    : This function uses memory manager function to free given */
/*                  buffer from scratch memory.                              */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void wps_mem_free(void *buffer_addr)
{
    if(buffer_addr != NULL)
    {
        if(g_wps_cur_scratch_indx != 0)
            g_wps_cur_scratch_indx--;
        else
        {
            PRINTD("WPS Mem Free Exception\n");
            return;
        }
        restore_scratch_mem_idx(g_wps_scratch_mem_idx[g_wps_cur_scratch_indx]);
        g_wps_scratch_mem_idx[g_wps_cur_scratch_indx] = 0;
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : wps_frame_mem_alloc                                      */
/*                                                                           */
/*  Description   : This function allocates frame memory for WPS.            */
/*                                                                           */
/*  Inputs        : 1) Size of buffer to be allocated                        */
/*                                                                           */
/*  Globals       : g_shared_mem_handle                                      */
/*                                                                           */
/*  Processing    : This function uses memory manager function to allocate a */
/*                  buffer of required size from shared memory.              */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : void *, Pointer to the buffer                            */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void* wps_frame_mem_alloc(UWORD16 size)
{
    return pkt_mem_alloc(MEM_PRI_HPTX);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : wps_frame_mem_free                                       */
/*                                                                           */
/*  Description   : This function frees frame memory for WPS.                */
/*                                                                           */
/*  Inputs        : 1) Pointer to the buffer to be freed                     */
/*                                                                           */
/*  Globals       : g_shared_mem_handle                                      */
/*                                                                           */
/*  Processing    : This function uses memory manager function to free given */
/*                  buffer from shared memory.                               */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void wps_frame_mem_free(void *buffer_addr)
{
    pkt_mem_free(buffer_addr);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : save_cred_to_pers_mem                                    */
/*                                                                           */
/*  Description   : This function stores a single credential to persistent   */
/*                  memory. Currently nothing is done for MWLAN              */
/*                                                                           */
/*  Inputs        : 1) Pointer to credential to be saved                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function stores a single credential to persistent   */
/*                  memory. There can only be a single credential in the     */
/*                  persistent memory                                        */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void save_cred_to_pers_mem(wps_store_cred_t* store_cred)
{
    /* TBD for MWLAN */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : read_cred_from_pers_mem                                  */
/*                                                                           */
/*  Description   : This function reads a single credential from the         */
/*                  persistent memory. TBD for MWLAN                         */
/*                                                                           */
/*  Inputs        : 1) Pointer to credential memory where read credential    */
/*                  needs to be stored                                       */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function reads a single credential from the         */
/*                  persistent memory and writes it in to the buffer pointed */
/*                  by the input pointer.There can only be a single          */
/*                  credential in the persistent memory                      */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void read_cred_from_pers_mem(wps_store_cred_t *store_cred)
{
    /* TBD for MWLAN */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : init_wps_sta_globals                                     */
/*                                                                           */
/*  Description   : This function initializes the required global variables  */
/*                  for WPS in STA mode.                                     */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function initializes the WPS configuration method to*/
/*                  the default value.                                       */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void init_wps_sta_globals(void)
{
    set_wps_config_method((UWORD16) DEFAULT_WPS_CONFIG_METH);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_allow_sys_restart_sta                             */
/*                                                                           */
/*  Description      : This function handles the system erros during WPS     */
/*                     protocol                                              */
/*                                                                           */
/*  Inputs           : 1) System Error code                                  */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function handles the system erros during WPS     */
/*                     protocol. If its a link loss or remote disconnect then*/
/*                     inform WPS library                                    */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : BTRUE/BFALSE to indicate whether restart is required  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

BOOL_T wps_allow_sys_restart_sta(ERROR_CODE_T sys_error)
{
    BOOL_T ret_val = BTRUE;
    if((LINK_LOSS == sys_error) || (DEAUTH_RCD == sys_error))
    {
        STATUS_T wps_dis_proc;
        wps_dis_proc = wps_process_disconnect(g_wps_enrollee_ptr);
        if(SUCCESS == wps_dis_proc)
        {
            ret_val = BFALSE;
        }
    }
    else if(RESCAN_NEEDED != sys_error)
    {
        disable_wps_mode();
    }
    return ret_val;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : send_1x_fail_deauth                                   */
/*                                                                           */
/*  Description      : This function sends a deauth frame with result code as*/
/*                     801.1x failure                                        */
/*                                                                           */
/*  Inputs           : None                                                  */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : If the MAC state is enabled i.e. connected and BSSID  */
/*                     is valid then this functions sends a deauth frame with*/
/*                     result code as 802.1x failure                         */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/
void send_1x_fail_deauth(void)
{
    UWORD8  null_bssid[6] = {0};
    if(ENABLED == get_mac_state())
    {
        /* If non-zero BSSID is present,then send de-auth frame              */
        if(mac_addr_cmp(null_bssid, mget_bssid()) != BTRUE)
        {
            UWORD32 cnt = 0;
            /* Suspend the MAC HW For transmission. It is assumed that after */
            /* the suspension, MAC H/W Will not give any TX complete INT     */
            set_machw_tx_suspend();

            /* Flush all the QMU - queues                                    */
            flush_all_qs();
            /* Resume MAC HW Tx                                              */
            set_machw_tx_resume();
            send_deauth_frame(mget_bssid(), (UWORD16)IEEE_802_1X_AUTH_FAIL);
            /* Wait on the transmission of this deauth                       */
            while((is_machw_q_null(HIGH_PRI_Q) == BFALSE) &&
                (cnt < DEAUTH_SEND_TIME_OUT_COUNT))
            {
                add_delay(0xFFF);
                cnt++;
            }
        } /* if(mac_addr_cmp(null_bssid, mget_bssid()) != BTRUE) */
    } /* if(ENABLED == get_mac_state()) */
    /* Reset the STA Entry                                                   */
    delete_entry(mget_bssid());
    /* Reset the BSS ID so that no more deauth is sent                       */
    mset_bssid(null_bssid);
} /* void send_1x_fail_deauth(void) */

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : handle_host_non_wps_cred_req                          */
/*                                                                           */
/*  Description      : This function handles the case when individual        */
/*                     credentials are programmed by the Host                */
/*                                                                           */
/*  Inputs           : None                                                  */
/*                                                                           */
/*  Globals          : g_config_write_from_host                              */
/*  Processing       : This function checks if credentials from WPS cred list*/
/*                     are being used, if so then it remover the credential  */
/*                     timeout timer and .1x failure                         */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void handle_host_non_wps_cred_req(void)
{
	TROUT_FUNC_ENTER;
	
    if((BFALSE == g_reset_mac_in_progress) &&
        (BTRUE == g_config_write_from_host))
    {
		//chenq add 2012-11-01
		if(get_wps_mode() == WPS_DISABLED)
		{
			return;
		}
		
        disable_wps_mode();
        if(NULL != g_wps_cred_join_timer)
        {
            /* Delete the WPS credential join timer                          */
            delete_alarm(&g_wps_cred_join_timer);
            g_wps_cred_join_timer = NULL;
        }
        if(WPS_STANDALONE_DEVICE == get_wps_dev_mode_enr())
        {
            set_wps_dev_mode_sta(WPS_HOST_MANAGED_DEVICE);
        }
        g_wps_use_creds = WPS_CRED_NOT_USED;
    } /* if(BTRUE == g_config_write_from_host) */

    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_create_alarm                                      */
/*                                                                           */
/*  Description      : This function maps the WPS create alarm function to   */
/*                     system create alarm function                          */
/*                                                                           */
/*  Inputs           : 1) Pointer to the alarm call back function            */
/*                     2) Input data                                         */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function maps the WPS create alarm function to   */
/*                     system create  alarm function. Also it locks the alarm*/
/*                     bufffer                                               */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : Pointer to the alarm handle                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void* wps_create_alarm(WPS_ALARM_FUNC_T* func, UWORD32 data, ALARM_FUNC_WORK_T *work_func)
{
    void *handle = 0;

    handle = ((void *)create_alarm((ALARM_FUNC_T*) func, (ADDRWORD_T) data, work_func));

    /* Lock the buffer to keep it across resets */
    //lock_the_buffer(handle); //caisf fix it

    return handle;

}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_delete_alarm                                      */
/*                                                                           */
/*  Description      : This function maps the WPS delete alarm function to   */
/*                     system delete alarm function                          */
/*                                                                           */
/*  Inputs           : 1) Pointer to the alarm handle                        */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function maps the WPS delete alarm function to   */
/*                     system delete alarm function.                         */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void wps_delete_alarm(void** handle)
{
    //unlock_the_buffer(*handle);//caisf fix it
    delete_alarm((ALARM_HANDLE_T** )handle);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_start_alarm                                       */
/*                                                                           */
/*  Description      : This function maps the WPS start alarm function to    */
/*                     system start alarm function                           */
/*                                                                           */
/*  Inputs           : 1) Pointer to the alarm handle                        */
/*                     2) Timeout in miliseconds                             */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function maps the WPS start alarm function to    */
/*                     system start alarm function.                          */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : SUCCESS/FAILURE                                       */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

BOOL_T wps_start_alarm(void* handle, UWORD32 timeout_ms)
{
#ifndef OS_LINUX_CSL_TYPE
    return(start_alarm((ALARM_HANDLE_T *)handle, (UWORD32) timeout_ms));
#else /* OS_LINUX_CSL_TYPE */
    start_alarm((ALARM_HANDLE_T *)handle, (UWORD32) timeout_ms);

    return BFALSE;
#endif /* OS_LINUX_CSL_TYPE */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_stop_alarm                                        */
/*                                                                           */
/*  Description      : This function maps the WPS stop alarm function to     */
/*                     system stop alarm function                            */
/*                                                                           */
/*  Inputs           : 1) Pointer to the alarm handle                        */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function maps the WPS stop alarm function to     */
/*                     system stop alarm function.                           */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/
void wps_stop_alarm(void* handle)
{
    stop_alarm((ALARM_HANDLE_T *)handle);
}

#endif /* IBSS_BSS_STATION_MODE */
#endif /* INT_WPS_SUPP */
