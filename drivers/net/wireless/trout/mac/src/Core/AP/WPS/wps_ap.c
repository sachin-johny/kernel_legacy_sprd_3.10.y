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
/*  File Name         : wps_ap.c                                             */
/*                                                                           */
/*  Description       : This file contains the all the functions that        */
/*                      implement the SS functionality of WPS internal       */
/*                      Registrar                                            */
/*                                                                           */
/*  List of Functions : sys_init_wps_reg                                     */
/*                      sys_start_wps_reg                                    */
/*                      sys_stop_wps_reg                                     */
/*                      wps_allow_sys_restart_ap                             */
/*                      wps_handle_sys_err_reg                               */
/*                      wps_timeout_fn_ap                                    */
/*                      wps_local_mem_alloc                                  */
/*                      wps_local_mem_free                                   */
/*                      wps_pkt_mem_alloc                                    */
/*                      wps_pkt_mem_free                                     */
/*                      wps_create_alarm                                     */
/*                      wps_delete_alarm                                     */
/*                      wps_start_alarm                                      */
/*                      wps_stop_alarm                                       */
/*                      wps_handle_event_ap                                  */
/*                      wps_alloc_mem_hdl_reg                                */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE
#ifdef INT_WPS_SUPP

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "wps_ap.h"
#include "wps_cred_mgmt_ap.h"
#include "iconfig.h"
#include "imem_if.h"
#include "mac_init.h"

/*****************************************************************************/
/* Global Variable Definitions                                               */
/*****************************************************************************/

BOOL_T g_wps_reg_enabled  = BTRUE;
BOOL_T g_wps_allow_config = BTRUE;
BOOL_T g_wps_reg_init_done = BFALSE;
BOOL_T g_wps_implemented_reg = BTRUE;

wps_t        *g_wps_ctxt_ptr   = NULL;
wps_config_t *g_wps_config_ptr = NULL;
wps_eap_tx_t *g_wps_eap_tx_ptr = NULL;
wps_eap_rx_t *g_wps_eap_rx_ptr = NULL;
wps_priv_t   *g_wps_priv_ptr   = NULL;
wps_attr_t   *g_wps_attr_ptr   = NULL;
wps_reg_t    *g_wps_reg_ptr    = NULL;

/*****************************************************************************/
/* Static Global Variable Definitions                                        */
/*****************************************************************************/

static wps_eap_tx_t g_wps_eap_tx = {0};
static wps_eap_rx_t g_wps_eap_rx = {BFALSE, BFALSE, 0};
static wps_t        g_wps_ctxt   = {NULL};
static wps_config_t g_wps_config;
static wps_priv_t   g_wps_priv;
static wps_attr_t   g_wps_attr;
static wps_reg_t    g_wps_reg;

static UWORD16 g_wps_scratch_mem_idx[MAX_NESTED_SCRATCH_MEM_ALLOC] = {0};
static UWORD8  g_wps_cur_scratch_indx = 0;

/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/

static void wps_alloc_mem_hdl_reg(void);

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : sys_init_wps_reg                                      */
/*                                                                           */
/*  Description      : This function does initialization of WPS registrar    */
/*                                                                           */
/*  Inputs           : None                                                  */
/*                                                                           */
/*  Globals          : g_wps_ctxt_ptr                                        */
/*                     g_wps_config_ptr                                      */
/*                                                                           */
/*  Processing       : This function does the following:                     */
/*                     1) Allocate memory for all the strcutures and update  */
/*                        pointers to point to these memories                */
/*                     2) Initialize the handles in Main Context Structure   */
/*                     3) Initialize the members in WPS Configuration        */
/*                        Structure to default values                        */
/*                     4) Check if there is a valid credential in EEPROM, if */
/*                        so configure the AP with this Credential           */
/*                                                                           */
/*  Outputs          : Updated Context and Configuration Structure           */
/*                                                                           */
/*  Returns          : None                                                  */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void sys_init_wps_reg(void)
{
    wps_store_cred_t eeprom_cred = {0};
    
	TROUT_FUNC_ENTER;
    if(BTRUE == g_wps_reg_init_done)
        return;

    /* Allocated memory for all the members in the context strutcure */
    wps_alloc_mem_hdl_reg();

    /* Update the memory handles in the main context structure */
    init_wps_ctxt_hdl();

    /*  Initialize the configuration structure */
    wps_init_config_ap();

    /* Update all the device specific information in Configuration Structure */
    wps_update_device_info_ap();

    /* Initialize the WPS Registrar */
    wps_init_registrar(g_wps_ctxt_ptr);

    /* Set WPS Registrar implemented flag */
    set_wps_implemented_reg(BTRUE);

    /* Indicate WID Configuration is allowed */
    set_wps_allow_config(BTRUE);

    /* Indicate WPS Registrar Initialization done */
    g_wps_reg_init_done = BTRUE;

// 20120709 caisf add, merged ittiam mac v1.2 code
#ifdef MAC_P2P
    if(BTRUE == mget_p2p_enable())
        return;
#endif /* MAC_P2P */

    /* Check if there is a valid Credential in EEPROM, if so then read it and*/
    /* configure the AP with these credential                                */
    if(SUCCESS == wps_get_cred_eeprom_ap(&eeprom_cred))
        wps_apply_cred_ap(&eeprom_cred);
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : sys_start_wps_reg                                     */
/*                                                                           */
/*  Description      : This function handles WPS Start request from the user */
/*                                                                           */
/*  Inputs           : WPS Protocol type (PIN/PBC)                           */
/*                                                                           */
/*  Globals          : g_wps_config_ptr                                      */
/*                     g_wps_ctxt_ptr                                        */
/*                                                                           */
/*  Processing       : This function handles the start request from the user */
/*                     Ths function updates the WPS configuration structure  */
/*                     call appropriate WPS library function to start the    */
/*                     the Registrar. If the start is a SUCCESS then update  */
/*                     g_wps_allow_config to BFALSE indicating that no WID   */
/*                     write request is allowed                              */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : None                                                  */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void sys_start_wps_reg(WPS_PROT_TYPE_T prot_type)
{
	// 20120709 caisf add, merged ittiam mac v1.2 code
    /* Update the requested protocol type in the configuration structure     */
   	/* Note : This is done before the check for registrar enabled as it is   */
   	/* required to be set for P2P just after switching to GO mode            */
   	/* The registrar will be enabled after all the WIDs have been restored   */
    g_wps_config_ptr->prot_type = prot_type;

    /* If WPS Registrar is disabled then return */
    if(BFALSE == get_wps_reg_enabled())
        return;

	// 20120709 caisf masked, merged ittiam mac v1.2 code
    /* Update the requested protocol type in the configuration structure     */
    //g_wps_config_ptr->prot_type = prot_type;

    /* Call the function in the WPS Library to start the Registrar, incase   */
    /* the start is successfull then update the flag to block all the WID    */
    /* write request from the user                                           */
    if(SUCCESS == wps_start_registrar(g_wps_ctxt_ptr))
        set_wps_allow_config(BFALSE);
    else
        set_wps_allow_config(BTRUE);

    return;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : sys_stop_wps_reg                                      */
/*                                                                           */
/*  Description      : This function handle WPS Stop request from the user   */
/*                                                                           */
/*  Inputs           : 1) Reason code to be used in De-Auth frame            */
/*                     2) Enrollee MAC address                               */
/*                     3) Value of selected registrar attribute              */
/*                                                                           */
/*  Globals          : g_wps_config_ptr                                      */
/*                     g_wps_ctxt_ptr                                        */
/*                                                                           */
/*  Processing       : This function does the following processing:          */
/*                     1) Check if the STA is WPS Capable and Currently      */
/*                        involved in Registration Protocol with the         */
/*                        registrar. Send a Deauth to this STA with specified*/
/*                        input reason code and delete the STA entry from the*/
/*                        global STA table                                   */
/*                     2) Call the Library function wps_stop_registrar to    */
/*                        stop the Registrar.                                */
/*                     3) Reset the MAC address member of configuration      */
/*                        structure and reset the flag to allow WID config   */
/*                        writes                                             */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : None                                                  */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void sys_stop_wps_reg(UWORD16 reason_code, UWORD8 *mac_addr, BOOL_T start_req)
{
    asoc_entry_t *ae = (asoc_entry_t *)find_entry(mac_addr);

    /* Check if the STA is WPS Capable and Currently involved in */
    /* Registration Protocol with the registrar                  */
    if((NULL != ae) && (BTRUE == ae->wps_cap_sta) &&
       (BTRUE == mac_addr_cmp(g_wps_config_ptr->mac_addr, mac_addr)))
    {
        /* Send the De-authentication Frame to the station */
        send_deauth_frame(g_wps_config_ptr->mac_addr, reason_code);

        /* Send the sta leaving information to host */
        send_join_leave_info_to_host(ae->asoc_id, mac_addr, BFALSE);

        delete_entry(mac_addr);

        PRINTD("Status0: Successfully Deauthenticated WPS STA %x:%x:%x:%x:%x:%x\n\r",
        mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    }

    /* Call the Library function to Stop the Registrar */
    wps_stop_registrar(g_wps_ctxt_ptr, start_req);

    /* Allow WID Write request by resetting the flag only if selected        */
    /* registrar attribute is being reset                                    */
    if(BFALSE == start_req)
    {
        /* Update required P2P protocol flag */
        update_wps_complete_flag(mac_addr);

        set_wps_allow_config(BTRUE);
        wps_set_ver2_cap_sta(g_wps_ctxt_ptr, BFALSE);
        wps_clear_black_list();
        wsp_clear_fail_enr_list();
    }

    /* Reset the MAC address */
    mem_set(g_wps_config_ptr->mac_addr, 0, MAC_ADDRESS_LEN);

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

	return;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_handle_event_ap                                   */
/*                                                                           */
/*  Description      : This function handle WPS Registrar Events             */
/*                                                                           */
/*  Inputs           : MAC-Controller Event                                  */
/*                                                                           */
/*  Globals          : g_wps_ctxt_ptr                                        */
/*                                                                           */
/*  Processing       : This function checks if the given Event is a          */
/*                     WPS_TIMEOUT_EVENT and calls the library function to   */
/*                     handle this event and return BTRUE. Returns BFALSE if */
/*                     not a WPS event                                       */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : BTRUE  - If the event was processed successfully      */
/*                     BFALSE - Otherwise                                    */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

BOOL_T wps_handle_event_ap(UWORD32 event)
{
    BOOL_T           event_handled = BFALSE;
    misc_event_msg_t *wps_event    = (misc_event_msg_t *)event;

    if(WPS_TIMEOUT_EVENT == wps_event->name)
    {
        wps_handle_event(g_wps_ctxt_ptr, wps_event->info);
        event_handled = BTRUE;
    }

    return event_handled;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_allow_sys_restart_ap                              */
/*                                                                           */
/*  Description      : This function handle WPS Registrar related operations */
/*                     during a system reset                                 */
/*                                                                           */
/*  Inputs           : System Error Code                                     */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function checks if the system error is anything  */
/*                     other than a NO_ERROR then Stop the WPS Registrar and */
/*                     returns BTRUE else BFALSE.                            */
/*                                                                           */
/*  Outputs          : Status of Error Processing                            */
/*                                                                           */
/*  Returns          : BTRUE  - If the Error was processed successfully      */
/*                     BFALSE - Otherwise                                    */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

BOOL_T wps_allow_sys_restart_ap(UWORD16 sys_error)
{
    BOOL_T retval = BFALSE;

    /* If the Error Code is serious then Stop the registrar */
    if(NO_ERROR != (ERROR_CODE_T)sys_error)
    {
        sys_stop_wps_reg(UNSPEC_REASON, g_wps_config_ptr->mac_addr, BFALSE);
        retval = BTRUE;
    }

    return retval;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_handle_sys_err_reg                                */
/*                                                                           */
/*  Description      : This function handle WPS Registrar`s system related   */
/*                     errors                                                */
/*                                                                           */
/*  Inputs           : System Error Code                                     */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function checks if the system error is anything  */
/*                     other than a NO_ERROR then call system software .     */
/*                     function to post a system restart request             */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : None                                                  */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void wps_handle_sys_err_reg(UWORD16 sys_error)
{
    /* If the Error Code is serious then Stop the registrar */
    if(NO_ERROR != (ERROR_CODE_T)sys_error)
    {
        raise_system_error((ERROR_CODE_T)sys_error);
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_timeout_fn_ap                                     */
/*                                                                           */
/*  Description      : This function handle WPS Registrar alarm timeouts     */
/*                                                                           */
/*  Inputs           : Alarm handle and Alarm data                           */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function allocates memory in local memory pool   */
/*                     and posts a MISC WPS_TIMEOUT_EVENT Event.             */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : None                                                  */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

#ifndef OS_LINUX_CSL_TYPE
void wps_timeout_fn_ap(HANDLE_T* alarm, ADDRWORD_T data)
#else /* OS_LINUX_CSL_TYPE */
void wps_timeout_fn_ap(ADDRWORD_T data)
#endif /* OS_LINUX_CSL_TYPE */
{
    misc_event_msg_t *misc = 0;

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
/*  Function Name : wps_local_mem_alloc                                      */
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

void* wps_local_mem_alloc(UWORD16 size)
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
/*  Function Name : wps_local_mem_free                                       */
/*                                                                           */
/*  Description   : This function frees local memory for WPS.                */
/*                                                                           */
/*  Inputs        : 1) Pointer to the buffer to be freed                     */
/*                                                                           */
/*  Globals       : g_local_mem_handle                                       */
/*                                                                           */
/*  Processing    : This function uses memory manager function to free given */
/*                  buffer from shared memory.                               */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None                                                     */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void wps_local_mem_free(void *buffer_addr)
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
/*  Function Name : wps_pkt_mem_alloc                                        */
/*                                                                           */
/*  Description   : This function allocates Packet memory for WPS.           */
/*                                                                           */
/*  Inputs        : 1) Size of buffer to be allocated                        */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function uses memory manager function to allocate a */
/*                  buffer of required size from packet memory.              */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : void *, Pointer to the buffer                            */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void* wps_pkt_mem_alloc(UWORD16 size)
{
    return pkt_mem_alloc(MEM_PRI_HPTX);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : wps_pkt_mem_free                                         */
/*                                                                           */
/*  Description   : This function frees Packet memory for WPS.               */
/*                                                                           */
/*  Inputs        : 1) Pointer to the buffer to be freed                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function uses memory manager function to free given */
/*                  buffer from packet memory.                               */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None                                                     */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void wps_pkt_mem_free(void *buffer_addr)
{
    pkt_mem_free(buffer_addr);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : wps_get_num_users                                        */
/*                                                                           */
/*  Description   : This function returns the number of user for specified   */
/*                  buffer.                                                  */
/*                                                                           */
/*  Inputs        : 1) Pointer to the buffer                                 */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function uses memory manager function to get the    */
/*                  number of users for specified buffer                     */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None                                                     */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 wps_get_num_users(void *buffer)
{
    return get_num_users(buffer);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : wps_add_mem_users                                        */
/*                                                                           */
/*  Description   : This function returns the number of user for specified   */
/*                  buffer.                                                  */
/*                                                                           */
/*  Inputs        : 1) Pointer to the buffer                                 */
/*                  2) Number of users to be added to this buffer            */
/*                                                                           */
/*  Globals       : g_shared_pkt_mem_handle                                  */
/*                                                                           */
/*  Processing    : This function uses memory manager function to set the    */
/*                  number of users for specified buffer                     */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None                                                     */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void wps_add_mem_users(void *buffer, UWORD8 num_user)
{
    mem_add_users(g_shared_pkt_mem_handle, buffer, num_user);
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
/*                                                                           */
/*  Processing       : This function maps the WPS create alarm function to   */
/*                     system create  alarm function. Also it locks the alarm*/
/*                     bufffer                                               */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : Pointer to the alarm handle                           */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void* wps_create_alarm(WPS_ALARM_FUNC_T* func, UWORD32 data, ALARM_FUNC_WORK_T *work_func)
{
    return((void *)create_alarm((ALARM_FUNC_T*) func, (ADDRWORD_T)data, work_func));
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
/*                                                                           */
/*  Processing       : This function maps the WPS delete alarm function to   */
/*                     system delete alarm function.                         */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : None                                                  */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void wps_delete_alarm(void** handle)
{
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
/*                                                                           */
/*  Processing       : This function maps the WPS start alarm function to    */
/*                     system start alarm function.                          */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : BTRUE  - Alarm Start was successful                   */
/*                     BFALSE - Otherwise                                    */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

BOOL_T wps_start_alarm(void* handle, UWORD32 timeout_ms)
{
#ifndef OS_LINUX_CSL_TYPE
    return start_alarm((ALARM_HANDLE_T *)handle, (UWORD32) timeout_ms);
#else /* OS_LINUX_CSL_TYPE */
    start_alarm((ALARM_HANDLE_T *)handle, (UWORD32) timeout_ms);

    return BTRUE;
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
/*                                                                           */
/*  Processing       : This function maps the WPS stop alarm function to     */
/*                     system stop alarm function.                           */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : None                                                  */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void wps_stop_alarm(void* handle)
{
    stop_alarm((ALARM_HANDLE_T *)handle);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_alloc_mem_hdl_reg                                 */
/*                                                                           */
/*  Description      : This function updates the appropriate handles to point*/
/*                     to respective global memory structures and zero       */
/*                     initializes them                                      */
/*                                                                           */
/*  Inputs           : None                                                  */
/*                                                                           */
/*  Globals          : g_wps_ctxt_ptr                                        */
/*                     g_wps_eap_tx_ptr                                      */
/*                     g_wps_eap_rx_ptr                                      */
/*                     g_wps_config_ptr                                      */
/*                     g_wps_priv_ptr                                        */
/*                     g_wps_attr_ptr                                        */
/*                     g_wps_reg_ptr                                         */
/*                                                                           */
/*  Processing       : This function updates the appropriate handles to point*/
/*                     to respective global memory structures and zero       */
/*                     initializes them                                      */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : None                                                  */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

static void wps_alloc_mem_hdl_reg(void)
{
    /* Update the Global memory pointers */
    g_wps_ctxt_ptr   = &g_wps_ctxt;
    g_wps_eap_tx_ptr = &g_wps_eap_tx;
    g_wps_eap_rx_ptr = &g_wps_eap_rx;
    g_wps_config_ptr = &g_wps_config;
    g_wps_priv_ptr   = &g_wps_priv;
    g_wps_attr_ptr   = &g_wps_attr;
    g_wps_reg_ptr    = &g_wps_reg;

    /* Zero initialize all the memory structure */
    mem_set(g_wps_ctxt_ptr, 0, sizeof(wps_t));
    mem_set(g_wps_eap_tx_ptr, 0, sizeof(wps_eap_tx_t));
    mem_set(g_wps_eap_rx_ptr, 0, sizeof(wps_eap_rx_t));
    mem_set(g_wps_config_ptr, 0, sizeof(wps_config_t));
    mem_set(g_wps_priv_ptr, 0, sizeof(wps_priv_t));
    mem_set(g_wps_attr_ptr, 0, sizeof(wps_attr_t));
    mem_set(g_wps_reg_ptr, 0, sizeof(wps_reg_t));
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_get_tsf_timestamp                                 */
/*                                                                           */
/*  Description      : This function gets the MAC-HW TSF timer value         */
/*                                                                           */
/*  Inputs           : Pointer to varaible to store TSF High                 */
/*                     Pointer to varaible to store TSF Low                  */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function users the MAC-HW interface function to  */
/*                     read the Low and High TSF timer value                 */
/*                                                                           */
/*  Outputs          : TSF Timer value                                       */
/*                                                                           */
/*  Returns          : None                                                  */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void wps_get_tsf_timestamp(UWORD32 *tsf_timer_hi, UWORD32 *tsf_timer_lo)
{
    get_machw_tsf_timer(tsf_timer_hi, tsf_timer_lo);
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_handle_disconnect_req_ap                          */
/*                                                                           */
/*  Description      : This function handles STA disconnection of a WPS      */
/*                     Enrollee                                              */
/*                                                                           */
/*  Inputs           : 1) Association Entry of the STA                       */
/*                     2) Mac address of the STA                             */
/*                                                                           */
/*  Globals          : g_wps_config_ptr                                      */
/*                     g_wps_ctxt_ptr                                        */
/*                                                                           */
/*  Processing       : This function stops the WPS Registrar if the STA is   */
/*                     WPS Capable                                           */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : None                                                  */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void wps_handle_disconnect_req_ap(asoc_entry_t *ae, UWORD8 *sta_addr)
{
    UWORD8 *enr_mac_addr = g_wps_config_ptr->mac_addr;

    if((NULL == ae) || (NULL == sta_addr))
        return;

    /* If the STA is WPS capable, if the MAC address of the enrollee with    */
    /* which the WPS registration protocol is in progress matches with that  */
    /* of the STA then stop WPS Registration Protocol with this STA          */
    if((BTRUE == ae->wps_cap_sta) &&
       (BTRUE == mac_addr_cmp(enr_mac_addr, sta_addr)))
    {
        /* Stop the registration protocol with the station  Call the Library */
        /* function to Stop the Registrar                                    */
        wps_stop_registrar(g_wps_ctxt_ptr, g_wps_reg_ptr->start_req);

        /* Allow WID Write request by resetting the flag */
        if(g_wps_reg_ptr->start_req == BFALSE)
            set_wps_allow_config(BTRUE);

        /* Reset the MAC address */
        mem_set(g_wps_config_ptr->mac_addr, 0, MAC_ADDRESS_LEN);
    }
}

#endif /* INT_WPS_SUPP */
#endif /* BSS_ACCESS_POINT_MODE */
