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
/*  File Name         : sme_ap.c                                             */
/*                                                                           */
/*  Description       : This file contains the SME functions specific to     */
/*                      AP mode.                                             */
/*                                                                           */
/*  List of Functions : send_mlme_rsp_to_host_ap                             */
/*                      handle_start_rsp                                     */
/*                      prepare_mlme_start_req                               */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "management_ap.h"
#include "sme_ap.h"
#include "iconfig.h"

/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/

static void handle_start_rsp(mac_struct_t *mac, UWORD8 *msg);

/*****************************************************************************/
/* Static Global Variables                                                   */
/*****************************************************************************/

/* The various types of MLME response types are handled in different ways. A */
/* function pointer array is maintained to map the different functions to    */
/* the different MLME response types.                                        */

static void (*g_handle_mlme_rsp_func_ap[MAX_MLME_RSP_TYPE])
            (mac_struct_t *mac, UWORD8 *message) =
{
    handle_start_rsp
};

/*****************************************************************************/
/*                                                                           */
/*  Function Name : send_mlme_rsp_to_host_ap                                 */
/*                                                                           */
/*  Description   : This function handles the request to send a MLME         */
/*                  response to the host based on the SME mode in use in the */
/*                  AP mode.                                                 */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC structure                          */
/*                  2) Type of MLME Response                                 */
/*                  3) Pointer to the MLME response message                  */
/*                                                                           */
/*  Globals       : g_handle_mlme_rsp_func_ap                                */
/*                                                                           */
/*  Processing    : In case test or auto SME modes are in use, the MLME      */
/*                  response is not sent to the host. Instead the response   */
/*                  is parsed and the next action appropriately initiated.   */
/*                  In case of manual SME mode (currently not supported) the */
/*                  response is sent to the host in appropraite format.      */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void send_mlme_rsp_to_host_ap(mac_struct_t *mac, UWORD8 type, UWORD8 *message)
{
	TROUT_FUNC_ENTER;
    g_handle_mlme_rsp_func_ap[type](mac, message);
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : handle_start_rsp                                         */
/*                                                                           */
/*  Description   : This function processes the MLME start response for auto */
/*                  and test SME modes.                                      */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC structure                          */
/*                  2) Pointer to the MLME response message                  */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The MLME Start Response is parsed. In case the status is */
/*                  successful no further action is taken. In case of a      */
/*                  failure the start request is sent again.                 */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void handle_start_rsp(mac_struct_t *mac, UWORD8 *msg)
{
    start_rsp_t *start_rsp = (start_rsp_t *)msg;

	TROUT_FUNC_ENTER;
    if(start_rsp->result_code != SUCCESS_MLMESTATUS)
        initiate_start(mac);
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : prepare_mlme_start_req                                   */
/*                                                                           */
/*  Description   : This function prepares the MLME start request.           */
/*                                                                           */
/*  Inputs        : 1) Pointer to the start request buffer                   */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The given start request buffer is updated with           */
/*                  parameters from MIB.                                     */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void prepare_mlme_start_req(start_req_t *start_req)
{
    start_req->start_timeout = get_join_start_timeout();
}

#endif /* BSS_ACCESS_POINT_MODE */
