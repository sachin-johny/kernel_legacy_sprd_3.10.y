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
/*  File Name         : wps_eap_rx_reg.h                                     */
/*                                                                           */
/*  Description       : This file contains all the data type definitions for */
/*                      WPS Registration Protocol EAP RX  of the Registrar   */
/*                                                                           */
/*  List of Functions : None                                                 */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*  Revision History  :                                                      */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         22 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
#ifdef INT_WPS_REG_SUPP

#ifndef WPS_EAP_RX_REG_H
#define WPS_EAP_RX_REG_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "wps_prot.h"

/*****************************************************************************/
/* Constant Macros                                                           */
/*****************************************************************************/
#define MAX_EAP_MSG_RETRY_LIMIT          2
#define EAP_RESPONSE_ID_OFFSET          (EAP_TYPE_OFFSET + 1)

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/
extern STATUS_T wps_handle_eapol_start_rx(wps_t *wps_ctxt_hdl);
extern STATUS_T wps_handle_eapol_pkt_rx(wps_t *wps_ctxt_hdl, UWORD8 *msa,
                                        UWORD16 pkt_len);

/*****************************************************************************/
/* Inline functions                                                          */
/*****************************************************************************/

/*****************************************************************************/
/* This function checks if there are any common WPS Config Methods between   */
/* Enrollee and Registrar and return BTRUE if there are common methods else  */
/* returns BFALSE                                                            */
/*****************************************************************************/
INLINE BOOL_T is_common_config_meth(UWORD16 enr_conf_meth,
                                    UWORD16 reg_conf_meth)
{
    BOOL_T retval = BFALSE;

    if(enr_conf_meth & reg_conf_meth)
        retval = BTRUE;

    return retval;
}

/*****************************************************************************/
/* This function checks if there are any common WPS Connection Type between  */
/* Enrollee and Registrar and return BTRUE if there are common type    else  */
/* returns BFALSE                                                            */
/*****************************************************************************/
INLINE BOOL_T is_common_conn_type(UWORD8 enr_conn_type,
                                  UWORD8 reg_conn_type)
{
    BOOL_T retval = BFALSE;

    if(enr_conn_type & reg_conn_type)
        retval = BTRUE;

    return retval;
}

/*****************************************************************************/
/* This function checks if there are any common Authentication Type between  */
/* Enrollee and Registrar and return BTRUE if there are common type else     */
/* returns BFALSE                                                            */
/*****************************************************************************/
INLINE BOOL_T is_common_auth_type(UWORD16 enr_auth_type,
                                  UWORD16 reg_auth_type)
{
    BOOL_T retval = BFALSE;

    if(enr_auth_type & reg_auth_type)
        retval = BTRUE;

    return retval;
}

/*****************************************************************************/
/* This function checks if there are any common Encryption  Type   between   */
/* Enrollee and Registrar and return BTRUE if there are common type else     */
/* returns BFALSE                                                            */
/*****************************************************************************/
INLINE BOOL_T is_common_encr_type(UWORD16 enr_encr_type,
                                  UWORD16 reg_encr_type)
{
    BOOL_T retval = BFALSE;

    if(enr_encr_type & reg_encr_type)
        retval = BTRUE;

    return retval;
}

/*****************************************************************************/
/* This function is called to check if the UUID received in M1 message       */
/* matches with the UUID-E received in probe request. Returns BTRUE on       */
/* successfull match and return BFALSE on mismatch or error conditions       */
/*****************************************************************************/
INLINE BOOL_T wps_check_pbc_uuid(wps_mon_pbc_t *pbc_enr_info, UWORD8 pbc_enr_cnt,
                                 UWORD8 *uuid_e)
{
    if(1 != pbc_enr_cnt)
    {
        send_wps_status(WPS_REG_MONITOR_EXC1, NULL, 0);
        return BFALSE;
    }

    if(0 != memcmp(uuid_e, pbc_enr_info[0].uuid_e, WPS_UUID_LEN))
        return BFALSE;

    return BTRUE;
}

// 20120709 caisf add, merged ittiam mac v1.2 code
#ifdef MAC_P2P    
/* This function checks if the Enrollee PID received in M1 message is valid */
INLINE BOOL_T check_wps_pid_reg(WPS_PROT_TYPE_T reg_prot_type, UWORD16 enr_pid)
{
    BOOL_T ret_val = BTRUE;
    
    switch(reg_prot_type)
    {
        /*********************************************************************/
        /* If Protocol type is PIN then check whether device password ID are */
        /* valid                                                             */
        /*********************************************************************/
        case PIN:
            switch(enr_pid)
            {
                case PASS_ID_DEFAULT:
                case PASS_ID_USER_SPECIFIED:
                case PASS_ID_MACH_SPECIFIED:
                case PASS_ID_REKEY:
                case PASS_ID_REG_SPECIFIED:
                    break;
                default:
                    ret_val = BFALSE;
            }
        break;
        case PBC:
            /* Check if the device password ID is set to PBC */
            if(PASS_ID_PUSHBUTTON != enr_pid)
                ret_val = BFALSE;
        break;
        default:
            ret_val = BFALSE;
    }
    
    return ret_val;
}
#endif /* MAC_P2P */
#endif /* WPS_EAP_RX_REG_H */
#endif /* INT_WPS_REG_SUPP */
