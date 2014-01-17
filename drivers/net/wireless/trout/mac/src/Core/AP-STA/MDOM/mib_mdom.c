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
/*  File Name         : mib_mdom.c                                            */
/*                                                                           */
/*  Description       : This file contains the 11d MIB structure and         */
/*                      initialization                                       */
/*                                                                           */
/*  List of Functions : initialize_mac_mib_11d                               */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_MULTIDOMAIN

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "mib_mdom.h"
#include "itypes.h"

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/
mib_11d_t   MIB_11D   = {0};

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : initialize_mac_mib_mdom                               */
/*                                                                           */
/*  Description      : This function initializes all MIB entries for         */
/*                     Multi-domain operation.                               */
/*                                                                           */
/*  Inputs           : None                                                  */
/*                                                                           */
/*  Globals          : MIB_11D                                               */
/*                                                                           */
/*  Processing       : All the MIB entries are initialized to their default  */
/*                     values.                                               */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void initialize_mac_mib_11d(void)
{
    MIB_11D.dot11MultiDomainOperationImplemented   = TV_TRUE;
    MIB_11D.dot11MultiDomainCapabilityEnabled      = TV_TRUE;
    MIB_11D.dot11CountryString[0]                  = 'X';
    MIB_11D.dot11CountryString[1]                  = 'X';
    MIB_11D.dot11CountryString[2]                  = 'X';
    MIB_11D.dot11OperatingClassesImplemented       = TV_TRUE;
    MIB_11D.dot11OperatingClassesRequired          = TV_FALSE;
    MIB_11D.dot11ExtendedChannelSwitchActivated    = TV_FALSE;
}

#endif /* MAC_MULTIDOMAIN */
