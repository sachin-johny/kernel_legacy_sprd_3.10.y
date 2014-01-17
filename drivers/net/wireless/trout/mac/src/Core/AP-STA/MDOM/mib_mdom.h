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
/*  File Name         : mib_mdom.h                                           */
/*                                                                           */
/*  Description       : This file contains the definitions and structures    */
/*                      related to MIB for Multi-domain operation.           */
/*                      Access functions to get and set the MIB values       */
/*                      are also provided.                                   */
/*                                                                           */
/*  List of Functions : Access functions for all Multidomain related MIB     */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_MULTIDOMAIN
#ifndef MIB_MDOM_H
#define MIB_MDOM_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "mib1.h"
#include "cglobals.h"

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

/* 11d MIB Structure - Common for both AP and STA */
typedef struct
{
    TRUTH_VALUE_T dot11MultiDomainOperationImplemented;
    TRUTH_VALUE_T dot11MultiDomainCapabilityEnabled;
    UWORD8        dot11CountryString[3];
    TRUTH_VALUE_T dot11OperatingClassesImplemented;
    TRUTH_VALUE_T dot11OperatingClassesRequired;
    TRUTH_VALUE_T dot11ExtendedChannelSwitchActivated;
} mib_11d_t;

/*****************************************************************************/
/* External Global Variables                                                 */
/*****************************************************************************/

extern mib_11d_t MIB_11D;

/*****************************************************************************/
/* Extern Function Declaration                                               */
/*****************************************************************************/

extern void initialize_mac_mib_11d(void);

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/* Functions for accessing standard Multi-domain MIB variables */
INLINE void mset_MultiDomainOperationImplemented(TRUTH_VALUE_T val)
{
    MIB_11D.dot11MultiDomainOperationImplemented = val;
}

INLINE TRUTH_VALUE_T mget_MultiDomainOperationImplemented(void)
{
    return MIB_11D.dot11MultiDomainOperationImplemented;
}

INLINE void mset_MultiDomainCapabilityEnabled(TRUTH_VALUE_T val)
{
    MIB_11D.dot11MultiDomainCapabilityEnabled = val;
}

INLINE TRUTH_VALUE_T mget_MultiDomainCapabilityEnabled(void)
{
    return MIB_11D.dot11MultiDomainCapabilityEnabled;
}

INLINE void mset_CountryString(UWORD8* cnt_str)
{
    strcpy(MIB_11D.dot11CountryString, cnt_str);
}

INLINE UWORD8* mget_CountryString(void)
{
    return (UWORD8 *) MIB_11D.dot11CountryString;
}

INLINE void mset_OperatingClassesImplemented(TRUTH_VALUE_T val)
{
    MIB_11D.dot11OperatingClassesImplemented = val;
}

INLINE TRUTH_VALUE_T mget_OperatingClassesImplemented(void)
{
    return MIB_11D.dot11OperatingClassesImplemented;
}

INLINE void mset_OperatingClassesRequired(TRUTH_VALUE_T val)
{
    MIB_11D.dot11OperatingClassesRequired = val;
}

INLINE TRUTH_VALUE_T mget_OperatingClassesRequired(void)
{
    return MIB_11D.dot11OperatingClassesRequired;
}

INLINE void mset_ExtendedChannelSwitchActivated(TRUTH_VALUE_T val)
{
    MIB_11D.dot11ExtendedChannelSwitchActivated = val;
}

INLINE TRUTH_VALUE_T mget_ExtendedChannelSwitchActivated(void)
{
    return MIB_11D.dot11ExtendedChannelSwitchActivated;
}

#endif /* MIB_MDOM_H */
#endif /* MAC_MULTIDOMAIN */
