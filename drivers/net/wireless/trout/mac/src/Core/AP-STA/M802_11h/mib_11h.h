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
/*  File Name         : mib_11h.h                                            */
/*                                                                           */
/*  Description       : This file contains the definitions and structures    */
/*                      related to MIB and private MIB for the 802.11h       */
/*                      standard. Access functions to get and set the MIB    */
/*                      values are also provided.                            */
/*                                                                           */
/*  List of Functions : Access functions for all 802.11h related MIB         */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_802_11H

#ifndef MIB_11H_H
#define MIB_11H_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "mib1.h"

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

/* 11h standard MIB */
typedef struct
{
	/************************************************************************/
	/*dot11SpectrumManagement TABLE-members of dot11SpectrumManagementEntry */
	/************************************************************************/
	/* This table of attributes are used to implement DFS and TPC algorithm */
	UWORD32   dot11SpectrumManagementIndex;
	UWORD32   dot11MitigationRequirement;
	UWORD32   dot11ChannelSwitchTime;
	WORD32    dot11PowerCapabilityMax;
    WORD32    dot11PowerCapabilityMin;
} mib_11h_t;

/* Structure for private MIB */
typedef struct
{
    BOOL_T  enableDFS;   /* Dot11h DFS feature */
    BOOL_T  enableTPC;   /* Dot11h TPC feature */
    UWORD8 NonOccupancyPeriod; /* Non Occupancy Period in secs */
    UWORD8 ValidityPeriod;     /* Validity Period in secs      */
} p_mib_11h_t;

/*****************************************************************************/
/* External Global Variables                                                 */
/*****************************************************************************/

extern mib_11h_t   MIB_11H;
extern p_mib_11h_t P_MIB_11H;

/*****************************************************************************/
/* Extern Function Declaration                                               */
/*****************************************************************************/

extern void initialize_mac_mib_11h(void);
extern void initialize_private_mib_11h(void);
extern void mset_enableDFS(BOOL_T inp);
extern BOOL_T mget_enableDFS(void);
extern void mset_enableTPC(BOOL_T inp);
extern BOOL_T mget_enableTPC(void);

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/


/* This attribute shall specify  power constraint  value */
/* presently Ittiam STA is not using this MIB variable   */
INLINE void mset_MitigationRequirement(UWORD32 inp)
{
	MIB_11H.dot11MitigationRequirement = inp;
}

/* This attribute shall specify  mitigation for power constraint  value */
/* presently Ittiam STA is not using this MIB variable   */
INLINE UWORD32 mget_MitigationRequirement(void)
{
	return(MIB_11H.dot11MitigationRequirement);
}

/* This attribute shall specify  the time that PHY can take  */
/* to switch the channel   */
INLINE void mset_ChannelSwitchTime(UWORD32 inp)
{
	MIB_11H.dot11ChannelSwitchTime = inp;
}

/* Function to get channel switch time  */
INLINE UWORD32 mget_ChannelSwitchTime(void)
{
	return(MIB_11H.dot11ChannelSwitchTime);
}

/* This attribute shall specify Max power that can be supported by RF for  */
/* the current channel. Presently Ittiam-STA using RF supported info from table*/
INLINE void mset_PowerCapabilityMax(WORD32 inp)
{
	MIB_11H.dot11PowerCapabilityMax = inp;
}

/* Function to get maximum power capaility of the STA */
INLINE WORD32 mget_PowerCapabilityMax(void)
{
	return(MIB_11H.dot11PowerCapabilityMax);
}

/* This attribute shall specify Max power that can be supported by RF for  */
/* the current channel. Presently Ittiam-STA using RF supported info from table*/
INLINE void mset_PowerCapabilityMin(WORD32 inp)
{
	MIB_11H.dot11PowerCapabilityMin = inp;
}

/* Function to get minimum power capability of STA */
INLINE WORD32 mget_PowerCapabilityMin(void)
{
	return(MIB_11H.dot11PowerCapabilityMin);
}

/* This function is used to set the Non Occupancy Period in minutes. The     */
/* default value as per ETSI is 30 min                                       */
INLINE void mset_NonOccupancyPeriod(UWORD8 inp)
{
    P_MIB_11H.NonOccupancyPeriod = inp;
}

/* This function is used to get the Non Occupancy Period (in minutes) */
INLINE UWORD8 mget_NonOccupancyPeriod(void)
{
    return (P_MIB_11H.NonOccupancyPeriod);
}

/* This function is used to set the Validity Period in hours.   The default  */
/* value as per ETSI is 24 hrs                                               */
INLINE void mset_ValidityPeriod(UWORD8 inp)
{
    P_MIB_11H.ValidityPeriod = inp;
}

/* This function is used to get the Validity Period (in hours) */
INLINE UWORD8 mget_ValidityPeriod(void)
{
    return (P_MIB_11H.ValidityPeriod);
}

#endif /* MIB_11H_H */

#endif /* MAC_802_11H */
