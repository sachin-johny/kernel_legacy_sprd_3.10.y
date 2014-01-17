/*****************************************************************************/
/*                                                                           */
/*                     Ittiam 802.11 MAC SOFTWARE                            */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2009                               */
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
/*  File Name         : mib_11h.c                                            */
/*                                                                           */
/*  Description       : This file contains the functions related to MIB and  */
/*                      private MIB in 802.11h standard.                     */
/*                                                                           */
/*  List of Functions : initialize_mac_mib_11h                               */
/*                      initialize_private_mib_11h                           */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_802_11H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "mib_11h.h"
#include "ch_info.h"

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

mib_11h_t   MIB_11H   = {0};
p_mib_11h_t P_MIB_11H = {0};

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : initialize_mac_mib_11h                                */
/*                                                                           */
/*  Description      : This function initializes all MIB entries for 802.11h */
/*                                                                           */
/*  Inputs           : None                                                  */
/*                                                                           */
/*  Globals          : MIB_11H                                               */
/*                                                                           */
/*  Processing       : All the MIB entries are initialized to their default  */
/*                     values.                                               */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void initialize_mac_mib_11h(void)
{
    MIB_11H.dot11MitigationRequirement                 = 3;  /* 3dB     */
    MIB_11H.dot11ChannelSwitchTime                     = 2;  /* 2TUs    */
    MIB_11H.dot11PowerCapabilityMax                    = 18; /* 0dBm    */
    MIB_11H.dot11PowerCapabilityMin                    = 0;  /* -100dBm */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : mset_enableDFS                                        */
/*                                                                           */
/*  Description      : This function sets the DFS enable value               */
/*                                                                           */
/*  Inputs           : None                                                  */
/*                                                                           */
/*  Globals          : P_MIB_11H                                             */
/*                                                                           */
/*  Processing       : Sets the DFS MIB value as it is from the input        */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/
void mset_enableDFS(BOOL_T inp)
{
    if(get_current_start_freq() == RC_START_FREQ_5)
    P_MIB_11H.enableDFS   =   inp;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : mget_enableDFS                                        */
/*                                                                           */
/*  Description      : This function returns the DFS enable value            */
/*                                                                           */
/*  Inputs           : None                                                  */
/*                                                                           */
/*  Globals          : P_MIB_11H                                             */
/*                                                                           */
/*  Processing       : Returns the DFS MIB value as it is in case of 5GHz    */
/*                     and returns BFALSE always in case of 2.4 GHz          */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/
BOOL_T mget_enableDFS(void)
{
   if(get_current_start_freq() == RC_START_FREQ_5)
   {
 	   return(P_MIB_11H.enableDFS);
   }
   else
   {
	   return BFALSE;
   }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : mset_enableTPC                                        */
/*                                                                           */
/*  Description      : This function sets the TPC enable value               */
/*                                                                           */
/*  Inputs           : None                                                  */
/*                                                                           */
/*  Globals          : P_MIB_11H                                             */
/*                                                                           */
/*  Processing       : Sets the TPC MIB value as it is from the input        */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/
void mset_enableTPC(BOOL_T inp)
{
    P_MIB_11H.enableTPC   =   inp;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : mget_enableTPC                                        */
/*                                                                           */
/*  Description      : This function returns the TPC enable value            */
/*                                                                           */
/*  Inputs           : None                                                  */
/*                                                                           */
/*  Globals          : P_MIB_11H                                             */
/*                                                                           */
/*  Processing       : Returns the TPC MIB value as it is in case of 5GHz    */
/*                     and returns BFALSE always in case of 2.4 GHz          */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/
BOOL_T mget_enableTPC(void)
{
   if(get_current_start_freq() == RC_START_FREQ_5)
   {
    	return(P_MIB_11H.enableTPC);
   }
   else
   {
	   return BFALSE;
   }
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : initialize_private_mib_11h                            */
/*                                                                           */
/*  Description      : This function initializes all the private MIB entries */
/*                     used by 11h MAC.                                      */
/*                                                                           */
/*  Inputs           : None                                                  */
/*                                                                           */
/*  Globals          : P_MIB_11H                                             */
/*                                                                           */
/*  Processing       : All the private MIB entries are initialized to their  */
/*                     default values.                                       */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void initialize_private_mib_11h(void)
{
    mset_enableDFS(BTRUE);
    mset_enableTPC(BTRUE);

    P_MIB_11H.NonOccupancyPeriod = 30; /* mins */
    P_MIB_11H.ValidityPeriod     = 24; /* hrs */
}

void mib_11h_print(void)
{
	PRINT_MIB("/*************************************************************************/\n");
	PRINT_MIB("/* mib 11h                                                               */\n");
	PRINT_MIB("/*************************************************************************/\n");
	PRINT_MIB("\n");


	PRINT_MIB("/*************************************************************************/\n");
	PRINT_MIB("/* dot11SpectrumManagement TABLE-members of dot11SpectrumManagementEntry */\n");
	PRINT_MIB("/*************************************************************************/\n");
	/* This table of attributes are used to implement DFS and TPC algorithm */
	
	PRINT_MIB("MIB_11H.dot11SpectrumManagementIndex = %u\n",MIB_11H.dot11SpectrumManagementIndex);
	PRINT_MIB("MIB_11H.dot11MitigationRequirement = %u\n",MIB_11H.dot11MitigationRequirement);
	PRINT_MIB("MIB_11H.dot11ChannelSwitchTime = %u\n",MIB_11H.dot11ChannelSwitchTime);
	
	PRINT_MIB("MIB_11H.dot11PowerCapabilityMax = %d\n",MIB_11H.dot11PowerCapabilityMax);
    PRINT_MIB("MIB_11H.dot11PowerCapabilityMin = %d\n",MIB_11H.dot11PowerCapabilityMin);


	/* Structure for private MIB */
	PRINT_MIB("/*************************************************************************/\n");
	PRINT_MIB("/* 11h private MIB                                                       */\n");
	PRINT_MIB("/*************************************************************************/\n");
	PRINT_MIB("P_MIB_11H.enableDFS  = %s\n",
		(P_MIB_11H.enableDFS == BTRUE) ? "BTRUE" : "BFALSE");/* Dot11h DFS feature */

	PRINT_MIB("P_MIB_11H.enableTPC  = %s\n",
		(P_MIB_11H.enableTPC == BTRUE) ? "BTRUE" : "BFALSE");/* Dot11h TPC feature */
 
    PRINT_MIB("P_MIB_11H.NonOccupancyPeriod = %u\n",P_MIB_11H.NonOccupancyPeriod);/* Non Occupancy Period in secs */
    PRINT_MIB("P_MIB_11H.ValidityPeriod     = %u\n",P_MIB_11H.ValidityPeriod);/* Validity Period in secs      */

}
#endif /* MAC_802_11H */
