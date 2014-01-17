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
/*  File Name         : test_config.c                                        */
/*                                                                           */
/*  Description       : This file contains all the functions for test mode   */
/*                      of SME.                                              */
/*                                                                           */
/*  List of Functions : set_test_wid                                         */
/*                      initialize_test_config                               */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef DEFAULT_SME

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "iconfig.h"
#include "host_if.h"
#include "maccontroller.h"
#include "test_config.h"

#ifdef DEBUG_MODE
#include "host_if_test.h"
#endif /* DEBUG_MODE */

/*****************************************************************************/
/* Static Global Variables                                                   */
/*****************************************************************************/

test_wid_struct_t g_test_wid[NUM_TEST_WIDS] = {{0,},};

/*****************************************************************************/
/*                                                                           */
/*  Function Name : initialize_test_config                                   */
/*                                                                           */
/*  Description   : This function initializes the test configuration. Note   */
/*                  that this function needs to be modified by the user to   */
/*                  configure MAC with the desired settings using SME in the */
/*                  test mode.                                               */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_test_wid                                               */
/*                                                                           */
/*  Processing    : This function initializes the global test configuration  */
/*                  structure.                                               */
/*                                                                           */
/*  Outputs       : The global configuration structure is initialized.       */
/*                                                                           */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void initialize_test_config(void)
{
    /*  BSS Type                                                             */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  Infrastructure    Independent   Access Point        */
    /*  Values to set :         0               1            2               */
    /*  --------------------------------------------------------------       */
	TROUT_FUNC_ENTER;
	
    g_test_wid[0].id    = WID_BSS_TYPE;
    g_test_wid[0].type  = WID_CHAR;

#ifdef BSS_ACCESS_POINT_MODE
    g_test_wid[0].value.c_val = 2;
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef IBSS_BSS_STATION_MODE
    g_test_wid[0].value.c_val = 0;
#endif /* IBSS_BSS_STATION_MODE */

    /*  11g operating mode (ignored if 11g not present)                      */
    /*  --------------------------------------------------------------       */
    /*  Configuration :   HighPerf  Compat(RSet #1) Compat(RSet #2)          */
    /*  Values to set :          1               2               3           */
    /*  --------------------------------------------------------------       */
    g_test_wid[1].id    = WID_11G_OPERATING_MODE;
    g_test_wid[1].type  = WID_CHAR;
    g_test_wid[1].value.c_val = 3;

    /*  Transmit Rate                                                        */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  1  2  5.5  11  6  9  12  18  24  36  48  54         */
    /*  Values to set :  1  2  5.5  11  6  9  12  18  24  36  48  54         */
    /*  --------------------------------------------------------------       */
    g_test_wid[2].id    = WID_CURRENT_TX_RATE;
    g_test_wid[2].type  = WID_CHAR;
#ifdef AUTORATE_FEATURE
    g_test_wid[2].value.c_val = 0;
#else /* AUTORATE_FEATURE */
    g_test_wid[2].value.c_val = 54;
#endif /* AUTORATE_FEATURE */

    /*  Channel                                                              */
    /*  -------------------------------------------------------------------  */
    /*  Configuration    :  Auto  Freq(2.4/5)  SecOff(SCN/SCA/SCB)  ChnNum   */
    /*  Value to set     :  0     BIT15(0/1)   BIT9-8(0/1/3)        BIT7-0   */
    /*  -------------------------------------------------------------------- */
    g_test_wid[3].id    = WID_USER_PREF_CHANNEL;
    g_test_wid[3].type  = WID_SHORT;
   	g_test_wid[3].value.s_val = 1;

    /*  Preamble                                                             */
    /*  --------------------------------------------------------------       */
    /*  Configuration :    Auto     Long Only                                */
    /*  Values to set :       0         1                                    */
    /*  --------------------------------------------------------------       */
    g_test_wid[4].id    = WID_PREAMBLE;
    g_test_wid[4].type  = WID_CHAR;
    g_test_wid[4].value.c_val = 0;

    /*  Protection mode for MAC                                              */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  Auto  No protection  ERP    HT    GF                */
    /*  Values to set :  0     1              2      3     4                 */
    /*  --------------------------------------------------------------       */
    g_test_wid[5].id    = WID_11N_PROT_MECH;
    g_test_wid[5].type  = WID_CHAR;
    g_test_wid[5].value.c_val = 0;

    /*  Scan type                                                            */
    /*  --------------------------------------------------------------       */
    /*  Configuration :   Passive Scanning   Active Scanning                 */
    /*  Values to set :                  0                 1                 */
    /*  --------------------------------------------------------------       */
    g_test_wid[6].id    = WID_SCAN_TYPE;
    g_test_wid[6].type  = WID_CHAR;
    g_test_wid[6].value.c_val = 1;

    /*  Site Survey Type                                                     */
    /*  --------------------------------------------------------------       */
    /*  Configuration       :  Values to set                                 */
    /*  Survey 1 Channel    :  0                                             */
    /*  survey all Channels :  1                                             */
    /*  Disable Site Survey :  2                                             */
    /*  --------------------------------------------------------------       */
    g_test_wid[7].id    = WID_SITE_SURVEY;
    g_test_wid[7].type  = WID_CHAR;
#ifdef BSS_ACCESS_POINT_MODE
    g_test_wid[7].value.c_val = 0; /* Invalid configuration for AP mode */
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef IBSS_BSS_STATION_MODE
    g_test_wid[7].value.c_val = SITE_SURVEY_OFF;
#endif /* IBSS_BSS_STATION_MODE */

    /*  RTS Threshold                                                        */
    /*  --------------------------------------------------------------       */
    /*  Configuration :   Any value between 256 to 2347                      */
    /*  Values to set :   Same value. Default is 2347                        */
    /*  --------------------------------------------------------------       */
    g_test_wid[8].id    = WID_RTS_THRESHOLD;
    g_test_wid[8].type  = WID_SHORT;
    g_test_wid[8].value.s_val = 3000;//dumy fix 3000 //2347;

    /*  Fragmentation Threshold                                              */
    /*  --------------------------------------------------------------       */
    /*  Configuration :   Any value between 256 to 2346                      */
    /*  Values to set :   Same value. Default is 2346                        */
    /*  --------------------------------------------------------------       */
    g_test_wid[9].id    = WID_FRAG_THRESHOLD;
    g_test_wid[9].type  = WID_SHORT;
    g_test_wid[9].value.s_val = 2346;

    /*  SSID                                                                 */
    /*  --------------------------------------------------------------       */
    /*  Configuration :   String with length less than 32 bytes              */
    /*  Values to set :   Any string with length less than 32 bytes          */
    /*                    ( In BSS Station Set SSID to "" (null string)      */
    /*                      to enable Broadcast SSID suppport )              */
    /*  --------------------------------------------------------------       */
    g_test_wid[10].id    = WID_SSID;
    g_test_wid[10].type  = WID_STR;
    mem_set(g_test_wid[10].value.str_val, 0x00, MAX_STRING_VAL_LEN);
#ifdef IBSS_BSS_STATION_MODE
    memcpy(g_test_wid[10].value.str_val, "TK", MAX_SSID_VAL_LEN);
#else
    memcpy(g_test_wid[10].value.str_val, "T-AP", MAX_SSID_VAL_LEN);
#endif

    /*  Broadcast SSID Option: Setting this will adhere to "" SSID element   */
    /*  ------------------------------------------------------------------   */
    /*  Configuration :   Enable             Disable                         */
    /*  Values to set :   1                  0                               */
    /*  ------------------------------------------------------------------   */
    g_test_wid[11].id    = WID_BCAST_SSID;
    g_test_wid[11].type  = WID_CHAR;
    g_test_wid[11].value.c_val = 0;

    /*  QoS Enable                                                           */
    /*  --------------------------------------------------------------       */
    /*  Configuration :   QoS Disable   WMM Enable                           */
    /*  Values to set :   0             1                                    */
    /*  --------------------------------------------------------------       */
    g_test_wid[12].id    = WID_QOS_ENABLE;
    g_test_wid[12].type  = WID_CHAR;
    g_test_wid[12].value.c_val = 1;

    /*  Power Management                                                     */
    /*  ------------------------------------------------------------------   */
    /*  Configuration :   NO_POWERSAVE   MIN_POWERSAVE   MAX_POWERSAVE       */
    /*  Values to set :   0              1               2                   */
    /*  ------------------------------------------------------------------   */
    g_test_wid[13].id    = WID_POWER_MANAGEMENT;
    g_test_wid[13].type  = WID_CHAR;
    g_test_wid[13].value.c_val = 0;


    /*  WEP/802 11I Configuration                                            */
    /*  ------------------------------------------------------------------   */
    /*  Configuration : Disable WP40 WP104 WPA-AES WPA-TKIP RSN-AES RSN-TKIP */
    /*  Values (0x)   :   00     03    07     29      49       31      51    */
    /*                                                                       */
    /*  Configuration : WPA-AES+TKIP RSN-AES+TKIP TSN/WPA-WEP40+TKIP         */
    /*  Values (0x)   :      69           71             CB                  */
    /*                                                                       */
    /*  Configuration : TSN/WPA-WEP40+AES+TKIP   TSN/RSN-WEP40+TKIP          */
    /*  Values (0x)   :            EB                    D3                  */
    /*                                                                       */
    /*  Configuration : TSN/RSN-WEP40+AES+TKIP   TSN/WPA-WEP104+TKIP         */
    /*  Values (0x)   :            F3                     CF                 */
    /*                                                                       */
    /*  Configuration : TSN/WPA-WEP104+AES+TKIP  TSN/RSN-WEP104+TKIP         */
    /*  Values (0x)   :            EF                     D7                 */
    /*                                                                       */
    /*  Configuration : TSN/RSN-WEP104+AES+TKIP   Mixed Mode-TKIP            */
    /*  Values (0x)   :            F7                    59                  */
    /*                                                                       */
    /*  Configuration : Mixed mode-AES+TKIP  TSN/Mixed mode-WEP40+TKIP       */
    /*  Values (0x)   :         79                       DB                  */
    /*                                                                       */
    /*  Configuration : TSN/Mixed mode-WEP40+AES+TKIP                        */
    /*  Values (0x)   :              FB                                      */
    /*                                                                       */
    /*  Configuration : TSN/Mixed mode-WEP104+TKIP                           */
    /*  Values (0x)   :              DF                                      */
    /*                                                                       */
    /*  Configuration : TSN/Mixed mode-WEP104+AES+TKIP                       */
    /*  Values (0x)   :              FF                                      */
    /*  ------------------------------------------------------------------   */
    g_test_wid[14].id    = WID_11I_MODE;
    g_test_wid[14].type  = WID_CHAR;
    g_test_wid[14].value.c_val = 0x00;

    /*  WEP Configuration: Used in BSS STA mode only when WEP is enabled     */
    /*  ------------------------------------------------------------------   */
    /*  Configuration : Open System  Shared Key  Any Type  |   802.1x Auth   */
    /*  Values (0x)   :    01             02         03    |      BIT2       */
    /*  ------------------------------------------------------------------   */
    g_test_wid[15].id    = WID_AUTH_TYPE;
    g_test_wid[15].type  = WID_CHAR;
    g_test_wid[15].value.c_val = 0x01;

    /*  WEP/802 11I Configuration                                            */
    /*  ------------------------------------------------------------------   */
    /*  Configuration : WEP Key                                              */
    /*  Values (0x)   : 5 byte for WEP40 and 13 bytes for WEP104             */
    /*                  In case more than 5 bytes are passed on for WEP 40   */
    /*                  only first 5 bytes will be used as the key           */
    /*  ------------------------------------------------------------------   */
    g_test_wid[16].id    = WID_WEP_KEY_VALUE;
    g_test_wid[16].type  = WID_STR;
    {
        UWORD8 wep_key[27] = "000102030405060708090a0b0c";
        mem_set(g_test_wid[16].value.str_val, 0x00,    MAX_STRING_VAL_LEN);
        memcpy(g_test_wid[16].value.str_val, wep_key, 26);
    }

    /*  WEP/802 11I Configuration                                            */
    /*  ------------------------------------------------------------------   */
    /*  Configuration : AES/TKIP WPA/RSNA Pre-Shared Key                     */
    /*  Values to set : Any string with length greater than equal to 8 bytes */
    /*                  and less than 64 bytes                               */
    /*  ------------------------------------------------------------------   */
    g_test_wid[17].id    = WID_11I_PSK;
    g_test_wid[17].type  = WID_STR;
    mem_set(g_test_wid[17].value.str_val, 0x00, MAX_STRING_VAL_LEN);
    memcpy(g_test_wid[17].value.str_val, "88880000", MAX_PSK_LEN);

    /*  IEEE802.1X Key Configuration                                         */
    /*  ------------------------------------------------------------------   */
    /*  Configuration : Radius Server Access Secret Key                      */
    /*  Values to set : Any string with length greater than equal to 8 bytes */
    /*                  and less than 65 bytes                               */
    /*  ------------------------------------------------------------------   */
    g_test_wid[18].id    = WID_1X_KEY;
    g_test_wid[18].type  = WID_STR;
    mem_set(g_test_wid[18].value.str_val, 0x00, MAX_STRING_VAL_LEN);
    memcpy(g_test_wid[18].value.str_val, "password", RAD_KEY_MAX_LEN);

    /*   IEEE802.1X Server Address Configuration                             */
    /*  ------------------------------------------------------------------   */
    /*  Configuration : Radius Server IP Address                             */
    /*  Values to set : Any valid IP Address                                 */
    /*  ------------------------------------------------------------------   */
    g_test_wid[19].id    = WID_1X_SERV_ADDR;
    g_test_wid[19].type  = WID_INT;
    {
        UWORD8 ip[4] = {192,168,20,112};
        memcpy((UWORD8 *)&g_test_wid[19].value.i_val, ip, 4);
    }

    /*  Listen Interval                                                      */
    /*  --------------------------------------------------------------       */
    /*  Configuration :   Any value between 1 to 255                         */
    /*  Values to set :   Same value. Default is 3                           */
    /*  --------------------------------------------------------------       */
    g_test_wid[20].id    = WID_LISTEN_INTERVAL;
    g_test_wid[20].type  = WID_CHAR;
    g_test_wid[20].value.c_val = 3;

    /*  DTIM Period                                                          */
    /*  --------------------------------------------------------------       */
    /*  Configuration :   Any value between 1 to 255                         */
    /*  Values to set :   Same value. Default is 3                           */
    /*  --------------------------------------------------------------       */
    g_test_wid[21].id    = WID_DTIM_PERIOD;
    g_test_wid[21].type  = WID_CHAR;
	// 20120709 caisf mod, merged ittiam mac v1.2 code
#ifdef MAC_P2P
    g_test_wid[21].value.c_val = 1;
#else /* MAC_P2P */
    g_test_wid[21].value.c_val = 3;
#endif /* MAC_P2P */

    /*  ACK Policy                                                           */
    /*  --------------------------------------------------------------       */
    /*  Configuration :   Normal Ack            No Ack                       */
    /*  Values to set :       0                   1                          */
    /*  --------------------------------------------------------------       */
    g_test_wid[22].id    = WID_ACK_POLICY;
    g_test_wid[22].type  = WID_CHAR;
    g_test_wid[22].value.c_val = 0;

    /*  Read address from the SDRAM                                          */
    /*  -------------------------------------------------------------------- */
    /*  Configuration : Take default values        Read Address from SDRAM   */
    /*  Values to set :       0                   1                          */
    /*  -------------------------------------------------------------------- */
    g_test_wid[23].id    = WID_READ_ADDR_SDRAM;
    g_test_wid[23].type  = WID_CHAR;
    g_test_wid[23].value.c_val = 0;

    /*  Enable User Control of TX Power                                      */
    /*  -------------------------------------------------------------------- */
    /*  Configuration : Disable                  Enable                      */
    /*  Values to set :    0                       1                         */
    /*  -------------------------------------------------------------------- */
    g_test_wid[24].id    = WID_USER_CONTROL_ON_TX_POWER;
    g_test_wid[24].type  = WID_CHAR;
    g_test_wid[24].value.c_val = 0;

    /*  11a Tx Power Level                                                   */
    /*  -------------------------------------------------------------------- */
    /*  Configuration : Sets TX Power (Higher the value greater the power)   */
    /*  Values to set : Any value between 0 and 63 (inclusive; Default is 48)*/
    /*  -------------------------------------------------------------------- */
    g_test_wid[25].id    = WID_TX_POWER_LEVEL_11A;
    g_test_wid[25].type  = WID_CHAR;
    g_test_wid[25].value.c_val = DEFAULT_TX_POWER;

    /*  11b Tx Power Level                                                   */
    /*  -------------------------------------------------------------------- */
    /*  Configuration : Sets TX Power (Higher the value greater the power)   */
    /*  Values to set : Any value between 0 and 63 (inclusive; Default is 48)*/
    /*  -------------------------------------------------------------------- */
    g_test_wid[26].id    = WID_TX_POWER_LEVEL_11B;
    g_test_wid[26].type  = WID_CHAR;
    g_test_wid[26].value.c_val = DEFAULT_TX_POWER - 20;

    /*  Beacon Interval                                                      */
    /*  -------------------------------------------------------------------- */
    /*  Configuration : Sets the beacon interval value                       */
    /*  Values to set : Any 16-bit value                                     */
    /*  -------------------------------------------------------------------- */
    g_test_wid[27].id    = WID_BEACON_INTERVAL;
    g_test_wid[27].type  = WID_SHORT;
    g_test_wid[27].value.s_val = 100;

    /*   Ittiam Stack IP Address Configuration                               */
    /*  ------------------------------------------------------------------   */
    /*  Configuration : Ittiam Board Stack IP Address                        */
    /*  Values to set : Any valid IP Address                                 */
    /*  ------------------------------------------------------------------   */
    g_test_wid[28].id    = WID_STACK_IP_ADDR;
    g_test_wid[28].type  = WID_INT;
    {
        UWORD8 ip[4] = {192,168,20,221};
        memcpy(((UWORD8 *)&g_test_wid[28].value.i_val), ip, 4);
    }

    /*  Group Key Update Policy Selection                                    */
    /*  -------------------------------------------------------------------- */
    /*  Configuration : Disabled  timeBased  packetBased   timePacketBased   */
    /*  Values to set :   1            2          3               4          */
    /*  -------------------------------------------------------------------- */
    g_test_wid[29].id    = WID_REKEY_POLICY;
    g_test_wid[29].type  = WID_CHAR;
    g_test_wid[29].value.c_val = 1;

    /*  Rekey Time (s) (Used only when the Rekey policy is 2 or 4)           */
    /*  -------------------------------------------------------------------- */
    /*  Configuration : Sets the Rekey Time (s)                              */
    /*  Values to set : 32-bit value                                         */
    /*  -------------------------------------------------------------------- */
    g_test_wid[30].id    = WID_REKEY_PERIOD;
    g_test_wid[30].type  = WID_INT;
    g_test_wid[30].value.i_val = 84600;

    /*  Rekey Packet Count (in 1000s; used when Rekey Policy is 3)           */
    /*  -------------------------------------------------------------------- */
    /*  Configuration : Sets Rekey Group Packet count                        */
    /*  Values to set : 32-bit Value                                         */
    /*  -------------------------------------------------------------------- */
    g_test_wid[31].id    = WID_REKEY_PACKET_COUNT;
    g_test_wid[31].type  = WID_INT;
    g_test_wid[31].value.i_val = 500;

    /*   Ittiam Stack Netmask Address Configuration                          */
    /*  ------------------------------------------------------------------   */
    /*  Configuration : Ittiam Board Stack NetMask                           */
    /*  Values to set : Any valid Subnet Mask                                */
    /*  ------------------------------------------------------------------   */
    g_test_wid[32].id    = WID_STACK_NETMASK_ADDR;
    g_test_wid[32].type  = WID_INT;
    {
        UWORD8 ip[4] = {255,255,255,0};
        memcpy(((UWORD8 *)&g_test_wid[32].value.i_val), ip, 4);
    }

    /*  Allow Short Slot                                                     */
    /*  --------------------------------------------------------------       */
    /*  Configuration : Disallow Short Slot      Allow Short Slot            */
    /*              (Enable Only Long Slot) (Enable Short Slot if applicable)*/
    /*  Values to set :    0         1                                       */
    /*  --------------------------------------------------------------       */
    g_test_wid[33].id          = WID_SHORT_SLOT_ALLOWED;
    g_test_wid[33].type        = WID_CHAR;
    g_test_wid[33].value.c_val = 0x01;

    /*  IEEE802.1X Supplicant Username                                       */
    /*  ------------------------------------------------------------------   */
    /*  Configuration : 802.1x Supplicant Username                           */
    /*  Values to set : Any string with length less than equal to 20 bytes   */
    /*  ------------------------------------------------------------------   */
    g_test_wid[34].id    = WID_SUPP_USERNAME;
    g_test_wid[34].type  = WID_STR;
    mem_set(g_test_wid[34].value.str_val, 0x00, MAX_STRING_VAL_LEN);
    memcpy(g_test_wid[34].value.str_val, "test", MAX_SUPP_USERNAME_LEN);

    /*  IEEE802.1X Supplicant Password                                       */
    /*  ------------------------------------------------------------------   */
    /*  Configuration : 802.1x Supplicant Password                           */
    /*  Values to set : Any string with length less than equal to 64 bytes   */
    /*  ------------------------------------------------------------------   */
    g_test_wid[35].id    = WID_SUPP_PASSWORD;
    g_test_wid[35].type  = WID_STR;
    mem_set(g_test_wid[35].value.str_val, 0x00, MAX_STRING_VAL_LEN);
    memcpy(g_test_wid[35].value.str_val, "test", MAX_SUPP_PASSWORD_LEN);

    /*  ERP Protection type for MAC                                          */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  Self-CTS   RTS-CTS                                  */
    /*  Values to set :  0          1                                        */
    /*  --------------------------------------------------------------       */
    g_test_wid[36].id    = WID_11N_ERP_PROT_TYPE;
    g_test_wid[36].type  = WID_CHAR;
    g_test_wid[36].value.c_val = 0;

    /*  HT Option Enable                                                     */
    /*  --------------------------------------------------------------       */
    /*  Configuration :   HT Enable          HT Disable                       */
    /*  Values to set :   1                  0                               */
    /*  --------------------------------------------------------------       */
    g_test_wid[37].id    = WID_11N_ENABLE;
    g_test_wid[37].type  = WID_CHAR;
    g_test_wid[37].value.c_val = 1;

    /*  11n Operating Type (Note that 11g operating mode will also be        */
    /*  used in addition to this, if this is set to HT Mixed Type)           */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  HT Mixed  HT Only     HT 2040 Only                  */
    /*  Values to set :     0         1               2                      */
    /*  --------------------------------------------------------------       */
    g_test_wid[38].id    = WID_11N_OPERATING_TYPE;
    g_test_wid[38].type  = WID_CHAR;
    g_test_wid[38].value.c_val = 0;

    /*  11n OBSS non-HT STA Detection flag                                   */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  Do not detect                                       */
    /*  Values to set :  0                                                   */
    /*  Configuration :  Detect, do not protect or report                    */
    /*  Values to set :  1                                                   */
    /*  Configuration :  Detect, protect and do not report                   */
    /*  Values to set :  2                                                   */
    /*  Configuration :  Detect, protect and report to other BSS             */
    /*  Values to set :  3                                                   */
    /*  --------------------------------------------------------------       */
    g_test_wid[39].id    = WID_11N_OBSS_NONHT_DETECTION;
    g_test_wid[39].type  = WID_CHAR;
    g_test_wid[39].value.c_val = 3;

    /*  11n HT Protection Type                                               */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  RTS-CTS   First Frame Exchange at non-HT-rate       */
    /*  Values to set :  0         1                                         */
    /*  Configuration :  LSIG TXOP First Frame Exchange in Mixed Fmt         */
    /*  Values to set :  2         3                                         */
    /*  --------------------------------------------------------------       */
    g_test_wid[40].id    = WID_11N_HT_PROT_TYPE;
    g_test_wid[40].type  = WID_CHAR;
    g_test_wid[40].value.c_val = 0;

    /*  11n RIFS Protection Enable Flag                                      */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  Disable    Enable                                   */
    /*  Values to set :  0          1                                        */
    /*  --------------------------------------------------------------       */
    g_test_wid[41].id    = WID_11N_RIFS_PROT_ENABLE;
    g_test_wid[41].type  = WID_CHAR;
    g_test_wid[41].value.c_val = 0;

    /*  SMPS Mode                                                            */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  Static   Dynamic   MIMO (Power Save Disabled)       */
    /*  Values to set :  1        2         3                                */
    /*  --------------------------------------------------------------       */
    g_test_wid[42].id    = WID_11N_SMPS_MODE;
    g_test_wid[42].type  = WID_CHAR;
    g_test_wid[42].value.c_val = 3;

    /*  Current transmit MCS                                                 */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  MCS Index for data rate                             */
    /*  Values to set :  0 to 127                                            */
    /*  --------------------------------------------------------------       */
    g_test_wid[43].id    = WID_11N_CURRENT_TX_MCS;
    g_test_wid[43].type  = WID_CHAR;
	// 20120709 caisf mod, merged ittiam mac v1.2 code
    g_test_wid[43].value.c_val = MAX_MCS_SUPPORTED;// 7;

    /*  Immediate Block-Ack Support                                          */
    /*  --------------------------------------------------------------       */
    /*  Configuration : Disable                  Enable                      */
    /*  Values to set :    0                       1                         */
    /*  --------------------------------------------------------------       */
    g_test_wid[44].id    = WID_11N_IMMEDIATE_BA_ENABLED;
    g_test_wid[44].type  = WID_CHAR;
    g_test_wid[44].value.c_val = 1;

    /*  Enable 20/40 Operation                                               */
    /*  --------------------------------------------------------------       */
    /*  Configuration : Disable                  Enable                      */
    /*  Values to set :    0                       1                         */
    /*  --------------------------------------------------------------       */
    g_test_wid[45].id    = WID_2040_ENABLE;
    g_test_wid[45].type  = WID_CHAR;
    g_test_wid[45].value.c_val = 0;

    /*  Enable 20/40 Coexistence Support                                     */
    /*  --------------------------------------------------------------       */
    /*  Configuration : Disable                  Enable                      */
    /*  Values to set :    0                       1                         */
    /*  --------------------------------------------------------------       */
    g_test_wid[46].id    = WID_2040_COEXISTENCE;
    g_test_wid[46].type  = WID_CHAR;
    g_test_wid[46].value.c_val = 0;

    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_test_wid                                             */
/*                                                                           */
/*  Description   : This function sets the WID ID, length and value of a     */
/*                  particular WID in the given buffer.                      */
/*                                                                           */
/*  Inputs        : 1) Pointer to the buffer in which to write the WID       */
/*                  2) Index to global test configuration structure array    */
/*                                                                           */
/*  Globals       : g_test_wid                                               */
/*                                                                           */
/*  Processing    : This function sets the WID ID, length and value accessed */
/*                  from the global test configuration structure array using */
/*                  the given count as index into the given buffer.          */
/*                                                                           */
/*  Outputs       : The given buffer is updated with the WID element         */
/*                                                                           */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 set_test_wid(UWORD8 *wid, UWORD8 count)
{
    UWORD8  wid_type = 0;
    UWORD8  wid_len  = 0;
    UWORD16 wid_id   = 0;
    UWORD8  *wid_val = 0;

    wid_id   = g_test_wid[count].id;
    wid_type = g_test_wid[count].type;

    /* The WID is set in the following format:                               */
    /* +-------------------------------------------------------------------+ */
    /* | WID Type  | WID Length | WID Value                                | */
    /* +-------------------------------------------------------------------+ */
    /* | 2 Bytes   | 1 Byte     | WID length                               | */
    /* +-------------------------------------------------------------------+ */

    /* Set the WID type. The WID length field is set at the end after        */
    /* getting the value of the WID. In case of variable length WIDs the     */
    /* length s known only after the value is obtained.                      */
    wid[0] = wid_id & 0xFF;
    wid[1] = (wid_id & 0xFF00) >> 8;

    /* Set the WID value with the value set by the user */
    wid_val = (wid + WID_VALUE_OFFSET);

    switch(wid_type)
    {
    case WID_CHAR:
    {
        UWORD8 val = g_test_wid[count].value.c_val;

        wid_len = 1;

        wid_val[0] = val;
    }
    break;

    case WID_SHORT:
    {
        UWORD16 val = g_test_wid[count].value.s_val;

        wid_len = 2;

        wid_val[0] = (UWORD8)(val & 0x00FF);
        wid_val[1] = (UWORD8)((val >> 8) & 0x00FF);
    }
    break;

    case WID_INT:
    {
        UWORD32 val = g_test_wid[count].value.i_val;

        wid_len = 4;

        wid_val[0] = (UWORD8)(val & 0x000000FF);
        wid_val[1] = (UWORD8)((val >> 8) & 0x000000FF);
        wid_val[2] = (UWORD8)((val >> 16) & 0x000000FF);
        wid_val[3] = (UWORD8)((val >> 24) & 0x000000FF);
    }
    break;

    case WID_STR:
    {
        UWORD8 *val = (UWORD8 *)g_test_wid[count].value.str_val;

        wid_len = strlen((WORD8 *)val);

        memcpy(wid_val, val, wid_len);
    }
    break;
    } /* end of switch(wid_len) */

    /* Set the length of the WID */
    wid[2] = wid_len;

    /* Return the WID response length */
    return (wid_len + WID_VALUE_OFFSET);
}

#endif /* DEFAULT_SME */
