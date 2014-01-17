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
/*  File Name         : test_config.h                                        */
/*                                                                           */
/*  Description       : This file contains all declarations and functions    */
/*                      related to test mode of SME.                         */
/*                                                                           */
/*  List of Functions : None                                                 */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef DEFAULT_SME

#ifndef TEST_CONFIG_H
#define TEST_CONFIG_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define NUM_TEST_WIDS         47
#define MAX_TEST_MSG_SIZE     1000
#define MAX_STRING_VAL_LEN    64
#define MAX_SSID_VAL_LEN      32
#define MAX_PSK_LEN           64
#define RAD_KEY_MAX_LEN       64
#define MAX_SUPP_USERNAME_LEN 20
#define MAX_SUPP_PASSWORD_LEN 63

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

typedef union
{
    UWORD8  c_val;
    UWORD16 s_val;
    UWORD32 i_val;
    WORD8   str_val[MAX_STRING_VAL_LEN];
} wid_val_t;

typedef struct
{
    UWORD16   id;    /* WID Identifier */
    UWORD8    type;  /* WID Data Type  */
    wid_val_t value; /* WID Value      */
} test_wid_struct_t;

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/
extern void   test_config_runtime(void);
extern void   initialize_test_config(void);
extern UWORD8 set_test_wid(UWORD8 *wid, UWORD8 count);


#endif /* TEST_CONFIG_H */

#endif /* DEFAULT_SME */
