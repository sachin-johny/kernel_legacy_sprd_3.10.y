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
/*  File Name         : buff_desc.h                                          */
/*                                                                           */
/*  Description       : This file contains the declarations of buffer        */
/*                      descriptors.                                         */
/*                                                                           */
/*  List of Functions : None                                                 */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef BUFF_DESC_H
#define BUFF_DESC_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "common.h"
#include "imem_if.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define MAX_NUM_FRAGS   16

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

typedef enum {PROC_ERROR  = 0,
              PROC_LAST_MSDU = 1,
              PROC_MORE_MSDU = 2
}MSDU_PROC_STATUS_T;

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

/* Generic Structure to describe a buffer. */
typedef struct
{
    UWORD8   *buff_hdl;      /* Buffer Handle                          */
    UWORD16  data_offset;    /* Offset for the data wrt the buff_hdl   */
    UWORD16  data_length;    /* Length of data contained in the buffer */
} buffer_desc_t;

/* MSDU Descriptor Structure. */
typedef struct
{
    UWORD8  *buffer_addr;    /* Buffer for entire MSDU including Host header */
    UWORD8  host_hdr_len;    /* Length of the host header                    */
    UWORD16 host_hdr_offset; /* Host header offset from the buffer base      */
    UWORD16 data_offset;     /* MSDU data offset from the buffer base        */
    UWORD16 data_len;        /* Length of the MSDU data                      */

	//chenq add for wapi 2012-09-29
	#ifdef MAC_WAPI_SUPP
	UWORD8 wapi_mode;
	#endif
} msdu_desc_t;

/* This structure is used to maintain the intermediate state of MSDU         */
/* Processing. This is mainly used when AMSDU is enabled.                    */
typedef struct
{
    UWORD32  *curr_dscr;         /* Pointer to the current descriptor        */
    UWORD32  *curr_buffer;       /* Pointer to the current buffer            */
    UWORD16  curr_offset;        /* Offset to next Sub MSDU in curr buffer   */
    UWORD8   curr_num_msdu;      /* Number of MSDUs present in curr buffer   */
    UWORD8   curr_num_msdu_proc; /* Number of MSDUs processed in curr buffer */
    UWORD8   tot_num_dscr_proc;  /* Total number of descriptors processed    */
} msdu_proc_state_t;

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern MSDU_PROC_STATUS_T create_msdu_desc(void *wlan_rx, void *msdu);

#endif /* BUFF_DESC_H */
