/*
 ****************************************************************
 *
 *  Component: VLCD BE component (VOGL extension)
 *
 *  Copyright (C) 2011, Red Bend Software. All Rights Reserved.
 *
 *  Contributor(s):
 *    Thomas Charleux <thomas.charleux@redbend.com>
 *
 ****************************************************************
 */

#ifndef VLCD_VOGL_H
#define VLCD_VOGL_H

#include <vlx/vlcd_backend.h>

extern vlcd_pconf_t voglPConf[VLCD_BMAX_HW_DEV_SUP][VLCD_MAX_CONF_NUMBER];

extern void voglUpdateFB  (vlcd_frontend_device_t* fDev);
extern void voglCleanupFB (vlcd_frontend_device_t* fDev);
extern int  voglInitFB    (vlcd_frontend_device_t* fDev);
extern int  voglEnabled   (void);

#endif
