/******************************************************************************
 * Copyright (c) 2012-2013, Trcaicio Co., Ltd.
 * All rights reserved.
 * 
 * Filename: error.h
 * Abstract: 
 *     
 * Author:   Hugh
 * Version:  1.00
 * Revison Log:
 *     2012/03/09, Hugh: Create this file.
 * CVS Log:
 *     $Id$
 ******************************************************************************/

#ifndef _ERROR_H
#define _ERROR_H

#define DV_OK 			 0
#define DV_ERROR 		(-1)
#define DV_ER_PARAM 	(-2)		/* error param */
#define DV_ER_INTERNAL 	(-3)		/* internal error. */
#define DV_ER_NOTFOUND 	(-4)		/* not_found. */
#define DV_ER_NOMEM 	(-5)		/* no memory  */
#define DV_ER_NOPERMIT	(-6)		/* no permit   */
#define DV_ER_TIMEOUT	(-7)		/* time out    */
#define DV_ER_FULL		(-8)		/* full  */
#define DV_ER_EMPTY		(-10)		/* empty  */
#define DV_ER_NOTMATCH	(-11)		/* NOT match   */
#define DV_ER_OUTOFRANGE	(-12)		/* out of range   */
#define DV_ER_EXCPT		(-13)		/* except ocurred   */
#define DV_ER_AUTHORIZE	(-14)		/* need authorization   */
#define DV_ER_BUSY		(-15)		/* busy   */
#define DV_ER_REDIRECT	(-16)		/*  redirection  */
#define DV_ER_DIAG		(-32767)	/* for cy diagnose   */


#endif /* #ifndef _ERROR_H */

/*
 *$Log$
 */

