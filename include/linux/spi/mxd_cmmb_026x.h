#ifndef __MXD_CMMB_026X_H__
#define __MXD_CMMB_026X_H__

#include <linux/types.h>

typedef void (*CMMB_POWER_FUNC)(void);
typedef int  (*CMMB_INIT_FUNC)(void);


struct mxd_cmmb_026x_platform_data {
	CMMB_POWER_FUNC poweron;
	CMMB_POWER_FUNC poweroff;
	CMMB_INIT_FUNC  init;
};


#endif

