#ifndef __MXD_CMMB_026X_H__
#define __MXD_CMMB_026X_H__

#include <linux/types.h>

struct mxd_cmmb_026x_platform_data {
	void (*poweron)(void);
	void (*poweroff)(void);
	int  (*init)(void);
	void (*set_spi_pin_input)(void);
	void (*restore_spi_pin_cfg)(void);
};

#endif

