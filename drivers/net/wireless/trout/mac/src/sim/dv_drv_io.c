#include <linux/kfifo.h>
#include "virt_bus.h"

struct dv_drv_ap {
	struct attach_point ap;

	struct kfifo to_drv;
};

static inline struct dv_drv_ap *get_drv_ap(void)
{
	static struct dv_drv_ap singleton;
	return &singleton;
}

static int data_to_drv(struct attach_point *dst, const void *src, int size)
{
	int ret;
	struct dv_drv_ap *drv_ap = dst->owner;

	ret = kfifo_in(&drv_ap->to_drv, src, size);

	return ret;
}

int init_drv_ap(void)
{
	int ret;
	struct dv_drv_ap *drv_ap = get_drv_ap();

	attach_point_init(&drv_ap->ap, drv_ap, data_to_drv, NULL);

	return kfifo_alloc(&drv_ap->to_drv, DATA_DEST_SPACE_SIZE, GFP_KERNEL);
}

int hwDvDataRxInWrite(char *pBuf, int bufLen, int kernMode)
{
	return data_through_bus(&get_drv_ap()->ap, pBuf, bufLen);
}

int hwDvDataTxoutRead(char *pBuf, int bufLen, int mode)
{
	return kfifo_out(&get_drv_ap()->to_drv, pBuf, bufLen);
}

