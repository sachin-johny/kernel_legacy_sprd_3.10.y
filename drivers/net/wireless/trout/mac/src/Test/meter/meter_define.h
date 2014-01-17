#ifndef _METER_DEFINE_H
#define _METER_DEFINE_H
#include <linux/list.h>
#include <linux/spinlock.h>

enum METER_PRECISION{
	JIFFIES =0,
	S,
	US,
	NS,
	MAX_PRECISION
};

enum METER_TYPE{
	METER_EVERY,
	METER_ALL
};
#endif
