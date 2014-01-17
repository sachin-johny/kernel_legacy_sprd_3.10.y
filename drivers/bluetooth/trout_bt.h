#ifndef _WCH_H_
#define _WCH_H_


#include <linux/cdev.h>
//#include <linux/semaphore.h>
#include <linux/module.h>
#include <linux/kernel.h>
//#include <linux/wait.h>

struct trout_bt_dev {
	struct cdev cdev;
};

#endif