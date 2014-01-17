#include <linux/mutex.h>
#include <linux/kfifo.h>
#include "../include/data_hook_space.h"

static DEFINE_KFIFO(data_hook, char, DATA_HOOK_SPACE_SIZE);
static DEFINE_MUTEX(fifo_lock);

static inline void data_hook_lock(void)
{
	mutex_lock(&fifo_lock);
}

static inline void data_hook_unlock(void)
{
	mutex_unlock(&fifo_lock);
}

int data_hook_in(char * src, int size, int from_kernel)
{
	int ret;

	data_hook_lock();

	if (from_kernel) {
		ret = kfifo_in(&data_hook, src, size);
	} else {
		kfifo_from_user(&data_hook, src, size, &ret);
	}

	data_hook_unlock();
	return ret;
}

int data_hook_out(char * dst, int size, int to_kernel)
{
	int ret;

	data_hook_lock();

	if (to_kernel) {
		ret = kfifo_out(&data_hook, dst, size);
	} else {
		kfifo_to_user(&data_hook, dst, size, &ret);
	}

	data_hook_unlock();
	return ret;
}

