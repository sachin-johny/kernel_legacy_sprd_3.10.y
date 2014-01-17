#ifndef _INCLUDE_VIRT_BUS_H
#define _INCLUDE_VIRT_BUS_H
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/kref.h>

#define BUS_NAME_MAX_LEN 32

struct virt_bus {
	const char *name;

	struct list_head attach_point_list;
	struct mutex list_lock;

	struct list_head bus_list_node;
	struct kref ref_cnt;
};

struct attach_point;

typedef int (*recv_fun_t)(struct attach_point *recipient, 
		const void *src, int size);

typedef int (*signal_fun_t)(struct attach_point *recipient,
		unsigned long signo, unsigned long param);

struct attach_point {
	void *owner;
	recv_fun_t recv;
	signal_fun_t signal;

	struct virt_bus *attached;
	struct list_head node;
};

extern int register_virt_bus(const char *name);
extern void unregister_virt_bus(const char *name);

extern void virt_bus_cleanup(void);

extern void attach_point_init(struct attach_point *ap, void *owner,
		recv_fun_t recv, signal_fun_t signal);

extern int attach_bus_by_name(struct attach_point *ap, const char *bus_name);
extern void detach_bus(struct attach_point *ap);

extern int data_through_bus(struct attach_point *sender, 
		const void *buf, int size);

extern int signal_through_bus(struct attach_point *sender,
		unsigned long signo, unsigned long param);

#endif

