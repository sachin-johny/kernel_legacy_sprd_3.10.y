#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/module.h>
#include "virt_bus.h"


/*****************************************************************\
*                                                                 *
*  Virtual Bus operations                                         *
*                                                                 *
\*****************************************************************/

static inline void virt_bus_lock(struct virt_bus *bus)
{
	mutex_lock(&bus->list_lock);
}

static inline void virt_bus_unlock(struct virt_bus *bus)
{
	mutex_unlock(&bus->list_lock);
}

static void virt_bus_init(struct virt_bus *bus, const char *name)
{
	bus->name = name;
	INIT_LIST_HEAD(&bus->attach_point_list);
	INIT_LIST_HEAD(&bus->bus_list_node);
	mutex_init(&bus->list_lock);
	kref_init(&bus->ref_cnt);
}

static inline struct list_head *get_global_bus_list(void)
{
	static LIST_HEAD(bus_list);
	return &bus_list;
}

static DEFINE_SPINLOCK(bus_list_lock);

static inline void global_bus_list_lock(void)
{
	spin_lock(&bus_list_lock);
}

static inline void global_bus_list_unlock(void)
{
	spin_unlock(&bus_list_lock);
}

static inline void __add_virt_bus(struct virt_bus *bus)
{
	list_add_tail(&bus->bus_list_node, get_global_bus_list());
}

static inline void __del_virt_bus(struct virt_bus *bus)
{
	list_del(&bus->bus_list_node);
}

static inline void get_virt_bus(struct virt_bus *bus)
{
	kref_get(&bus->ref_cnt);
}

static void virt_bus_release(struct kref *kref)
{
	struct virt_bus *bus = container_of(kref, struct virt_bus, ref_cnt);

	kfree(bus);
}

static inline void put_virt_bus(struct virt_bus *bus)
{
	kref_put(&bus->ref_cnt, virt_bus_release);
}

static struct virt_bus *__find_virt_bus_by_name(const char *name)
{
	struct virt_bus *entry;

	list_for_each_entry (entry, get_global_bus_list(), bus_list_node) {
		if (!strncmp(name, entry->name, BUS_NAME_MAX_LEN)) {
			return entry;
		}
	}
	return NULL;
}

int register_virt_bus(const char *name)
{
	int ret;
	struct virt_bus *bus = kmalloc(sizeof(*bus), GFP_KERNEL);

	if (bus == NULL) {
		return -ENOMEM;
	}

	virt_bus_init(bus, name);

	global_bus_list_lock();
	if (!__find_virt_bus_by_name(name)) {
		__add_virt_bus(bus);
		ret = 0;
	} else {
		ret = -EEXIST;
	}
	global_bus_list_unlock();

	return ret;
}
EXPORT_SYMBOL(register_virt_bus);


static inline void __unregister_virt_bus(struct virt_bus *bus)
{
	__del_virt_bus(bus);
	put_virt_bus(bus);
}

void unregister_virt_bus(const char *name)
{
	struct virt_bus *bus;

	global_bus_list_lock();
	bus = __find_virt_bus_by_name(name);
	if (bus) {
		__unregister_virt_bus(bus);
	}
	global_bus_list_unlock();
}
EXPORT_SYMBOL(unregister_virt_bus);

void virt_bus_cleanup(void)
{
	struct virt_bus *entry, *tmp;

	global_bus_list_lock();
	list_for_each_entry_safe (entry, tmp, get_global_bus_list(),
			bus_list_node) {
		__unregister_virt_bus(entry);
	}
	global_bus_list_unlock();
}

/*****************************************************************\
*                                                                 *
*  Attach Point operations                                        *
*                                                                 *
\*****************************************************************/

void attach_point_init(struct attach_point *ap, void *owner,
			recv_fun_t recv, signal_fun_t signal)
{
	ap->owner = owner;
	ap->attached = NULL;
	ap->recv = recv;
	ap->signal = signal;
	INIT_LIST_HEAD(&ap->node);
}
EXPORT_SYMBOL(attach_point_init);

static void __attach_to_bus(struct attach_point *ap, struct virt_bus *bus)
{
	ap->attached = bus;
	list_add_tail(&ap->node, &bus->attach_point_list);
}

int attach_bus_by_name(struct attach_point *ap, const char *bus_name)
{
	struct virt_bus *bus;

	global_bus_list_lock();
	bus = __find_virt_bus_by_name(bus_name);
	BUG_ON(!bus);
	get_virt_bus(bus);
	global_bus_list_unlock();

	virt_bus_lock(bus);
	__attach_to_bus(ap, bus);
	virt_bus_unlock(bus);

	return 0;
}
EXPORT_SYMBOL(attach_bus_by_name);

static void __detach_bus(struct attach_point *ap)
{
	list_del(&ap->node);
	ap->attached = NULL;
}

void detach_bus(struct attach_point *ap)
{
	struct virt_bus *bus = ap->attached;

	virt_bus_lock(bus);
	__detach_bus(ap);
	virt_bus_lock(bus);

	put_virt_bus(bus);
}
EXPORT_SYMBOL(detach_bus);

int data_through_bus(struct attach_point *sender, const void *buf,
		     int size)
{
	int ret;
	struct virt_bus *bus = sender->attached;
	struct attach_point *entry;

	virt_bus_lock(bus);
	list_for_each_entry (entry, &bus->attach_point_list, node) {
		if (entry == sender || !entry->recv) {
			continue;
		}

		ret = entry->recv(entry, buf, size);
		if (ret >= 0) {
			goto done;
		}
	}
	ret = -ENODEV;
done:
	virt_bus_unlock(bus);

	return ret;
}
EXPORT_SYMBOL(data_through_bus);

int signal_through_bus(struct attach_point *sender, unsigned long signo,
		unsigned long param)
{
	int ret;
	struct virt_bus *bus = sender->attached;
	struct attach_point *entry;

	virt_bus_lock(bus);
	list_for_each_entry (entry, &bus->attach_point_list, node) {
		if (entry == sender || !entry->signal) {
			continue;
		}

		ret = entry->signal(entry, signo, param);
		if (ret >= 0) {
			goto done;
		}
	}
	ret = -ENODEV;
done:
	virt_bus_unlock(bus);

	return ret;
}
EXPORT_SYMBOL(signal_through_bus);

