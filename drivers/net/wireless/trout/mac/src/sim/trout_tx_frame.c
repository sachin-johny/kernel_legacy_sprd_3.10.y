#include "trout_tx_frame.h"

static struct list_head *get_send_list(void)
{
	static LIST_HEAD(send_list);
	return &send_list;
}

static DEFINE_SPINLOCK(list_lock);
static inline void send_list_lock(void)
{
	spin_lock(&list_lock);
}

static inline void send_list_unlock(void)
{
	spin_unlock(&list_lock);
}

struct tx_frame *alloc_tx_frame(int frame_len)
{
	struct tx_frame *frame;
	frame = kzalloc(sizeof(struct tx_frame) + frame_len, GFP_KERNEL);

	if (frame) {
		frame->frame_len = frame_len;
	}

	return frame;
}

void add_tx_frame(struct tx_frame *frame)
{
	send_list_lock();
	list_add_tail(&frame->node, get_send_list());
	send_list_unlock();
}

struct tx_frame *fetch_tx_frame(void)
{
	struct tx_frame *ret = NULL;
	send_list_lock();
	if (!list_empty(get_send_list())) {
		ret = list_entry(get_send_list()->next, struct tx_frame, node);
		list_del(&ret->node);
	}
	send_list_unlock();

	return ret;
}

void del_tx_frame_tail(void)
{
	struct tx_frame *entry;
	struct list_head *head = get_send_list();

	send_list_lock();
	if (!list_empty(head)) {
		entry = list_entry(head->prev, struct tx_frame, node);
		list_del(&entry->node);
		free_tx_frame(entry);
	}
	send_list_unlock();
}

void cleanup_send_list(void)
{
	struct tx_frame *entry, *tmp;

	send_list_lock();
	list_for_each_entry_safe(entry, tmp, get_send_list(), node) {
		free_tx_frame(entry);
	}
	send_list_unlock();
}


