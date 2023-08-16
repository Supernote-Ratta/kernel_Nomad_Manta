// SPDX-License-Identifier: GPL-2.0

#include <linux/skbuff.h>
#include <net/cfg80211.h>

#include "skw_core.h"
#include "skw_timer.h"
#include "skw_msg.h"
#include "skw_mlme.h"

static LIST_HEAD(skw_timer_list);
static DEFINE_SPINLOCK(skw_timer_lock);
static u32 timer_count;
static struct timer_list skw_timer;

static int skw_timer_show(struct seq_file *seq, void *data)
{
	struct skw_timer *timer;

	seq_printf(seq, "count: %d\n", timer_count);

	if (!timer_count)
		return 0;

	spin_lock_bh(&skw_timer_lock);
	list_for_each_entry(timer, &skw_timer_list, list) {
		seq_puts(seq, "\n");

		seq_printf(seq, "name: %s\n"
				"id: 0x%p\n"
				"time left: %u ms\n",
				timer->name,
				timer->id,
				jiffies_to_msecs(timer->timeout - jiffies));
	}

	spin_unlock_bh(&skw_timer_lock);

	return 0;
}

static int skw_timer_open(struct inode *inode, struct file *file)
{
	return single_open(file, skw_timer_show, inode->i_private);
}

static const struct file_operations skw_timer_fops = {
	.owner = THIS_MODULE,
	.open = skw_timer_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 15, 0)
static void skw_timer_work(struct timer_list *list)
#else
static void skw_timer_work(unsigned long data)
#endif
{
	LIST_HEAD(timeout_list);
	struct skw_timer *timer, *next;

	spin_lock_bh(&skw_timer_lock);
	list_for_each_entry_safe(timer, next, &skw_timer_list, list) {
		if (time_before(jiffies, timer->timeout))
			break;

		list_move(&timer->list, &timeout_list);
	}

	spin_unlock_bh(&skw_timer_lock);

	while (!list_empty(&timeout_list)) {
		timer = list_first_entry(&timeout_list, struct skw_timer, list);
		skw_log(SKW_TIMER, "[SKWIFI TIMER] %s: %s(id: 0x%p)\n",
			__func__, timer->name, timer->id);
		list_del(&timer->list);

		timer->cb(timer->data);
		timer_count--;

		SKW_KFREE(timer);
	}

	spin_lock_bh(&skw_timer_lock);
	timer = list_first_entry(&skw_timer_list, struct skw_timer, list);
	if (timer)
		mod_timer(&skw_timer, timer->timeout);
	spin_unlock_bh(&skw_timer_lock);
}


static bool skw_timer_id_exist(void *id)
{
	bool result = false;
	struct skw_timer *timer;

	spin_lock_bh(&skw_timer_lock);

	list_for_each_entry(timer, &skw_timer_list, list) {
		if (id == timer->id) {
			result = true;
			break;
		}
	}

	spin_unlock_bh(&skw_timer_lock);

	return result;
}

int skw_add_timer_work(const char *name, void (*cb)(void *dat), void *data,
			unsigned long timeout, void *timer_id, gfp_t flags)
{
	struct skw_timer *timer, *node;
	struct list_head *head;

	if (!timer_id || !cb)
		return -EINVAL;

	skw_log(SKW_TIMER, "[SKWIFI TIMER] %s: %s(id: 0x%p), time out = %ld\n",
		__func__, name, timer_id, timeout);

	if (skw_timer_id_exist(timer_id)) {
		skw_warn("id: 0x%p exist\n", timer_id);
		SKW_BUG_ON(1);

		return -EINVAL;
	}

	timer = SKW_ALLOC(sizeof(*timer), flags);
	if (IS_ERR(timer))
		return -ENOMEM;

	INIT_LIST_HEAD(&timer->list);

	timer->name = name;
	timer->cb = cb;
	timer->data = data;
	timer->id = timer_id;
	timer->timeout = msecs_to_jiffies(timeout) + jiffies + 1;

	spin_lock_bh(&skw_timer_lock);
	head = &skw_timer_list;

	list_for_each_entry(node, &skw_timer_list, list) {
		if (time_before_eq(timer->timeout, node->timeout)) {
			head = &node->list;
			break;
		}
	}

	list_add(&timer->list, head);


	timer_count++;
	node = list_first_entry(&skw_timer_list, struct skw_timer, list);

	mod_timer(&skw_timer, node->timeout);
	spin_unlock_bh(&skw_timer_lock);

	return 0;
}

void skw_del_timer_work(void *timer_id)
{
	struct skw_timer *timer;

	skw_log(SKW_TIMER, "[SKWIFI TIMER] %s: id: 0x%p\n",
		__func__, timer_id);

	spin_lock_bh(&skw_timer_lock);
	list_for_each_entry(timer, &skw_timer_list, list) {
		if (timer->id == timer_id) {
			list_del(&timer->list);
			timer_count--;
			SKW_KFREE(timer);
			break;
		}
	}

	timer = list_first_entry(&skw_timer_list, struct skw_timer, list);
	if (timer)
		mod_timer(&skw_timer, timer->timeout);

	spin_unlock_bh(&skw_timer_lock);
}

void skw_timer_init(void)
{
	// skw->timer_work.timeout = LONG_MAX;
	// skw->timer_work.count = 0;

	// INIT_LIST_HEAD(&skw_timer_list);
	// spin_lock_init(&skw_timer_lock);

	// fixme:
	// timer_setup(&skw_timer, skw_timer_work, 0);
	skw_compat_setup_timer(&skw_timer, skw_timer_work);
	skw_add_debugfs("timer", 04444, NULL, &skw_timer_fops);
}

void skw_timer_deinit(void)
{
	LIST_HEAD(flush_list);

	del_timer(&skw_timer);

	spin_lock_bh(&skw_timer_lock);
	list_replace_init(&skw_timer_list, &flush_list);
	spin_unlock_bh(&skw_timer_lock);

	while (!list_empty(&flush_list)) {
		struct skw_timer *timer = list_first_entry(&flush_list,
				struct skw_timer, list);
		list_del(&timer->list);
		skw_log(SKW_TIMER, "[SKWIFI TIMER] %s: name: %s\n",
			__func__, timer->name);
		SKW_KFREE(timer);
	}
}
