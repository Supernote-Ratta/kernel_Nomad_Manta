/* SPDX-License-Identifier: GPL-2.0 */
/******************************************************************************
 *
 * Copyright(c) 2020, Seekwave Corporation. All right reserved.
 *
 *****************************************************************************/
#ifndef __SKW_WORK_H__
#define __SKW_WORK_H__

struct skw_timer {
	struct list_head list;
	unsigned long timeout;
	void (*cb)(void *data);
	void *id;
	void *data;
	const char *name;
};

int skw_add_timer_work(const char *name, void (*cb)(void *dat), void *data,
		unsigned long timeout, void *timer_id, gfp_t flags);
void skw_del_timer_work(void *timer_id);
void skw_timer_init(void);
void skw_timer_deinit(void);
#endif
