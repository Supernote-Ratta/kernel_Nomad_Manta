/* SPDX-License-Identifier: GPL-2.0 */
/******************************************************************************
 *
 * Copyright(c) 2020-2030  Seekwave Corporation.
 *
 *****************************************************************************/
#ifndef __SKW_DEBUGFS_H__
#define __SKW_DEBUGFS_H__

#include <linux/fs.h>
#include <linux/debugfs.h>

static inline void  skw_remove_debugfs(struct dentry *dentry)
{
	debugfs_remove(dentry);
}

struct dentry *skw_add_debugfs(const char *name, umode_t mode, void *data,
			const struct file_operations *fops);

int skw_debugfs_init(void);
void skw_debugfs_deinit(void);

#endif
