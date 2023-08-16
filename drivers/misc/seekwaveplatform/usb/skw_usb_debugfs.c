/*****************************************************************************
 * Copyright(c) 2020-2030  Seekwave Corporation.
 * SEEKWAVE TECH LTD..CO
 *Seekwave Platform the usb log debug fs
 *FILENAME:skw_usb_debugfs.c
 *DATE:2022-04-11
 *MODIFY:
 *
 **************************************************************************/

#include "skw_usb_debugfs.h"
#include "skw_usb_log.h"
#include "skw_usb.h"

static struct dentry *skw_usb_root_dir;

static ssize_t skw_usb_default_read(struct file *fp, char __user *buf, size_t len,
				loff_t *offset)
{
	return 0;
}

static ssize_t skw_usb_state_write(struct file *fp, const char __user *buffer,
				size_t len, loff_t *offset)
{
	return len;
}

static const struct file_operations skw_usb_state_fops = {
	.open = skw_usb_default_open,
	.read = skw_usb_default_read,
	.write = skw_usb_state_write,
};

struct dentry *skw_usb_add_debugfs(const char *name, umode_t mode, void *data,
			       const struct file_operations *fops)
{
	skw_usb_dbg("%s:name: %s\n",__func__,name);

	return debugfs_create_file(name, mode, skw_usb_root_dir, data, fops);
}

int skw_usb_debugfs_init(void)
{
	skw_usb_root_dir = debugfs_create_dir("skwusb", NULL);
	if (IS_ERR(skw_usb_root_dir))
		return PTR_ERR(skw_usb_root_dir);

	// skw_usb_add_debugfs("state", 0666, wiphy, &skw_usb_state_fops);
	// skw_usb_add_debugfs("log_level", 0444, wiphy, &skw_usb_log_fops);

	return 0;
}

void skw_usb_debugfs_deinit(void)
{
	skw_usb_dbg("%s :traced\n", __func__);

	debugfs_remove_recursive(skw_usb_root_dir);
}
