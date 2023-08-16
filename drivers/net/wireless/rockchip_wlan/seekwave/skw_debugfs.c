// SPDX-License-Identifier: GPL-2.0

#include <generated/utsrelease.h>
#include "skw_debugfs.h"
#include "skw_compat.h"
#include "version.h"

static struct dentry *skw_root_dir;

static int skwifi_proc_show(struct seq_file *seq, void *v)
{
	seq_puts(seq, "\n");
	seq_printf(seq, "Kernel Version:  \t%s\n"
			"Wi-Fi Driver:    \t%s\n"
			"Wi-Fi Branch:    \t%s\n",
			UTS_RELEASE,
			SKW_VERSION,
			SKW_BRANCH);

	seq_puts(seq, "\n");
	seq_printf(seq, "STA SME EXTR:    \t%s\n"
			"SAP SME EXTR:    \t%s\n",
			IS_ENABLED(SKW_STA_SME_EXTERNAL) ? "True" : "False",
			IS_ENABLED(SKW_SAP_SME_EXTERNAL) ? "True" : "False");

	seq_puts(seq, "\n");

	return 0;
}

static int skwifi_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, skwifi_proc_show, NULL);
}

static const struct file_operations skwifi_proc_fops = {
	.open = skwifi_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

struct dentry *skw_add_debugfs(const char *name, umode_t mode, void *data,
			       const struct file_operations *fops)
{
	if (!skw_root_dir)
		return NULL;

	return debugfs_create_file(name, mode, skw_root_dir, data, fops);
}

int skw_debugfs_init(void)
{
	skw_proc_create("skwifi", 0, NULL, skwifi_proc_show, &skwifi_proc_fops);

	skw_root_dir = debugfs_create_dir("skwifi", NULL);
	if (IS_ERR(skw_root_dir)) {
		printk(KERN_ERR "create skwifi failed, ret: %ld\n",
					PTR_ERR(skw_root_dir));
		skw_root_dir = NULL;
	}

	return 0;
}

void skw_debugfs_deinit(void)
{
	debugfs_remove_recursive(skw_root_dir);
	remove_proc_entry("skwifi", NULL);
}
