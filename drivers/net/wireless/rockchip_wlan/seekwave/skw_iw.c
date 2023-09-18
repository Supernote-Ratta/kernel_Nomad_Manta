// SPDX-License-Identifier: GPL-2.0
#include <net/iw_handler.h>

#include "skw_core.h"
#include "skw_cfg80211.h"
#include "skw_iface.h"
#include "skw_iw.h"
#include "skw_log.h"

static int skw_iw_commit(struct net_device *dev, struct iw_request_info *info,
			 union iwreq_data *wrqu, char *extra)
{
	skw_dbg("traced\n");
	return 0;
}

static int skw_iw_get_name(struct net_device *dev, struct iw_request_info *info,
			   union iwreq_data *wrqu, char *extra)
{
	skw_dbg("traced\n");
	return 0;
}

static int skw_iw_set_freq(struct net_device *dev, struct iw_request_info *info,
			   union iwreq_data *wrqu, char *extra)
{
	skw_dbg("traced\n");
	return 0;
}

static int skw_iw_get_freq(struct net_device *dev, struct iw_request_info *info,
			   union iwreq_data *wrqu, char *extra)
{
	skw_dbg("traced\n");
	return 0;
}

static int skw_iw_set_mode(struct net_device *dev, struct iw_request_info *info,
			   union iwreq_data *wrqu, char *extra)
{
	skw_dbg("traced\n");
	return 0;
}

static int skw_iw_get_mode(struct net_device *dev, struct iw_request_info *info,
			   union iwreq_data *wrqu, char *extra)
{
	skw_dbg("traced\n");
	return 0;
}

static struct iw_statistics *skw_get_wireless_stats(struct net_device *dev)
{
	skw_dbg("traced\n");
	return NULL;
}

static const iw_handler skw_iw_standard_handlers[] = {
	skw_iw_commit,      /* SIOCSIWCOMMIT */
	skw_iw_get_name,    /* SIOCGIWNAME */
	NULL,               /* SIOCSIWNWID */
	NULL,               /* SIOCGIWNWID */
	skw_iw_set_freq,    /* SIOCSIWFREQ */
	skw_iw_get_freq,    /* SIOCGIWFREQ */
	skw_iw_set_mode, /**/
	skw_iw_get_mode, /**/
};

#ifdef CONFIG_WEXT_PRIV

#define SKW_SET_LEN_64            64
#define SKW_SET_LEN_128           128
#define SKW_SET_LEN_256           256
#define SKW_GET_LEN_512           512

enum SKW_IW_PRIV_ID {
	SKW_IW_PRIV_SET = SIOCIWFIRSTPRIV + 1,
	SKW_IW_PRIV_GET = SIOCIWFIRSTPRIV + 3,
	SKW_IW_PRIV_AT = SIOCIWFIRSTPRIV + 5,
	SKW_IW_PRIV_80211MODE = SIOCIWFIRSTPRIV + 6,
	SKW_IW_PRIV_GET_80211MODE = SIOCIWFIRSTPRIV + 7,
	SKW_IW_PRIV_LAST,
};

static int skw_send_at_cmd(struct skw_core *skw, char *cmd, int cmd_len,
			char *buf, int buf_len)
{
	int ret, len, resp_len;
	char *command, *resp;

	len = round_up(cmd_len, skw->hw.align);

	resp_len = round_up(buf_len, skw->hw_pdata->align_value);

	command = SKW_ALLOC(len, GFP_KERNEL);
	if (!command) {
		ret = -ENOMEM;
		goto out;
	}

	resp = SKW_ALLOC(resp_len, GFP_KERNEL);
	if (!resp) {
		ret = -ENOMEM;
		goto fail_alloc_resp;
	}

	ret = skw_uart_open(skw);
	if (ret < 0)
		goto failed;

	memcpy(command, cmd, cmd_len);
	ret = skw_uart_write(skw, command, len);
	if (ret < 0)
		goto failed;

	ret = skw_uart_read(skw, resp, resp_len);
	if (ret < 0)
		goto failed;

	memcpy(buf, resp, buf_len);
	ret = 0;

failed:
	SKW_KFREE(resp);
fail_alloc_resp:
	SKW_KFREE(command);
out:
	if (ret < 0)
		skw_err("failed: ret: %d\n", ret);

	return ret;
}

static int skw_at_set(struct skw_core *skw, void *param,
		char *args, char *resp, int resp_len)
{
	char buf[128];
	struct skw_at_cmd *cmd = param;

	if (args == NULL)
		return -EINVAL;

	sprintf(buf, "at+wifi%s=%s\r\n", cmd->name, args);

	skw_send_at_cmd(skw, buf, strlen(buf), resp, resp_len);

	return 0;
}

static int skw_at_get(struct skw_core *skw, void *param,
		char *args, char *resp, int resp_len)
{
	char buf[128];
	struct skw_at_cmd *cmd = param;

	sprintf(buf, "at+wifi%s\r\n", cmd->name);

	skw_send_at_cmd(skw, buf, strlen(buf), resp, resp_len);

	return 0;
}

static int skw_at_help(struct skw_core *skw, void *param, char *args,
			char *resp, int resp_len)
{
	int len = 0;
	struct skw_at_cmd *cmd = param;

	len = sprintf(resp, " %s:\n", cmd->help_info);
	cmd++;

	while (cmd->handler) {
		len += sprintf(resp + len, "%-4.4s %s\n", "", cmd->help_info);
		cmd++;
	}

	return 0;
}

static struct skw_at_cmd skw_at_set_cmds[] = {
	/* keep first */
	{"help", skw_at_help, "usage"},

	{"macratectrl", skw_at_set, "macratectrl=0,0,0"},
	{"macmibctrl", skw_at_set, "macmibctrl=0,0,0"},
	{"macpsctrl", skw_at_set, "macpsctrl"},
	{"macomctrl", skw_at_set, "macomctrl=0,0,0"},
	{"macbactrl", skw_at_set, "macbactrl=0,0,0"},
	{"mactxdata", skw_at_set, "mactxdata=0/1/1000"},
	{"mp", skw_at_set, "mp=0/1"},

	/*keep last*/
	{NULL, NULL, NULL}
};

static struct skw_at_cmd skw_at_get_cmds[] = {
	/* keep first */
	{"help", skw_at_help, "usage"},

	{"macstainfo", skw_at_get, "macstainfo"},

	/*keep last*/
	{NULL, NULL, NULL}
};

static struct skw_iw_priv_mode skw_iw_priv_modes[] = {
	/* keep first */
	{"11B", SKW_MODE_11B},
	{"11G", SKW_MODE_11G},
	{"11A", SKW_MODE_11A},
	{"11N", SKW_MODE_11N},
	{"11AC", SKW_MODE_11AC},
	{"11AX", SKW_MODE_11AX},
	{"11G_Only", SKW_MODE_11G_ONLY},
	{"11N_Only", SKW_MODE_11N_ONLY},

	/*keep last*/
	{NULL, 0}
};

static struct skw_at_cmd *skw_at_cmd_match(struct skw_at_cmd *cmds,
					const char *key, int key_len)
{
	int i;

	for (i = 0; cmds[i].name; i++) {
		if (!memcmp(cmds[i].name, key, key_len))
			return &cmds[i];
	}

	return NULL;
}

static int skw_iwpriv_mode(struct net_device *dev,
			   struct iw_request_info *info,
			   union iwreq_data *wrqu, char *extra)
{
	int i;
	char param[128];
	struct skw_iw_priv_mode *modes = skw_iw_priv_modes;
	struct skw_iface *iface = (struct skw_iface *)netdev_priv(dev);

	WARN_ON(sizeof(param) < wrqu->data.length);

	if (copy_from_user(param, wrqu->data.pointer, sizeof(param))) {
		skw_err("copy failed, length: %d\n",
			wrqu->data.length);

		return -EFAULT;
	}

	skw_dbg("cmd: 0x%x, %s(len: %d)\n",
		info->cmd, param, wrqu->data.length);

	for (i = 0; modes[i].name; i++) {
		if (!strcmp(modes[i].name, param)) {
			iface->iw_flags.mode = modes[i].mode;
			return 0;
		}
	}

	return -EINVAL;
}

static int skw_iwpriv_get_mode(struct net_device *dev,
			struct iw_request_info *info,
			union iwreq_data *wrqu, char *extra)
{
	skw_dbg("traced\n");
	return 0;
}

static int skw_iwpriv_set(struct net_device *dev,
			   struct iw_request_info *info,
			   union iwreq_data *wrqu, char *extra)
{
	int ret = 0;
	int key_len;
	char param[128];
	char *token, *args;
	struct skw_at_cmd *at_cmd;
	struct skw_core *skw = ((struct skw_iface *)netdev_priv(dev))->skw;

	WARN_ON(sizeof(param) < wrqu->data.length);

	if (copy_from_user(param, wrqu->data.pointer, sizeof(param))) {
		skw_err("copy failed, length: %d\n",
			wrqu->data.length);

		return -EFAULT;
	}

	skw_dbg("cmd: 0x%x, %s(len: %d)\n",
		info->cmd, param, wrqu->data.length);

	token = strchr(param, '=');
	if (!token) {
		key_len = strlen(param);
		args = NULL;
	} else {
		key_len = token - param;
		args = token + 1;
	}

	at_cmd = skw_at_cmd_match(skw_at_set_cmds, param, key_len);
	if (at_cmd)
		ret = at_cmd->handler(skw, at_cmd, args,
				extra, SKW_GET_LEN_512);
	else
		ret = skw_at_help(skw, skw_at_set_cmds, NULL,
				extra, SKW_GET_LEN_512);

	if (ret < 0)
		sprintf(extra, " usage: %s\n", at_cmd->help_info);

	wrqu->data.length = SKW_GET_LEN_512;

	skw_dbg("resp: %s\n", extra);

	return 0;
}

static int skw_iwpriv_get(struct net_device *dev,
			   struct iw_request_info *info,
			   union iwreq_data *wrqu, char *extra)
{
	int ret;
	char cmd[128];
	struct skw_at_cmd *at_cmd;
	struct skw_core *skw = ((struct skw_iface *)netdev_priv(dev))->skw;

	if (copy_from_user(cmd, wrqu->data.pointer, sizeof(cmd))) {
		skw_err("copy failed, length: %d\n",
			wrqu->data.length);

		return -EFAULT;
	}

	skw_dbg("cmd: 0x%x, %s(len: %d)\n", info->cmd, cmd, wrqu->data.length);

	at_cmd = skw_at_cmd_match(skw_at_get_cmds, cmd, strlen(cmd));
	if (at_cmd)
		ret = at_cmd->handler(skw, at_cmd, NULL, extra,
				SKW_GET_LEN_512);
	else
		ret = skw_at_help(skw, skw_at_get_cmds, NULL,
				extra, SKW_GET_LEN_512);

	wrqu->data.length = SKW_GET_LEN_512;

	skw_dbg("resp: %s\n", extra);

	return ret;
}

static int skw_iwpriv_at(struct net_device *dev,
			   struct iw_request_info *info,
			   union iwreq_data *wrqu, char *extra)
{
	int ret;
	char cmd[SKW_SET_LEN_256];
	int len = wrqu->data.length;
	struct skw_core *skw = ((struct skw_iface *)netdev_priv(dev))->skw;

	BUG_ON(sizeof(cmd) < len);

	if (copy_from_user(cmd, wrqu->data.pointer, sizeof(cmd))) {
		skw_err("copy failed, length: %d\n", len);

		return -EFAULT;
	}

	skw_dbg("cmd: %s, len: %d\n", cmd, len);

	if (len + 2 > sizeof(cmd))
		return -EINVAL;

	cmd[len - 1] = 0xd;
	cmd[len + 0] = 0xa;
	cmd[len + 1] = 0x0;

	ret = skw_send_at_cmd(skw, cmd, len + 2, extra, SKW_GET_LEN_512);

	wrqu->data.length = SKW_GET_LEN_512;

	skw_dbg("resp: %s", extra);

	return ret;
}

static struct iw_priv_args skw_iw_priv_args[] = {
	{
		SKW_IW_PRIV_SET,
		IW_PRIV_TYPE_CHAR | SKW_SET_LEN_128,
		IW_PRIV_TYPE_CHAR | SKW_GET_LEN_512,
		"set",
	},
	{
		SKW_IW_PRIV_GET,
		IW_PRIV_TYPE_CHAR | SKW_SET_LEN_128,
		IW_PRIV_TYPE_CHAR | SKW_GET_LEN_512,
		"get",
	},
	{
		SKW_IW_PRIV_AT,
		IW_PRIV_TYPE_CHAR | SKW_SET_LEN_256,
		IW_PRIV_TYPE_CHAR | SKW_GET_LEN_512,
		"at",
	},
	{
		SKW_IW_PRIV_80211MODE,
		IW_PRIV_TYPE_CHAR | SKW_SET_LEN_128,
		IW_PRIV_TYPE_CHAR | SKW_GET_LEN_512,
		"mode",
	},
	{
		SKW_IW_PRIV_GET_80211MODE,
		IW_PRIV_TYPE_CHAR | SKW_SET_LEN_128,
		IW_PRIV_TYPE_CHAR | SKW_GET_LEN_512,
		"get_mode",
	},
	{ 0, 0, 0, { 0 } }
};

static const iw_handler skw_iw_priv_handlers[] = {
	NULL,
	skw_iwpriv_set,
	NULL,
	skw_iwpriv_get,
	NULL,
	skw_iwpriv_at,
	skw_iwpriv_mode,
	skw_iwpriv_get_mode,
};

#endif

static const struct iw_handler_def skw_iw_ops = {
	.standard = skw_iw_standard_handlers,
	.num_standard = ARRAY_SIZE(skw_iw_standard_handlers),
#ifdef CONFIG_WEXT_PRIV
	.private = skw_iw_priv_handlers,
	.num_private = ARRAY_SIZE(skw_iw_priv_handlers),
	.private_args = skw_iw_priv_args,
	.num_private_args = ARRAY_SIZE(skw_iw_priv_args),
#endif
	.get_wireless_stats = skw_get_wireless_stats,
};

const void *skw_iw_handlers(void)
{
#ifdef CONFIG_WIRELESS_EXT
	return &skw_iw_ops;
#else
	skw_info("CONFIG_WIRELESS_EXT not enabled\n");
	return NULL;
#endif
}
