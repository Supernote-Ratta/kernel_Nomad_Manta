/* SPDX-License-Identifier: GPL-2.0 */
/******************************************************************************
 *
 * Copyright(c) 2020, Seekwave Corporation. All right reserved.
 *
 *****************************************************************************/
#ifndef __SKW_IW_H__
#define __SKW_IW_H__

#define SKW_MAX_TLV_BUFF_LEN          1024

typedef int (*skw_at_handler)(struct skw_core *skw, void *param,
			char *args, char *resp, int resp_len);

struct skw_at_cmd {
	char *name;
	skw_at_handler handler;
	char *help_info;
};

struct skw_iw_priv_mode {
	char *name;
	enum SKW_MODE_INFO mode;
};

struct skw_iface_mib {
	u16 n_mib_len;
	u8 mib_buff[SKW_MAX_TLV_BUFF_LEN];
};

const void *skw_iw_handlers(void);
int skw_send_tlv_cmd(struct wiphy *wiphy, struct net_device *dev);
#endif
