// SPDX-License-Identifier: GPL-2.0
#include <net/cfg80211.h>
#include <net/genetlink.h>

#include "skw_vendor.h"
#include "skw_core.h"
#include "skw_iface.h"
#include "skw_util.h"
#include "version.h"

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)
static int skw_vendor_send_cmd_reply(struct wiphy *wiphy,
		struct net_device *dev, const void  *data, int len)
{
	struct sk_buff *skb;

	/* Alloc the SKB for vendor_event */
	skb = cfg80211_vendor_cmd_alloc_reply_skb(wiphy, len);
	if (unlikely(!skb)) {
		skw_err("skb alloc failed");
		return -ENOMEM;
	}

	/* Push the data to the skb */
	nla_put_nohdr(skb, len, data);

	return cfg80211_vendor_cmd_reply(skb);
}

#ifdef SKW_GSCAN
static int skw_vendor_gscan_get_capa(struct wiphy *wiphy,
		struct wireless_dev *wdev, const void *data, int len)
{
	int ret = 0;
	struct skw_gscan_capabilities reply;
	u32 reply_len = sizeof(struct skw_gscan_capabilities);

	skw_dbg("start getting gscan capabilities\n");
	memset(&reply, 0, reply_len);
	/* Hardcoding these values for now, need to get
	 * these values from FW, will change in a later check-in
	 */
	reply.max_scan_cache_size = GSCAN_MAX_AP_CACHE;
	reply.max_scan_buckets = GSCAN_MAX_CH_BUCKETS;
	reply.max_ap_cache_per_scan = GSCAN_MAX_AP_CACHE_PER_SCAN;
	reply.max_scan_reporting_threshold = 100;
	reply.max_hotlist_aps = PFN_HOTLIST_MAX_NUM_APS;
	reply.max_epno_ssid_crc32 = MAX_EPNO_SSID_NUM;
	reply.max_epno_hidden_ssid = MAX_EPNO_HIDDEN_SSID;
	reply.max_white_list_ssid = MAX_WHITELIST_SSID;

	ret = skw_vendor_send_cmd_reply(wiphy, wdev->netdev, &reply,
			reply_len);

	return ret;
}
#endif

static int skw_vendor_get_feature(struct wiphy *wiphy,
		struct wireless_dev *wdev, const void *data, int len)
{
	int ret = 0;
	u32 feature_set = 0;

	skw_dbg("Getting feature set\n");
	/* Hardcoding these values for now, need to get
	 * these values from FW, will change in a later check-in
	 */
	feature_set |= WIFI_FEATURE_INFRA;
	feature_set |= WIFI_FEATURE_INFRA_5G;
	feature_set |= WIFI_FEATURE_P2P;
	feature_set |= WIFI_FEATURE_SOFT_AP;
	feature_set |= WIFI_FEATURE_AP_STA;
	//feature_set |= WIFI_FEATURE_TDLS;
	//feature_set |= WIFI_FEATURE_TDLS_OFFCHANNEL;
	//feature_set |= WIFI_FEATURE_NAN;
	//feature_set |= WIFI_FEATURE_HOTSPOT;
	//feature_set |= WIFI_FEATURE_LINK_LAYER_STATS; //TBC
	//feature_set |= WIFI_FEATURE_RSSI_MONITOR; //TBC with roaming
	//feature_set |= WIFI_FEATURE_MKEEP_ALIVE; //TBC compare with QUALCOM
	//feature_set |= WIFI_FEATURE_CONFIG_NDO; //TBC
	//feature_set |= WIFI_FEATURE_SCAN_RAND;
	//feature_set |= WIFI_FEATURE_RAND_MAC;
	//feature_set |= WIFI_FEATURE_P2P_RAND_MAC ;
	//feature_set |= WIFI_FEATURE_CONTROL_ROAMING;

	ret = skw_vendor_send_cmd_reply(wiphy, wdev->netdev, &feature_set,
			sizeof(u32));

	return ret;
}

static int skw_vendor_set_rand_mac_oui(struct wiphy *wiphy,
		struct wireless_dev *wdev, const void *data, int len)
{
	int type;
	struct skw_iface *iface = netdev_priv(wdev->netdev);

	skw_dbg("setting random mac oui\n");
	type = nla_type(data);

	if (type == ANDR_WIFI_ATTRIBUTE_RANDOM_MAC_OUI)
		memcpy(iface->rand_mac_oui, nla_data(data), DOT11_OUI_LEN);

	return 0;
}

static int skw_vendor_set_country(struct wiphy *wiphy,
		struct wireless_dev *wdev, const void *data, int data_len)
{
	int rem;
	const struct nlattr *attr;

	skw_dbg("Enter");
	nla_for_each_attr(attr, data, data_len, rem) {
		switch (nla_type(attr)) {
		case ANDR_WIFI_ATTRIBUTE_COUNTRY:
			break;
		default:
			break;
		}
	}

	return 0;
}

#define SKW_ATTR_VER_DRIVER      1
#define SKW_ATTR_VER_FIRMWARE    2
static int skw_vendor_get_version(struct wiphy *wiphy,
			struct wireless_dev *wdev, const void *data, int len)
{
	int rem, type;
	const struct nlattr *attr;
	char version[256] = {0};
	struct skw_core *skw = wiphy_priv(wiphy);

	nla_for_each_attr(attr, data, len, rem) {
		type = nla_type(attr);
		switch (type) {
		case SKW_ATTR_VER_DRIVER:
			strncpy(version, SKW_VERSION, sizeof(version));
			break;

		case SKW_ATTR_VER_FIRMWARE:
			if (test_bit(SKW_FLAG_FW_ASSERT, &skw->flags)) {
				if (skw->hw_pdata->debug_info) {
					char *ver = strstr(skw->hw_pdata->debug_info, "File");
					if (!ver)
						ver = skw->hw_pdata->debug_info;

					strncpy(version, ver, sizeof(version));
				} else {
					strcpy(version, "Assert, invalid platform data");
				}
			} else {
				snprintf(version, sizeof(version), "%s-%s",
					skw->fw.plat_ver, skw->fw.wifi_ver);
			}

			break;

		default:
			snprintf(version, sizeof(version), "invalid type: %d\n", type);
			break;
		}
	}

	version[255] = 0;

	return skw_vendor_send_cmd_reply(wiphy, wdev->netdev, version, sizeof(version));
}

static int
skw_vendor_gscan_get_channel_list(struct wiphy *wiphy,
		struct wireless_dev *wdev, const void *data, int len)
{
	int err = 0, type, band, i;
	u32 channel_size = 0, num_channels, mem_needed;
	struct sk_buff *skb = NULL;
	struct net_device *ndev = wdev->netdev;
	u32 *p = NULL;

	skw_dbg("Enter");
	if (!ndev)
		return -EINVAL;

	type = nla_type(data);
	if (type == GSCAN_ATTRIBUTE_BAND)
		band = nla_get_u32(data);
	else
		return -EINVAL;

	num_channels = wiphy->bands[band]->n_channels;
	channel_size = sizeof(u32) *  num_channels;
	p = (u32 *)SKW_ALLOC(channel_size, GFP_KERNEL);

	if (!p)
		return -ENOMEM;

	for (i = 0; i < num_channels; i++)
		p[i] = wiphy->bands[band]->channels[i].center_freq;

	mem_needed = channel_size + VENDOR_REPLY_OVERHEAD
					+ (ATTRIBUTE_U32_LEN * 2);
	skb = cfg80211_vendor_cmd_alloc_reply_skb(wiphy, mem_needed);
	if (unlikely(!skb)) {
		skw_err("skb alloc failed");
		err = -ENOMEM;
		goto exit;
	}
	nla_put_u32(skb, GSCAN_ATTRIBUTE_NUM_CHANNELS, num_channels);
	nla_put(skb, GSCAN_ATTRIBUTE_CHANNEL_LIST, channel_size, p);

	err =  cfg80211_vendor_cmd_reply(skb);

	if (unlikely(err))
		skw_err("Vendor command reply failed ret:%d\n", err);

exit:
	kfree(p);
	return err;

}

static int
skw_vendor_get_wake_reason_stats(struct wiphy *wiphy,
		struct wireless_dev *wdev, const void *data, int len)
{
	wake_counts_t *pwake_count_info;
	int ret = -1, mem_needed;
	struct sk_buff *skb = NULL;

	skw_dbg("Recv get wake status info cmd.\n");
	skw_dbg("Enter");

	pwake_count_info = SKW_ALLOC(sizeof(wake_counts_t), GFP_KERNEL);
	mem_needed =  VENDOR_REPLY_OVERHEAD + (ATTRIBUTE_U32_LEN * 20) +
		(WLC_E_LAST * sizeof(uint));

	skb = cfg80211_vendor_cmd_alloc_reply_skb(wiphy, mem_needed);
	if (unlikely(!skb)) {
		skw_dbg("%s: can't allocate %d bytes\n", __func__, mem_needed);
		ret = -ENOMEM;
		goto exit;
	}

	ret = nla_put_u32(skb, WAKE_STAT_ATTRIBUTE_TOTAL_RX_DATA_WAKE,
				pwake_count_info->rxwake);
	if (unlikely(ret)) {
		skw_dbg("Failed to put Total Wake due RX data, ret=%d\n", ret);
		goto exit;
	}

	ret = cfg80211_vendor_cmd_reply(skb);
	if (unlikely(ret))
		skw_dbg("Vendor cmd reply for -get wake status failed:%d \n", ret);

	return ret;
exit:
	/* Free skb memory */
	if (skb)
		kfree_skb(skb);

	kfree(pwake_count_info);

	return ret;
}

static int
skw_vendor_apf_get_capabilities(struct wiphy *wiphy,
	struct wireless_dev *wdev, const void *data, int len)
{
	struct sk_buff *skb = NULL;
	int ret, ver, max_len, mem_needed;

	/* APF version */
	ver = 0;
	//TBD:Get the actual version
	/* APF memory size limit */
	max_len = 1024;

	mem_needed = VENDOR_REPLY_OVERHEAD + (ATTRIBUTE_U32_LEN * 2);
	skw_dbg("Enter");
	skb = cfg80211_vendor_cmd_alloc_reply_skb(wiphy, mem_needed);
	if (unlikely(!skb)) {
		skw_dbg("%s: can't allocate %d bytes\n", __func__, mem_needed);
		return -ENOMEM;
	}

	ret = nla_put_u32(skb, APF_ATTRIBUTE_VERSION, ver);
	if (ret < 0) {
		skw_dbg("Failed to put APF_ATTRIBUTE_VERSION, ret:%d\n", ret);
		goto exit;
	}
	ret = nla_put_u32(skb, APF_ATTRIBUTE_MAX_LEN, max_len);
	if (ret < 0) {
		skw_dbg("Failed to put APF_ATTRIBUTE_MAX_LEN, ret:%d\n", ret);
		goto exit;
	}

	ret = cfg80211_vendor_cmd_reply(skb);
	if (unlikely(ret))
		skw_dbg("vendor command reply failed, ret=%d\n", ret);

	return ret;
exit:
	/* Free skb memory */
	skw_dbg("Leave");
	kfree_skb(skb);
	return ret;
}

static int skw_vendor_dbg_get_ring_status(struct wiphy *wiphy,
	struct wireless_dev *wdev, const void  *data, int len)
{
	int ret = SKW_OK;
	//int ring_id, i;
	int i, ring_cnt;
	struct sk_buff *skb;

	dhd_dbg_ring_status_t dbg_ring_status[DEBUG_RING_ID_MAX] = {
		{"fw", 0, FW_VERBOSE_RING_ID, 512, 2, 2, 2, 2},
		{"host", 0, DHD_EVENT_RING_ID, 512, 3, 3, 3, 3},
	};

	skw_dbg("Enter");
	ring_cnt = 2;

	/* Alloc the SKB for vendor_event */
	skb = cfg80211_vendor_cmd_alloc_reply_skb(wiphy,
		nla_total_size(DBG_RING_STATUS_SIZE) * ring_cnt +
		nla_total_size(sizeof(ring_cnt)));
	if (!skb) {
		skw_err("skb allocation is failed\n");
		ret = -ENOMEM;
		goto exit;
	}

	/*
	 * Ignore return of nla_put_u32 and nla_put since the skb allocated
	 * above has a requested size for all payload
	 */
	(void)nla_put_u32(skb, DEBUG_ATTRIBUTE_RING_NUM, ring_cnt);
	for (i = 0; i < ring_cnt; i++) {
		(void)nla_put(skb, DEBUG_ATTRIBUTE_RING_STATUS,
				DBG_RING_STATUS_SIZE,
				&dbg_ring_status[i]);
	}

	ret = cfg80211_vendor_cmd_reply(skb);
	if (ret)
		skw_err("Vendor Command reply failed ret:%d \n", ret);

exit:
	skw_dbg("Leave");
	return ret;
}

static int skw_vendor_dbg_get_ring_data(struct wiphy *wiphy,
	struct wireless_dev *wdev, const void  *data, int len)
{
	int ret = SKW_OK, rem, type;
	char ring_name[DBGRING_NAME_MAX] = {0};
	const struct nlattr *iter;

	skw_dbg("Enter");
	nla_for_each_attr(iter, data, len, rem) {
		type = nla_type(iter);
		switch (type) {
		case DEBUG_ATTRIBUTE_RING_NAME:
			strncpy(ring_name, nla_data(iter),
				MIN(sizeof(ring_name) - 1, nla_len(iter)));
			break;
		default:
			skw_err("Unknown type: %d\n", type);
			return ret;
		}
	}

	ret = SKW_OK;
	skw_dbg("Leave");
	return ret;
}

static int skw_vendor_dbg_start_logging(struct wiphy *wiphy,
	struct wireless_dev *wdev, const void  *data, int len)
{
	int ret = SKW_OK, rem, type;
	char ring_name[DBGRING_NAME_MAX] = {0};
	int log_level = 0, flags = 0, time_intval = 0, threshold = 0;
	const struct nlattr *iter;

	skw_dbg("Enter");
	nla_for_each_attr(iter, data, len, rem) {
		type = nla_type(iter);
		switch (type) {
		case DEBUG_ATTRIBUTE_RING_NAME:
			strncpy(ring_name, nla_data(iter),
				MIN(sizeof(ring_name) - 1, nla_len(iter)));
			break;
		case DEBUG_ATTRIBUTE_LOG_LEVEL:
			log_level = nla_get_u32(iter);
			break;
		case DEBUG_ATTRIBUTE_RING_FLAGS:
			flags = nla_get_u32(iter);
			break;
		case DEBUG_ATTRIBUTE_LOG_TIME_INTVAL:
			time_intval = nla_get_u32(iter);
			break;
		case DEBUG_ATTRIBUTE_LOG_MIN_DATA_SIZE:
			threshold = nla_get_u32(iter);
			break;
		default:
			skw_err("Unknown type: %d\n", type);
			ret = -EINVAL;
			goto exit;
		}
	}

	ret = 0;
	if (ret < 0)
		skw_err("start_logging is failed ret: %d\n", ret);

exit:
	skw_dbg("leave ret:%d\n", ret);
	return ret;
}

static int skw_vendor_dbg_reset_logging(struct wiphy *wiphy,
	struct wireless_dev *wdev, const void  *data, int len)
{
	int ret = SKW_OK;

	skw_dbg("Enter\n");

	return ret;
}


static int skw_vendor_dbg_get_feature(struct wiphy *wiphy,
	struct wireless_dev *wdev, const void  *data, int len)
{
	int ret = SKW_OK;
	u32 supported_features = 0;

	skw_dbg("Enter\n");
	supported_features |= DBG_MEMORY_DUMP_SUPPORTED;
	ret = skw_vendor_send_cmd_reply(wiphy, wdev->netdev, &supported_features,
		sizeof(supported_features));
	if (ret < 0) {
		skw_err("wl_cfgvendor_send_cmd_reply failed ret:%d\n", ret);
		goto exit;
	}
exit:
	skw_dbg("leave ret:%d\n", ret);
	return ret;
}

static int skw_vendor_logger_get_firmware_memory_dump(struct wiphy *wiphy,
	struct wireless_dev *wdev, const void  *data, int len)
{
	int ret = WIFI_SUCCESS;

	skw_dbg("Enter");
	return ret;
}

static int skw_vendor_set_p2p_rand_mac(struct wiphy *wiphy,
		struct wireless_dev *wdev, const void *data, int len)
{
	int type;
	//struct skw_iface *iface = netdev_priv(wdev->netdev);
	u8 mac_addr[6] = {0};

	skw_dbg("set skw mac addr\n");
	type = nla_type(data);

	if (type == SKW_ATTR_DRIVER_RAND_MAC) {
		memcpy(mac_addr, nla_data(data), 6);
		skw_dbg("mac:%pM\n", mac_addr);
	}

	return 0;
}

static int
skw_vendor_set_hal_started(struct wiphy *wiphy,
		struct wireless_dev *wdev, const void  *data, int len)
{
	skw_dbg("Enter");
	return 0;
}

static int
skw_vendor_stop_hal(struct wiphy *wiphy,
		struct wireless_dev *wdev, const void  *data, int len)
{
	skw_dbg("Enter");
	return 0;
}

static int
skw_vendor_set_hal_pid(struct wiphy *wiphy,
		struct wireless_dev *wdev, const void  *data, int len)
{
	skw_dbg("Enter");
	return 0;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 3, 0))
const struct nla_policy skw_drv_attr_policy[SKW_ATTR_DRIVER_MAX] = {
	[SKW_ATTR_DRIVER_RAND_MAC] = { .type = NLA_BINARY, .len = 6 },
};

const struct nla_policy hal_start_attr_policy[SET_HAL_START_ATTRIBUTE_MAX] = {
	[0] = { .strict_start_type = 0 },
	[SET_HAL_START_ATTRIBUTE_DEINIT] = { .type = NLA_UNSPEC },
	[SET_HAL_START_ATTRIBUTE_PRE_INIT] = { .type = NLA_NUL_STRING },
	[SET_HAL_START_ATTRIBUTE_EVENT_SOCK_PID] = { .type = NLA_U32 },
};

const struct nla_policy andr_dbg_policy[DEBUG_ATTRIBUTE_MAX] = {
	[DEBUG_ATTRIBUTE_GET_DRIVER] = { .type = NLA_BINARY },
	[DEBUG_ATTRIBUTE_GET_FW] = { .type = NLA_BINARY },
	[DEBUG_ATTRIBUTE_RING_ID] = { .type = NLA_U32 },
	[DEBUG_ATTRIBUTE_RING_NAME] = { .type = NLA_NUL_STRING },
	[DEBUG_ATTRIBUTE_RING_FLAGS] = { .type = NLA_U32 },
	[DEBUG_ATTRIBUTE_LOG_LEVEL] = { .type = NLA_U32 },
	[DEBUG_ATTRIBUTE_LOG_TIME_INTVAL] = { .type = NLA_U32 },
	[DEBUG_ATTRIBUTE_LOG_MIN_DATA_SIZE] = { .type = NLA_U32 },
	[DEBUG_ATTRIBUTE_FW_DUMP_LEN] = { .type = NLA_U32 },
	[DEBUG_ATTRIBUTE_FW_DUMP_DATA] = { .type = NLA_U64 },
	[DEBUG_ATTRIBUTE_FW_ERR_CODE] = { .type = NLA_U32 },
	[DEBUG_ATTRIBUTE_RING_DATA] = { .type = NLA_BINARY },
	[DEBUG_ATTRIBUTE_RING_STATUS] = { .type = NLA_BINARY },
	[DEBUG_ATTRIBUTE_RING_NUM] = { .type = NLA_U32 },
};

const struct nla_policy gscan_attr_policy[GSCAN_ATTRIBUTE_MAX] = {
	[GSCAN_ATTRIBUTE_BAND] = { .type = NLA_U32 },
	[GSCAN_ATTRIBUTE_NUM_CHANNELS] = { .type = NLA_U32 },
	[GSCAN_ATTRIBUTE_CHANNEL_LIST] = { .type = NLA_BINARY },
	[GSCAN_ATTRIBUTE_WHITELIST_SSID] = { .type = NLA_BINARY, .len = IEEE80211_MAX_SSID_LEN },
	[GSCAN_ATTRIBUTE_NUM_WL_SSID] = { .type = NLA_U32 },
	[GSCAN_ATTRIBUTE_WL_SSID_LEN] = { .type = NLA_U32 },
	[GSCAN_ATTRIBUTE_WL_SSID_FLUSH] = { .type = NLA_U32 },
	[GSCAN_ATTRIBUTE_WHITELIST_SSID_ELEM] = { .type = NLA_NESTED },
	/* length is sizeof(wl_ssid_whitelist_t) * MAX_SSID_WHITELIST_NUM */
	[GSCAN_ATTRIBUTE_NUM_BSSID] = { .type = NLA_U32 },
	[GSCAN_ATTRIBUTE_BSSID_PREF_LIST] = { .type = NLA_NESTED },
	/* length is sizeof(wl_bssid_pref_list_t) * MAX_BSSID_PREF_LIST_NUM */
	[GSCAN_ATTRIBUTE_BSSID_PREF_FLUSH] = { .type = NLA_U32 },
	[GSCAN_ATTRIBUTE_BSSID_PREF] = { .type = NLA_BINARY, .len = ETH_ALEN },
	[GSCAN_ATTRIBUTE_RSSI_MODIFIER] = { .type = NLA_U32 },
	[GSCAN_ATTRIBUTE_BSSID_BLACKLIST_FLUSH] = { .type = NLA_U32 },
	[GSCAN_ATTRIBUTE_BLACKLIST_BSSID] = { .type = NLA_BINARY, .len = ETH_ALEN },
	[GSCAN_ATTRIBUTE_ROAM_STATE_SET] = { .type = NLA_U32 },
};

const struct nla_policy andr_wifi_attr_policy[ANDR_WIFI_ATTRIBUTE_MAX] = {
	[ANDR_WIFI_ATTRIBUTE_NUM_FEATURE_SET] = { .type = NLA_U32 },
	[ANDR_WIFI_ATTRIBUTE_FEATURE_SET] = { .type = NLA_U32 },
	[ANDR_WIFI_ATTRIBUTE_RANDOM_MAC_OUI] = { .type = NLA_NUL_STRING, .len = 3 },
	[ANDR_WIFI_ATTRIBUTE_NODFS_SET] = { .type = NLA_U32 },
	[ANDR_WIFI_ATTRIBUTE_COUNTRY] = { .type = NLA_NUL_STRING, .len = 3 },
	[ANDR_WIFI_ATTRIBUTE_ND_OFFLOAD_VALUE] = { .type = NLA_U8 },
	[ANDR_WIFI_ATTRIBUTE_TCPACK_SUP_VALUE] = { .type = NLA_U32 },
	[ANDR_WIFI_ATTRIBUTE_LATENCY_MODE] = { .type = NLA_U32, .len = sizeof(u32) },
	[ANDR_WIFI_ATTRIBUTE_RANDOM_MAC] = { .type = NLA_U32 },
	[ANDR_WIFI_ATTRIBUTE_TX_POWER_SCENARIO] = { .type = NLA_S8 },
	[ANDR_WIFI_ATTRIBUTE_THERMAL_MITIGATION] = { .type = NLA_S8 },
	[ANDR_WIFI_ATTRIBUTE_THERMAL_COMPLETION_WINDOW] = { .type = NLA_U32 },
	[ANDR_WIFI_ATTRIBUTE_VOIP_MODE] = { .type = NLA_U32, .len = sizeof(u32) },
	[ANDR_WIFI_ATTRIBUTE_DTIM_MULTIPLIER] = { .type = NLA_U32, .len = sizeof(u32) },
};


#endif

static struct wiphy_vendor_command skw_vendor_cmds[] = {
#ifdef SKW_GSCAN
	{
		.info = {
			.vendor_id = OUI_GOOGLE,
			.subcmd = GSCAN_SUBCMD_GET_CAPABILITIES,
		},
		.flags = WIPHY_VENDOR_CMD_NEED_WDEV |
			 WIPHY_VENDOR_CMD_NEED_NETDEV,
		.doit = skw_vendor_gscan_get_capa,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
		.policy = VENDOR_CMD_RAW_DATA,
#endif
	},
#endif
	{
		.info = {
			.vendor_id = OUI_BRCM,
			.subcmd = SKW_VENDOR_SUBCMD_SET_MAC,
		},
		.flags = WIPHY_VENDOR_CMD_NEED_WDEV,
		.doit = skw_vendor_set_p2p_rand_mac,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
		.policy = skw_drv_attr_policy,
		.maxattr = SKW_ATTR_DRIVER_MAX
#endif
	},
	{
		.info = {
			.vendor_id = OUI_GOOGLE,
			.subcmd = ANDR_WIFI_SUBCMD_GET_FEATURE_SET,
		},
		.flags = WIPHY_VENDOR_CMD_NEED_WDEV |
			 WIPHY_VENDOR_CMD_NEED_NETDEV,
		.doit = skw_vendor_get_feature,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
		.policy = VENDOR_CMD_RAW_DATA,
#endif
	},
	{
		.info = {
			.vendor_id = OUI_GOOGLE,
			.subcmd = DEBUG_GET_VER,
		},
		.flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
		.doit = skw_vendor_get_version,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
		.policy = andr_dbg_policy,
		.maxattr = SKW_ATTR_DRIVER_MAX
#endif
	},
	{
		{
			.vendor_id = OUI_GOOGLE,
			.subcmd = GSCAN_SUBCMD_GET_CHANNEL_LIST
		},
		.flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 3, 0))
		.policy = gscan_attr_policy,
		.maxattr = GSCAN_ATTRIBUTE_MAX,
#endif
		.doit = skw_vendor_gscan_get_channel_list
	},
	{
		{
			.vendor_id = OUI_GOOGLE,
			.subcmd = DEBUG_GET_WAKE_REASON_STATS
		},
		.flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 3, 0))
		.policy = VENDOR_CMD_RAW_DATA,
#endif
		.doit = skw_vendor_get_wake_reason_stats
	},
	{
		{
			.vendor_id = OUI_GOOGLE,
			.subcmd = APF_SUBCMD_GET_CAPABILITIES
		},
		.flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 3, 0))
		.policy = VENDOR_CMD_RAW_DATA,
#endif
		.doit = skw_vendor_apf_get_capabilities
	},
	{
		{
			.vendor_id = OUI_GOOGLE,
			.subcmd = DEBUG_GET_RING_STATUS
		},
		.flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 3, 0))
		.policy = andr_dbg_policy,
#endif
		.doit = skw_vendor_dbg_get_ring_status
	},
	{
		{
			.vendor_id = OUI_GOOGLE,
			.subcmd = DEBUG_GET_RING_DATA
		},
		.flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
		.policy = andr_dbg_policy,
		.maxattr = DEBUG_ATTRIBUTE_MAX,
#endif
		.doit = skw_vendor_dbg_get_ring_data
	},
	{
		{
			.vendor_id = OUI_GOOGLE,
			.subcmd = DEBUG_START_LOGGING
		},
		.flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 3, 0))
		.policy = andr_dbg_policy,
		.maxattr = DEBUG_ATTRIBUTE_MAX,
#endif
		.doit = skw_vendor_dbg_start_logging
	},
	{
		{
			.vendor_id = OUI_GOOGLE,
			.subcmd = DEBUG_RESET_LOGGING
		},
		.flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 3, 0))
		.policy = andr_dbg_policy,
#endif
		.doit = skw_vendor_dbg_reset_logging
	},
	{
		{
			.vendor_id = OUI_GOOGLE,
			.subcmd = DEBUG_GET_FEATURE
		},
		.flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 3, 0))
		.policy = VENDOR_CMD_RAW_DATA,
#endif
		.doit = skw_vendor_dbg_get_feature
	},
	{
		{
			.vendor_id = OUI_GOOGLE,
			.subcmd = DEBUG_TRIGGER_MEM_DUMP
		},
		.flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 3, 0))
		.policy = VENDOR_CMD_RAW_DATA,
#endif
		.doit = skw_vendor_logger_get_firmware_memory_dump
	},
	{
		.info = {
			.vendor_id = OUI_GOOGLE,
			.subcmd = ANDR_WIFI_RANDOM_MAC_OUI,
		},
		.flags = WIPHY_VENDOR_CMD_NEED_WDEV |
			 WIPHY_VENDOR_CMD_NEED_NETDEV,
		.doit = skw_vendor_set_rand_mac_oui,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
		.policy = VENDOR_CMD_RAW_DATA,
#endif
	},
	{
		{
			.vendor_id = OUI_GOOGLE,
			.subcmd = ANDR_WIFI_SET_COUNTRY
		},
		.flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 3, 0))
		.policy = andr_wifi_attr_policy,
		.maxattr = ANDR_WIFI_ATTRIBUTE_MAX,

#endif
		.doit = skw_vendor_set_country
	},
	{
		{
			.vendor_id = OUI_GOOGLE,
			.subcmd = DEBUG_SET_HAL_START
		},
		.flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
		.doit = skw_vendor_set_hal_started,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 3, 0))
		.policy = hal_start_attr_policy,
		.maxattr = SET_HAL_START_ATTRIBUTE_MAX
#endif /* LINUX_VERSION >= 5.3 */
	},
	{
		{
			.vendor_id = OUI_GOOGLE,
			.subcmd = DEBUG_SET_HAL_STOP
		},
		.flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 3, 0))
		.policy = VENDOR_CMD_RAW_DATA,
#endif
		.doit = skw_vendor_stop_hal
	},
	{
		{
			.vendor_id = OUI_GOOGLE,
			.subcmd = DEBUG_SET_HAL_PID
		},
		.flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
		.doit = skw_vendor_set_hal_pid,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 3, 0))
		.policy = hal_start_attr_policy,
		.maxattr = SET_HAL_START_ATTRIBUTE_MAX
#endif /* LINUX_VERSION >= 5.3 */
	},
};

static struct nl80211_vendor_cmd_info skw_vendor_events[] = {
	{
		// fixme:
		// set vendor_id & subcmd
		.vendor_id = 0,
		.subcmd = 0,
	},
};

void skw_vendor_init(struct wiphy *wiphy)
{
	wiphy->vendor_commands = skw_vendor_cmds;
	wiphy->n_vendor_commands = ARRAY_SIZE(skw_vendor_cmds);
	wiphy->vendor_events = skw_vendor_events;
	wiphy->n_vendor_events = ARRAY_SIZE(skw_vendor_events);
}

void skw_vendor_deinit(struct wiphy *wiphy)
{
	wiphy->vendor_commands = NULL;
	wiphy->n_vendor_commands = 0;
}

#else

void skw_vendor_init(struct wiphy *wiphy)
{
}

void skw_vendor_deinit(struct wiphy *wiphy)
{
}

#endif
