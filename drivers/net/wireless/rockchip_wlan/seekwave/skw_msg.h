/* SPDX-License-Identifier: GPL-2.0 */
/******************************************************************************
 *
 * Copyright(c) 2020-2030  Seekwave Corporation.
 *
 *****************************************************************************/
#ifndef __SKW_MSG_H__
#define __SKW_MSG_H__

#include <net/cfg80211.h>
#include "skw_core.h"

#define SKW_CMD_MAX_LEN                      1588
#define SKW_CMD_TIMEOUT                      2000

#define SKW_TXBA_COUNT_MASK                  0x3
#define SKW_TXBA_DONE                        BIT(4)
#define SKW_TXBA_WAIT_EVENT                  BIT(5)
#define SKW_TXBA_DISABLED                    BIT(6)

#define SKW_CMD_FLAG_DONE                    16
#define SKW_CMD_FLAG_XMIT                    17
#define SKW_CMD_FLAG_ACKED                   18

typedef int (*skw_event_fn)(struct skw_core *, struct skw_iface *, void *, int);

enum SKW_BA_ACTION_CMD {
	SKW_ADD_TX_BA,
	SKW_DEL_TX_BA,
	SKW_ADD_RX_BA,
	SKW_DEL_RX_BA,
	SKW_REQ_RX_BA,
};

enum SKW_TX_BA_STATUS_CODE {
	SKW_TX_BA_SETUP_SUCC,
	SKW_TX_BA_REFUSED,
	SKW_TX_BA_TIMEOUT,
	SKW_TX_BA_DEL_EVENT,
	SKW_TX_BA_DEL_SUCC,
};

struct skw_ba_action {
	u8 action;
	u8 peer_idx;
	u8 tid;
	u8 status_code;
	u16 ssn;
	u16 win_size;
} __packed;

struct skw_event_func {
	int id;
	char *name;
	skw_event_fn func;
};

enum SKW_CSA_COMMAND_MODE {
	SKW_CSA_DONE,
	SKW_CSA_START,
};

struct skw_event_csa_param {
	u8 mode;
	u8 bss_type;  /* reference SKW_CAPA_ */
	u8 chan;
	u8 center_chan1;
	u8 center_chan2;
	u8 band_width;
	u8 oper_class;		/* Only valid for csa start */
	u8 switch_mode;		/* Only valid for csa start */
	u8 switch_count;	/* Only valid for csa start */
};

enum SKW_CMD_ID {
	SKW_CMD_DOWNLOAD_INI = 0,
	SKW_CMD_GET_INFO = 1,
	SKW_CMD_SYN_VERSION = 2,
	SKW_CMD_OPEN_DEV = 3,
	SKW_CMD_CLOSE_DEV = 4,
	SKW_CMD_START_SCAN = 5,
	SKW_CMD_STOP_SCAN = 6,
	SKW_CMD_START_SCHED_SCAN = 7,
	SKW_CMD_STOP_SCHED_SCAN = 8,
	SKW_CMD_JOIN = 9,
	SKW_CMD_AUTH = 10,
	SKW_CMD_ASSOC = 11,
	SKW_CMD_ADD_KEY = 12,
	SKW_CMD_DEL_KEY = 13,
	SKW_CMD_TX_MGMT = 14 ,
	SKW_CMD_TX_DATA_FRAME = 15,
	SKW_CMD_SET_IP = 16,
	SKW_CMD_DISCONNECT = 17,
	SKW_CMD_RPM_REQ = 18,
	SKW_CMD_START_AP = 19,
	SKW_CMD_STOP_AP = 20,
	SKW_CMD_ADD_STA = 21,
	SKW_CMD_DEL_STA = 22,
	SKW_CMD_GET_STA = 23,
	SKW_CMD_RANDOM_MAC = 24,
	SKW_CMD_GET_LLSTAT = 25,
	SKW_CMD_SET_MC_ADDR = 26,
	SKW_CMD_RESUME = 27,
	SKW_CMD_SUSPEND = 28,
	SKW_CMD_REMAIN_ON_CHANNEL = 29,
	SKW_CMD_BA_ACTION = 30,
	SKW_CMD_TDLS_MGMT = 31,
	SKW_CMD_TDLS_OPER = 32,
	SKW_CMD_TDLS_CHANNEL_SWITCH = 33,
	SKW_CMD_SET_CQM_RSSI = 34,
	SKW_CMD_NPI_MSG = 35,
	SKW_CMD_IBSS_JOIN =36,
	SKW_CMD_SET_IBSS_ATTR = 37,
	SKW_CMD_RSSI_MONITOR = 38,
	SKW_CMD_SET_IE = 39,
	SKW_CMD_SET_TLV = 40,
	SKW_CMD_REGISTER_FRAME = 41,
	SKW_CMD_ADD_TX_TS = 42,
	SKW_CMD_DEL_TX_TS= 43,
	SKW_CMD_REQ_CHAN_SWITCH = 44,
	SKW_CMD_CHANGE_BEACON = 45,
	SKW_CMD_DPD_ILC_GEAR_PARAM = 46,
	SKW_CMD_DPD_ILC_MARTIX_PARAM = 47,
	SKW_CMD_DPD_ILC_COEFF_PARAM = 48,
	SKW_CMD_WIFI_RECOVER = 49,
	SKW_CMD_PHY_BB_CFG = 50,
	SKW_CMD_SET_REGD = 51,
	SKW_CMD_SET_EFUSE = 52,
	SKW_CMD_SET_PROBEREQ_FILTER = 53,
	SKW_CMD_CFG_ANT = 54,
	SKW_CMD_RTT = 55,
	SKW_CMD_GSCAN = 56,

	SKW_CMD_NUM,
};

enum SKW_EVENT_ID {
	SKW_EVENT_NORMAL_SCAN_CMPL = 0x0,
	SKW_EVENT_SCHED_SCAN_CMPL,
	SKW_EVENT_DISCONNECT,
	SKW_EVENT_ASOCC,
	SKW_EVNET_RX_MGMT,
	SKW_EVENT_DEAUTH,
	SKW_EVENT_DISASOC,
	SKW_EVENT_JOIN_CMPL,

	SKW_EVENT_ACS_REPORT,
	SKW_EVENT_DEL_STA,
	SKW_EVENT_RRM_REPORT,
	SKW_EVENT_SCAN_REPORT,
	SKW_EVENT_MGMT_TX_STATUS,
	SKW_EVENT_BA_ACTION,
	SKW_EVENT_CANCEL_ROC,
	SKW_EVENT_TDLS,
	SKW_EVENT_CREDIT_UPDATE,
	SKW_EVENT_MIC_FAILURE,
	SKW_EVENT_THERMAL_WARN,
	SKW_EVENT_RSSI_MONITOR,
	SKW_EVENT_CQM,
	SKW_EVENT_RX_UNPROTECT_FRAME,
	SKW_EVENT_CHAN_SWITCH,
	SKW_EVENT_CHN_SCH_DONE,
	SKW_EVENT_DOWNLOAD_FW,
	SKW_EVENT_TX_FRAME,
	SKW_EVENT_NPI_MP_MODE,
	SKW_EVENT_DPD_ILC_COEFF_REPORT,
	SKW_EVENT_DPD_ILC_GEAR_CMPL,
	SKW_EVENT_FW_RECOVERY,
	SKW_EVENT_TDLS_CHAN_SWITCH_RESULT,
	SKW_EVENT_THM_FW_STATE,
	SKW_EVENT_ENTER_ROC,

	SKW_EVENT_MAX
};

enum SKW_EVENT_LOCAL_ID {
	SKW_EVENT_LOCAL_STA_AUTH_ASSOC_TIMEOUT,
	SKW_EVENT_LOCAL_AP_AUTH_TIMEOUT,
	SKW_EVENT_LOCAL_STA_CONNECT,
	SKW_EVENT_LOCAL_IBSS_CONNECT,

	SKW_EVENT_LOCAL_MAX
};

enum SKW_MSG_TYPE {
	SKW_MSG_CMD,
	SKW_MSG_CMD_ACK,
	SKW_MSG_EVENT,
	SKW_MSG_EVENT_LOCAL
};

struct skw_msg {
	/* for a global message, inst_id should be 0 */
	u8 inst_id:4;

	/* reference SKW_MSG_TYPE */
	u8 type:4;
	u8 id;
	u16 seq;
	u16 total_len;
	u8 resv[2];
	u16 data[0];
};

struct skw_mgmt_hdr {
	u16 chan;
	s16 signal;
	u16 mgmt_len;
	u16 resv;
	struct ieee80211_mgmt mgmt[0];
} __packed;

struct skw_tx_mgmt_status {
	u64 cookie;
	u16 ack;
	u16 payload_len;
	const u8 mgmt;
} __packed;

struct skw_enter_roc {
	u8 inst;
	u8 chn;
	u64 cookie;
	u32 duration;
} __packed;

struct skw_cancel_roc {
	u8 inst;
	u8 chn;
	u64 cookie;
} __packed;

struct skw_mc_list {
	u16 count;
	struct mac_address mac[];
} __packed;

struct skw_discon_event_params {
	u16 reason;
	u8 bssid[ETH_ALEN];
} __packed;

/*
 * IEEE 802.2 Link Level Control headers, for use in conjunction with
 * 802.{3,4,5} media access control methods.
 *
 * Headers here do not use bit fields due to shortcommings in many
 * compilers.
 */

struct llc {
	u_int8_t llc_dsap;
	u_int8_t llc_ssap;
	union {
	    struct {
		u_int8_t control;
		u_int8_t format_id;
		u_int8_t class;
		u_int8_t window_x2;
	    } __packed type_u;
	    struct {
		u_int8_t num_snd_x2;
		u_int8_t num_rcv_x2;
	    } __packed type_i;
	    struct {
		u_int8_t control;
		u_int8_t num_rcv_x2;
	    } __packed type_s;
	    struct {
		u_int8_t control;
		/*
		 * We cannot put the following fields in a structure because
		 * the structure rounding might cause padding.
		 */
		u_int8_t frmr_rej_pdu0;
		u_int8_t frmr_rej_pdu1;
		u_int8_t frmr_control;
		u_int8_t frmr_control_ext;
		u_int8_t frmr_cause;
	    } __packed type_frmr;
	    struct {
		u_int8_t  control;
		u_int8_t  org_code[3];
		u_int16_t ether_type;
	    } __packed type_snap;
	    struct {
		u_int8_t control;
		u_int8_t control_ext;
	    } __packed type_raw;
	} llc_un /* XXX __packed ??? */;
} __packed;

struct skw_frame_tx_status {
	u8 status;
	u8 resv;
	u16 mgmt_len;
	struct ieee80211_mgmt mgmt[0];
} __packed;

static inline void skw_abort_cmd(struct skw_core *skw)
{
	if (mutex_is_locked(&skw->cmd.lock) &&
	    !test_and_set_bit(SKW_CMD_FLAG_ACKED, &skw->cmd.flags))
		skw->cmd.callback(skw);
}

int skw_msg_xmit_timeout(struct wiphy *wiphy, int dev_id, int cmd,
			 void *buf, int len, void *arg, int size,
			 char *name, unsigned long timeout);

#define skw_msg_xmit(wiphy, inst, cmd, buf, len, arg, size)             \
	skw_msg_xmit_timeout(wiphy, inst, cmd, buf, len,                \
			arg, size, #cmd,                                \
			msecs_to_jiffies(SKW_CMD_TIMEOUT))

#define SKW_NDEV_ID(d) (((struct skw_iface *)netdev_priv(d))->id)

#define skw_send_msg(wiphy, dev, cmd, buf, len, arg, size)           \
	skw_msg_xmit_timeout(wiphy, dev ? SKW_NDEV_ID(dev) : -1,     \
			cmd, buf, len, arg, size, #cmd,              \
			msecs_to_jiffies(SKW_CMD_TIMEOUT))

int skw_msg_try_send(struct skw_core *skw, int dev_id,
		     int cmd, void *data, int data_len,
		     void *arg, int arg_size, char *name);

void skw_default_event_work(struct work_struct *work);
int skw_cmd_ack_handler(struct skw_core *skw, void *data, int data_len);
void skw_event_handler(struct skw_core *skw, struct skw_iface *iface,
		       struct skw_msg *msg_hdr, void *data, size_t data_len);
int skw_queue_local_event(struct wiphy *wiphy, struct skw_iface *iface,
			  int event_id, void *data, size_t data_len);
void skw_del_sta_event(struct skw_iface *iface, const u8 *addr, u16 reason);
#endif
