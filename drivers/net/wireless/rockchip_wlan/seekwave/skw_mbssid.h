/* SPDX-License-Identifier: GPL-2.0 */
/******************************************************************************
 *
 * Copyright(c) 2020, Seekwave Corporation. All right reserved.
 *
 *****************************************************************************/
#ifndef __SKW_MBSSID_H__
#define __SKW_MBSSID_H__

#define SKW_EID_EXT_NON_INHERITANCE   56

void skw_mbssid_data_parser(struct wiphy *wiphy, bool beacon,
		struct ieee80211_channel *chan, s32 signal,
		struct ieee80211_mgmt *mgmt, int mgmt_len);

#endif
