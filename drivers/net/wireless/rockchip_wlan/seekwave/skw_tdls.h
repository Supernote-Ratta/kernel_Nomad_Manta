/* SPDX-License-Identifier: GPL-2.0 */
/******************************************************************************
 *
 * Copyright(c) 2020, Seekwave Corporation. All right reserved.
 *
 *****************************************************************************/
#ifndef __SKW_TDLS_H__
#define __SKW_TDLS_H__

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 11, 0)
int skw_tdls_mgmt(struct wiphy *wiphy, struct net_device *dev,
		  u8 *peer, u8 action_code,  u8 dialog_token,
		  u16 status_code, const u8 *ies, size_t ies_len);
#else
int skw_tdls_mgmt(struct wiphy *wiphy, struct net_device *dev,
		  const u8 *peer, u8 action_code,  u8 dialog_token,
		  u16 status_code, u32 peer_capability,
		  bool initiator, const u8 *ies, size_t ies_len);
#endif

#endif
