/* SPDX-License-Identifier: GPL-2.0 */
/******************************************************************************
 *
 * Copyright(c) 2020, Seekwave Corporation. All right reserved.
 *
 *****************************************************************************/
#ifndef __SKW_TDLS_H__
#define __SKW_TDLS_H__

#ifdef SKW_TDLS
enum SKW_WMM_TYPE{
	SKW_WMM_TYPE_INFO,
	SKW_WMM_TYPE_PARAMETER,
};

enum SKW_TDLS_PEER_CAPA {
	SKW_TDLS_PEER_HT  = BIT(0),
	SKW_TDLS_PEER_VHT = BIT(1),
	SKW_TDLS_PEER_WMM = BIT(2),
};

int skw_tdls_build_send_mgmt(struct skw_core *skw, struct net_device *ndev,
			const u8 *peer, u8 action_code, u8 dialog_token,
			u16 status_code, u32 peer_cap, bool initiator,
			const u8 *ies, size_t ies_len);
#else
static inline int skw_tdls_build_send_mgmt(struct skw_core *skw,
			struct net_device *ndev, const u8 *peer, u8 action,
			u8 token, u16 status, u32 peer_capa, bool initiator,
			const u8 *ies, size_t ies_len)
{
	return 0;
}

#endif
#endif
