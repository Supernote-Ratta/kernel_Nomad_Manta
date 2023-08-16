/* SPDX-License-Identifier: GPL-2.0 */
/******************************************************************************
 *
 * Copyright(c) 2020-2030  Seekwave Corporation.
 *
 *****************************************************************************/
#ifndef __SKW_REGD_H__
#define __SKW_REGD_H__

#include <linux/ctype.h>

struct skw_reg_rule {
	u8 start_channel;
	u8 nr_channel;
	s8 max_power;
	s8 max_gain;
	u32 flags;
} __packed;

struct skw_regdom {
	u8 country[3];
	u8 nr_reg_rules;
	struct skw_reg_rule rules[8];
} __packed;

static inline bool skw_is_an_alpha2(const char *alpha2)
{
	if (!alpha2)
		return false;

	return isalpha(alpha2[0]) && isalpha(alpha2[1]);
}

void skw_regd_init(struct wiphy *wiphy);
int skw_set_regdom(struct wiphy *wiphy, char *country);
int skw_cmd_set_regdom(struct wiphy *wiphy, const char *alpha2,
		const struct ieee80211_regdomain *regdom);
#endif
