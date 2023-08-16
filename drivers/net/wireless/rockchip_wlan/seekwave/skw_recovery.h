/* SPDX-License-Identifier: GPL-2.0 */
/******************************************************************************
 *
 * Copyright(c) 2020, Seekwave Corporation. All right reserved.
 *
 *****************************************************************************/
#ifndef __SKW_RECOVERY_H__
#define __SKW_RECOVERY_H__

int skw_recovery_data_update(struct skw_iface *iface, void *param, int len);
void skw_recovery_data_clear(struct skw_iface *iface);
int skw_recovery_init(struct skw_core *skw);
void skw_recovery_deinit(struct skw_core *skw);

#endif
