// SPDX-License-Identifier: GPL-2.0
#include <linux/skbuff.h>

#include "skw_core.h"
#include "skw_cfg80211.h"
#include "skw_iface.h"
#include "skw_mlme.h"
#include "skw_msg.h"
#include "skw_work.h"
#include "skw_timer.h"
#include "skw_recovery.h"
#include "skw_tx.h"
#include "skw_dfs.h"

#define SKW_WORK_FLAG_ASSERT        0
#define SKW_WORK_FLAG_RCU_FREE      1

DEFINE_SPINLOCK(skw_rcu_lock);
static struct rcu_head *skw_rcu_hdr;
static struct rcu_head **skw_rcu_tail;

static struct sk_buff_head skw_work_list;
static volatile unsigned long skw_work_flags;

static void skw_ap_acl_check(struct wiphy *wiphy, struct skw_iface *iface)
{
	int idx;
	struct skw_peer_ctx *ctx;
	u32 peer_idx_map = atomic_read(&iface->peer_map);
	struct skw_core *skw = wiphy_priv(wiphy);

	while (peer_idx_map) {

		idx = ffs(peer_idx_map) - 1;

		ctx = &skw->peer_ctx[idx];

		if (ctx->peer)
			skw_mlme_ap_del_sta(wiphy, iface->ndev,
					ctx->peer->addr, false);

		SKW_CLEAR(peer_idx_map, BIT(idx));
	}
}

static void skw_work_async_adma_tx_free(struct skw_core *skw,
				struct scatterlist *sglist, int nents)
{
	int idx, count;
	void *sg_addr;
	unsigned long flags;
	struct scatterlist *sg;
	struct sk_buff *skb, *tmp;
	struct sk_buff_head qlist;

	count = 0;
	__skb_queue_head_init(&qlist);

	spin_lock_irqsave(&skw->txq_free_list.lock, flags);
	skb_queue_splice_tail_init(&skw->txq_free_list, &qlist);
	spin_unlock_irqrestore(&skw->txq_free_list.lock, flags);

	for_each_sg(sglist, sg, nents, idx) {
		sg_addr = sg_virt(sg);
		skb_queue_walk_safe(&qlist, skb, tmp) {
			if (skb && skb->data == sg_addr) {
				__skb_unlink(skb, &qlist);
				skb->dev->stats.tx_packets++;
				skb->dev->stats.tx_bytes += SKW_SKB_TXCB(skb)->skb_native_len;
				kfree_skb(skb);
				count++;
			}
		}
	}

	if (qlist.qlen) {
		spin_lock_irqsave(&skw->txq_free_list.lock, flags);
		skb_queue_splice_tail_init(&qlist, &skw->txq_free_list);
		spin_unlock_irqrestore(&skw->txq_free_list.lock, flags);
	}

	SKW_KFREE(sglist);
}

static int skw_work_process(struct wiphy *wiphy, struct skw_iface *iface,
			int work_id, void *data, int data_len, const u8 *name)
{
	int ret = 0;
	struct skw_sg_node *node;
	struct skw_ba_action *ba;
	struct skw_core *skw = wiphy_priv(wiphy);

	skw_log(SKW_WORK, "[SKWIFI WORK]: iface: %d, %s (id: %d)\n",
		iface ? iface->id : -1, name, work_id);

	switch (work_id) {
	case SKW_WORK_BA_ACTION:
		ret = skw_send_msg(wiphy, iface->ndev, SKW_CMD_BA_ACTION,
				data, data_len, NULL, 0);
		break;

	case SKW_WORK_SCAN_TIMEOUT:
		skw_scan_done(skw, iface, true);
		break;

	case SKW_WORK_ACL_CHECK:
		skw_ap_acl_check(wiphy, iface);
		break;

	case SKW_WORK_SET_MC_ADDR:
		ret = skw_send_msg(wiphy, iface->ndev, SKW_CMD_SET_MC_ADDR,
				data, data_len, NULL, 0);
		break;

	case SKW_WORK_SET_IP:
		ret = skw_send_msg(wiphy, iface->ndev, SKW_CMD_SET_IP,
				data, data_len, NULL, 0);
		break;

	case SKW_WORK_TX_FREE:
		node = data;
		skw_work_async_adma_tx_free(skw, node->sg, node->nents);

		break;

	case SKW_WORK_SETUP_TXBA:
		ba = data;

		skw_dbg("%s, iface: %d, peer: %d, tid: %d\n",
			name, iface->id, ba->peer_idx, ba->tid);

		ret = skw_send_msg(wiphy, iface->ndev, SKW_CMD_BA_ACTION,
				data, data_len, NULL, 0);
		if (ret) {
			struct skw_peer_ctx *ctx;

			skw_err("setup TXBA failed, ret: %d\n", ret);

			ctx = skw_get_ctx(skw, ba->peer_idx);

			skw_peer_ctx_lock(ctx);

			if (ctx->peer)
				SKW_CLEAR(ctx->peer->txba.bitmap, BIT(ba->tid));

			skw_peer_ctx_unlock(ctx);
		}

		break;

	case SKW_WORK_TX_ETHER_DATA:
		skw_send_msg(wiphy, iface->ndev, SKW_CMD_TX_DATA_FRAME,
				data, data_len, NULL, 0);
		break;

	case SKW_WORK_RADAR_PULSE:
		skw_dfs_radar_pulse_event(wiphy, iface, data, data_len);
		break;

	case SKW_WORK_RADAR_CAC:
		skw_dfs_start_cac_event(wiphy, iface, data, data_len);
		break;

	case SKW_WORK_RADAR_CAC_END:
		skw_dfs_stop_cac_event(wiphy, iface);
		break;

	default:
		skw_info("invalid work: %d\n", work_id);
		break;
	}

	return ret;
}

static void skw_work(struct work_struct *work)
{
	int ret;
	struct sk_buff *skb;
	struct skw_work_cb *cb;
	struct skw_core *skw = container_of(work, struct skw_core, work);
	struct wiphy *wiphy = priv_to_wiphy(skw);

	while (skw_work_flags || skb_queue_len(&skw_work_list)) {

		if (test_bit(SKW_WORK_FLAG_RCU_FREE, &skw_work_flags)) {
			struct rcu_head *head;

			spin_lock_bh(&skw_rcu_lock);

			head = skw_rcu_hdr;
			if (head)
				skw_rcu_hdr = head->next;

			spin_unlock_bh(&skw_rcu_lock);

			if (head) {
				synchronize_rcu();
				head->func(head);
			} else {
				skw_rcu_tail = &skw_rcu_hdr;
				clear_bit(SKW_WORK_FLAG_RCU_FREE, &skw_work_flags);
			}
		}

		if (test_and_clear_bit(SKW_WORK_FLAG_ASSERT, &skw_work_flags)) {
			if (skw->hw_pdata->modem_assert)
				skw->hw_pdata->modem_assert();
		}

		if (!skb_queue_len(&skw_work_list))
			continue;

		skb = skb_dequeue(&skw_work_list);
		cb = SKW_WORK_CB(skb);
		ret = skw_work_process(wiphy, cb->iface, cb->id,
				skb->data, skb->len, cb->name);
		kfree_skb(skb);
	}
}

void skw_assert_schedule(struct wiphy *wiphy)
{
	struct skw_core *skw = wiphy_priv(wiphy);

	set_bit(SKW_WORK_FLAG_ASSERT, &skw_work_flags);
	schedule_work(&skw->work);
}

#ifdef SKW_GKI_DRV
void skw_call_rcu(void *core, struct rcu_head *head, rcu_callback_t func)
{
	struct skw_core *skw = core;

	spin_lock_bh(&skw_rcu_lock);

	head->func = func;
	head->next = NULL;

	*skw_rcu_tail = head;
	skw_rcu_tail = &head->next;

	spin_unlock_bh(&skw_rcu_lock);

	set_bit(SKW_WORK_FLAG_RCU_FREE, &skw_work_flags);

	schedule_work(&skw->work);
}
#endif

int __skw_queue_work(struct wiphy *wiphy, struct skw_iface *iface,
		     enum SKW_WORK_ID id, void *data,
		     int dat_len, const u8 *name)
{
	struct skw_core *skw = wiphy_priv(wiphy);
	struct skw_work_cb *wcb;
	struct sk_buff *skb;

	skb = dev_alloc_skb(dat_len);
	if (!skb)
		return -ENOMEM;

	if (data)
		skw_put_skb_data(skb, data, dat_len);

	wcb = SKW_WORK_CB(skb);
	wcb->iface = iface;
	wcb->id = id;
	wcb->name = name;

	skb_queue_tail(&skw_work_list, skb);
	schedule_work(&skw->work);

	return 0;
}

int skw_queue_event_work(struct wiphy *wiphy, struct skw_event_work *work,
			 struct sk_buff *skb)
{
	struct skw_core *skw = wiphy_priv(wiphy);

	if (!atomic_read(&work->enabled))
		return -EINVAL;

	skb_queue_tail(&work->qlist, skb);

	if (!work_pending(&work->work))
		queue_work(skw->event_wq, &work->work);

	return 0;
}

void skw_work_init(struct wiphy *wiphy)
{
	struct skw_core *skw = wiphy_priv(wiphy);

	skw_rcu_hdr = NULL;
	skw_rcu_tail = &skw_rcu_hdr;

	skb_queue_head_init(&skw_work_list);
	INIT_WORK(&skw->work, skw_work);
}

void skw_work_deinit(struct wiphy *wiphy)
{
	struct skw_core *skw = wiphy_priv(wiphy);

	flush_work(&skw->work);
	skb_queue_purge(&skw_work_list);
}
