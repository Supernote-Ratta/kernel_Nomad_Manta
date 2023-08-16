/******************************************************************************
 *
 * Copyright(c) 2020-2030  Seekwave Corporation.
 * Trace system support the bsp part.
 *
 *****************************************************************************/
#undef TRACE_SYSTEM
#define TRACE_SYSTEM skwusb

#if !defined(__SKW_USB_TRACE_H__) || defined(TRACE_HEADER_MULTI_READ)
#define __SKW_USB_TRACE_H__

#include <linux/tracepoint.h>
#include <linux/etherdevice.h>
#include "skw_usb.h"

TRACE_EVENT(skw_tx_xmit,
	    TP_PROTO(u8 *mac, int peer_idx, int ip_prot, int fix_rate,
		     int do_csum, int tid, int qlen),
	    TP_ARGS(mac, peer_idx, ip_prot, fix_rate, do_csum, tid, qlen),
	    TP_STRUCT__entry(
	    __array(u8, dest, ETH_ALEN)
	    __field(int, peer_idx)
	    __field(int, ip_prot)
	    __field(int, fix_rate)
	    __field(int, do_csum)
	    __field(int, tid)
	    __field(int, qlen)
	    ),

	    TP_fast_assign(
	    memcpy(__entry->dest, mac, ETH_ALEN);
	    __entry->peer_idx = peer_idx;
	    __entry->ip_prot = ip_prot;
	    __entry->fix_rate = fix_rate;
	    __entry->do_csum = do_csum;
	    __entry->tid = tid;
	    __entry->qlen = qlen;
	    ),

	    TP_printk("dest: %pM, peer: %d, ip_prot: %d, fix rate: %d, csum: %d, queue[%d]: %d",
		      __entry->dest, __entry->peer_idx, __entry->ip_prot,
		      __entry->fix_rate, __entry->do_csum,
		      __entry->tid, __entry->qlen)
);

TRACE_EVENT(skw_tx_info,
	    TP_PROTO(int irq, u32 qlen, u32 cred, long timeout),
	    TP_ARGS(irq, qlen, cred, timeout),

	    TP_STRUCT__entry(
	    __field(int, irq)
	    __field(u32, qlen)
	    __field(u32, cred)
	    __field(long, timeout)
	    ),

	    TP_fast_assign(
	    __entry->irq = irq;
	    __entry->qlen = qlen;
	    __entry->cred = cred;
	    __entry->timeout = timeout;
	    ),

	    TP_printk("irq: %d, pending qlen: %d, credit: %d, timeout: %ld",
		      __entry->irq, __entry->qlen,
		      __entry->cred, __entry->timeout)
);
#if 0
TRACE_EVENT(skw_tx_thread,
	    TP_PROTO(u32 cred, u32 nents, u32 tx_data_len, u32 tx_buff_len, u8 *tx_ac, u32 *cached),
	    TP_ARGS(cred, nents, tx_data_len, tx_buff_len, tx_ac, cached),

	    TP_STRUCT__entry(
	    __field(u32, cred)
	    __field(u32, nents)
	    __field(u32, tx_data_len)
	    __field(u32, tx_buff_len)
	    __array(u8, tx_ac, SKW_WMM_AC_MAX)
	    __array(u32, cached, SKW_WMM_AC_MAX)
	    ),

	    TP_fast_assign(
	    __entry->cred = cred;
	    __entry->nents = nents;
	    __entry->tx_data_len = tx_data_len;
	    __entry->tx_buff_len = tx_buff_len;
	    memcpy(__entry->tx_ac, tx_ac, sizeof(u32) * SKW_WMM_AC_MAX);
	    memcpy(__entry->cached, cached, sizeof(u32) * SKW_WMM_AC_MAX);
	    ),

	    TP_printk("creds: %d, tx: %d, len: %d/%d, VO[%d:%d] VI[%d:%d] BE[%d:%d] BK[%d:%d]",
		      __entry->cred, __entry->nents,
		      __entry->tx_data_len,
		      __entry->tx_buff_len,

		      __entry->tx_ac[SKW_WMM_AC_VO],
		      __entry->cached[SKW_WMM_AC_VO],

		      __entry->tx_ac[SKW_WMM_AC_VI],
		      __entry->cached[SKW_WMM_AC_VI],

		      __entry->tx_ac[SKW_WMM_AC_BE],
		      __entry->cached[SKW_WMM_AC_BE],

		      __entry->tx_ac[SKW_WMM_AC_BK],
		      __entry->cached[SKW_WMM_AC_BK])
);
#endif
TRACE_EVENT(skw_tx_thread_ret,
	    TP_PROTO(int ret, u32 pending_qlen, u32 ac_reset),
	    TP_ARGS(ret, pending_qlen, ac_reset),

	    TP_STRUCT__entry(
	    __field(int, ret)
	    __field(u32, pending_qlen)
	    __field(u32, ac_reset)
	    ),

	    TP_fast_assign(
	    __entry->ret = ret;
	    __entry->pending_qlen = pending_qlen;
	    __entry->ac_reset = ac_reset;
	    ),

	    TP_printk("ret: %d, pending: %d, ac_reset: 0x%x",
		      __entry->ret, __entry->pending_qlen, __entry->ac_reset)
);

TRACE_EVENT(skw_rx_reorder_timer,
	    TP_PROTO(u8 tid, u16 seq, unsigned long rx_time, unsigned long timeout),
	    TP_ARGS(tid, seq, rx_time, timeout),

	    TP_STRUCT__entry(
	    __field_struct(u8, tid)
	    __field_struct(u16, seq)
	    __field_struct(unsigned long, rx_time)
	    __field_struct(unsigned long, timeout)
	    ),

	    TP_fast_assign(
	    __entry->tid = tid;
	    __entry->seq = seq;
	    __entry->rx_time = rx_time;
	    __entry->timeout = timeout;
	    ),

	    TP_printk("tid: %d, seq: %d, rx_time: %ld, timeout: %ld",
		    __entry->tid, __entry->seq,
		    __entry->rx_time, __entry->timeout)
);

TRACE_EVENT(skw_rx_reorder_timeout,
	    TP_PROTO(u8 tid),
	    TP_ARGS(tid),

	    TP_STRUCT__entry(
	    __field_struct(u8, tid)
	    ),

	    TP_fast_assign(
	    __entry->tid = tid;
	    ),

	    TP_printk("tid: %d", __entry->tid)
);
#if 0
TRACE_EVENT(skw_rx_data,
	    TP_PROTO(struct skw_rx_desc *desc),
	    TP_ARGS(desc),

	    TP_STRUCT__entry(
	    __field_struct(struct skw_rx_desc, rx)
	    ),

	    TP_fast_assign(
	    __entry->rx = *desc;
	    ),

	    TP_printk("inst: %d, peer: %d, tid: %d, qos: %d, seq: %d, amsdu: %d, pkt_len: %d",
		      __entry->rx.inst_id, __entry->rx.peer_idx,
		      __entry->rx.tid, __entry->rx.is_qos_data,
		      __entry->rx.sn, __entry->rx.is_amsdu,
		      __entry->rx.pkt_len)
);

TRACE_EVENT(skw_rx_reorder,
	    TP_PROTO(u16 win_start, struct skw_tid_rx *tid_rx, struct skw_rx_desc *desc, bool release, bool drop),
	    TP_ARGS(win_start, tid_rx, desc, release, drop),

	    TP_STRUCT__entry(
	    __field(u16, win_start)
	    __field_struct(struct skw_tid_rx, tid_rx)
	    __field_struct(struct skw_rx_desc, desc)
	    __field(bool, release)
	    __field(bool, drop)
	    ),

	    TP_fast_assign(
	    __entry->win_start = win_start;
	    __entry->tid_rx = *tid_rx;
	    __entry->desc = *desc;
	    __entry->release = release;
	    __entry->drop = drop;
	    ),

	    TP_printk("tid: %d, size: %d, stored: %d, ssn: %d, sn: %d (ssn: %d, sn: %d), amsdu: %d, idx: %d, release: %d, drop: %d",
		      __entry->tid_rx.tid, __entry->tid_rx.win_size,
		      __entry->tid_rx.stored_num, __entry->win_start,
		      __entry->desc.sn,
		      __entry->win_start % __entry->tid_rx.win_size,
		      __entry->desc.sn % __entry->tid_rx.win_size,
		      __entry->desc.is_amsdu, __entry->desc.amsdu_idx,
		      __entry->release, __entry->drop)
);

#endif
TRACE_EVENT(skw_rx_reorder_release,
	    TP_PROTO(u16 win_start, u16 skip, u16 seq, u16 index, u16 ssn, u16 left),
	    TP_ARGS(win_start, skip, seq, index, ssn, left),

	    TP_STRUCT__entry(
	    __field(u16, win_start)
	    __field(u16, skip)
	    __field(u16, seq)
	    __field(u16, index)
	    __field(u16, ssn)
	    __field(u16, left)
	    ),

	    TP_fast_assign(
	    __entry->win_start = win_start;
	    __entry->skip = skip;
	    __entry->seq = seq;
	    __entry->index = index;
	    __entry->ssn = ssn;
	    __entry->left = left;
	    ),

	    TP_printk("win start: %d, skip: %d, seq: %d (index: %d), ssn: %d, left: %d",
		    __entry->win_start, __entry->skip, __entry->seq,
		    __entry->index, __entry->ssn, __entry->left)
);

TRACE_EVENT(skw_rx_reorder_force_release,
	    TP_PROTO(u16 index, u16 ssn, u16 left),
	    TP_ARGS(index, ssn, left),

	    TP_STRUCT__entry(
	    __field(u16, index)
	    __field(u16, ssn)
	    __field(u16, left)
	    ),

	    TP_fast_assign(
	    __entry->index = index;
	    __entry->ssn = ssn;
	    __entry->left = left;
	    ),

	    TP_printk("seq: %d(index: %d), ssn: %d, left: %d",
		    __entry->ssn - 1, __entry->index,
		    __entry->ssn, __entry->left)
);

TRACE_EVENT(skw_rx_handler_seq,
	    TP_PROTO(u16 sn),
	    TP_ARGS(sn),

	    TP_STRUCT__entry(
	    __field(u16, sn)
	    ),

	    TP_fast_assign(
	    __entry->sn = sn;
	    ),

	    TP_printk("seq: %d", __entry->sn)
);

TRACE_EVENT(skw_rx_add_ba,
	    TP_PROTO(u16 tid, u16 ssn),
	    TP_ARGS(tid, ssn),

	    TP_STRUCT__entry(
	    __field(u16, tid)
	    __field(u16, ssn)
	    ),

	    TP_fast_assign(
	    __entry->tid = tid;
	    __entry->ssn = ssn;
	    ),

	    TP_printk("tid: %d, ssn: %d", __entry->tid, __entry->ssn)
);

TRACE_EVENT(skw_rx_del_ba,
	    TP_PROTO(u16 tid),
	    TP_ARGS(tid),

	    TP_STRUCT__entry(
	    __field(u16, tid)
	    ),

	    TP_fast_assign(
	    __entry->tid = tid;
	    ),

	    TP_printk("del ba, tid: %d", __entry->tid)
);

TRACE_EVENT(skw_rx_irq,
	    TP_PROTO(int nents, int idx,  int port, int len),
	    TP_ARGS(nents, idx, port, len),

	    TP_STRUCT__entry(
	    __field(int, nents)
	    __field(int, idx)
	    __field(int, port)
	    __field(int, len)
	    ),

	    TP_fast_assign(
	    __entry->nents = nents;
	    __entry->idx = idx;
	    __entry->port = port;
	    __entry->len = len;
	    ),

	    TP_printk("nents: %d, idx: %d, port: %d, len: %d",
		      __entry->nents, __entry->idx,
		      __entry->port, __entry->len)
);
#if 0
TRACE_EVENT(skw_msg_rx,
	    TP_PROTO(struct skw_msg *msg),
	    TP_ARGS(msg),

	    TP_STRUCT__entry(
	    __field_struct(struct skw_msg, msg)
	    ),

	    TP_fast_assign(
	    __entry->msg = *msg;
	    ),

	    TP_printk("type: %d, inst: %d, id: %d, seq: %d, len: %d",
		      __entry->msg.type, __entry->msg.inst_id, __entry->msg.id,
		      __entry->msg.seq, __entry->msg.total_len)
);
#endif

#endif

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .

#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE skw_usb_trace

#include <trace/define_trace.h>
