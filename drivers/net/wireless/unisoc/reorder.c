// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2025 Otto Pflüger
 */

#include "cmd.h"
#include "txrx.h"

#define SC23XX_BA_TIMEOUT	(HZ / 10)

#define SC23XX_SEQ_INVALID	0xffff
#define SC23XX_SEQ_NUM_MASK	0xfff
#define SC23XX_SEQ_HALF_RANGE	((SC23XX_SEQ_NUM_MASK + 1) >> 1)

/* Upper bound on a firmware-supplied block-ack window (covers HT 64, VHT 256,
 * HE 1024). Guards against a bogus win_size making the reorder buffer index
 * run wild; see sc23xx_rx_addba_req(). */
#define SC23XX_MAX_BA_WIN_SIZE	1024

/*
 * Frames released from the reorder buffer are collected onto @deliver and
 * handed to the stack (sc23xx_rx_now) only after sta_lock is dropped by the
 * caller. Delivering under the lock deadlocks: sc23xx_rx_now() -> netif_rx()
 * runs the NET_RX softirq inline, which can re-enter sc23xx_start_xmit() ->
 * sc23xx_tx_prepare() and take sta_lock again on the same thread.
 */
static void reorder_buf_advance(struct sc23xx_reorder_data *r,
				unsigned int count, bool force,
				struct sk_buff_head *deliver)
{
	struct list_head *head;
	struct sk_buff *skb;
	struct sc23xx_rx_data_hdr *hdr;

	while (count--) {
		bool last_msdu = false;

		r->seq_start = (r->seq_start + 1) & SC23XX_SEQ_NUM_MASK;
		r->buf_pos = (r->buf_pos + 1) % r->win_size;
		head = &r->buf[r->buf_pos];

		while (!list_empty(head)) {
			skb = list_first_entry(head, struct sk_buff, list);
			skb_list_del_init(skb);
			if (last_msdu) {
				wiphy_dbg(r->sdev->wiphy,
					  "dropping duplicate frame 0x%03x\n",
					  r->seq_start);
				dev_kfree_skb_any(skb);
			} else {
				hdr = (void *)skb->data;
				last_msdu = le16_get_bits(hdr->flags_0,
							  SC23XX_RX_LAST_MSDU);
				__skb_queue_tail(deliver, skb);
			}
		}

		if (!last_msdu && !force)
			break;
	}
}

static void reorder_insert(struct sc23xx_reorder_data *r, struct sk_buff *skb,
			   u16 seq_num, bool last_msdu,
			   struct sk_buff_head *deliver)
{
	u16 offset;

	if (r->seq_start == SC23XX_SEQ_INVALID) {
		r->seq_start = (seq_num + !!last_msdu) & SC23XX_SEQ_NUM_MASK;
		wiphy_dbg(r->sdev->wiphy, "receiving first frame\n");
		__skb_queue_tail(deliver, skb);
		return;
	}

	offset = (seq_num - r->seq_start) & SC23XX_SEQ_NUM_MASK;
	if (offset >= SC23XX_SEQ_HALF_RANGE) {
		wiphy_dbg(r->sdev->wiphy, "dropping old frame\n");
		dev_kfree_skb_any(skb);
		return;
	}

	/*
	 * The reorder window covers offsets [0, win_size - 1]. A frame at
	 * offset >= win_size is ahead of the window and forces it forward so
	 * the new frame sits at the window tail: new head = seq - win_size + 1,
	 * i.e. advance by (offset - win_size + 1), leaving offset = win_size - 1.
	 * (The original `> win_size` / advance `offset - win_size` / clamp to
	 * win_size under-advanced by one at the edge and then indexed
	 * (buf_pos + win_size) % win_size == buf_pos, misfiling the frame into
	 * the head slot and corrupting the window.)
	 */
	if (offset >= r->win_size) {
		reorder_buf_advance(r, offset - r->win_size + 1, true, deliver);
		offset = r->win_size - 1;
	}

	if (seq_num == r->seq_start) {
		__skb_queue_tail(deliver, skb);
		if (last_msdu)
			reorder_buf_advance(r, r->win_size, false, deliver);
		mod_timer(&r->timer, jiffies + SC23XX_BA_TIMEOUT);
	} else {
		u16 idx = (r->buf_pos + offset) % r->win_size;

		INIT_LIST_HEAD(&skb->list);
		list_add_tail(&skb->list, &r->buf[idx]);

		/*
		 * A buffered frame is a hole waiting to be filled. The timer is
		 * the only thing that releases held frames when the missing
		 * seq_start never arrives; arm it here too, not just on in-order
		 * delivery. Without this, a hole that forms while no timer is
		 * pending (e.g. right after the first frame, which delivers but
		 * never armed a timer) strands the buffer forever -> silent RX
		 * wedge, no "frames lost", zero throughput.
		 */
		mod_timer(&r->timer, jiffies + SC23XX_BA_TIMEOUT);
	}
}

static void reorder_timeout(struct timer_list *t)
{
	struct sc23xx_reorder_data *r = timer_container_of(r, t, timer);
	struct sk_buff_head deliver;
	struct sk_buff *skb;
	unsigned int count;

	__skb_queue_head_init(&deliver);

	spin_lock_irq(&r->sdev->sta_lock);

	if (!r->buf)
		goto out;

	/* Drop all frames in the buffer, keeping the free space at the end. */
	count = r->win_size;
	while (count && list_empty(&r->buf[(r->buf_pos + count) % r->win_size]))
		count--;

	/* If the whole buffer is empty, nothing needs to be done. */
	if (!count)
		goto out;

	/*
	 * A frame lost on the air is ordinary, and mac80211 treats it that way:
	 * the same event goes through ht_dbg_ratelimited() in
	 * ieee80211_sta_reorder_release(), so it is compiled out unless
	 * MAC80211_HT_DEBUG is set and rate-limited even then. At warn level with
	 * no limit it buries the panel, which is this board's only console.
	 */
	wiphy_dbg(r->sdev->wiphy, "timeout, some RX frames lost\n");
	reorder_buf_advance(r, count, true, &deliver);

	/*
	 * The force-advance above stops on the highest buffered frame's slot, so
	 * seq_start ends up equal to the last delivered seq, not one past it.
	 * Step over it into the (now empty) next slot so the next in-sequence
	 * frame is treated as in-order. The force=false pass does exactly one
	 * iteration here and breaks on the empty slot. Without this, seq_start
	 * stays one behind forever: every subsequent frame lands at offset 1,
	 * gets buffered, and is only released by the next 100ms timeout ->
	 * permanent ~100ms RX latency and a "frames lost" warning per packet.
	 */
	reorder_buf_advance(r, r->win_size, false, &deliver);

out:
	spin_unlock_irq(&r->sdev->sta_lock);

	while ((skb = __skb_dequeue(&deliver)))
		sc23xx_rx_now(r->sdev, skb);
}

void sc23xx_rx_reorder(struct sc23xx_dev *sdev, struct sk_buff *skb)
{
	struct sc23xx_rx_data_hdr *hdr = (void *)skb->data;
	u16 flags_0, flags_1, seq_info, sta_lut_idx;
	struct sc23xx_reorder_data *r;
	struct sc23xx_sta *sta;
	struct sk_buff_head deliver;
	unsigned long flags;

	__skb_queue_head_init(&deliver);

	if (skb->len < sizeof(*hdr)) {
		wiphy_err(sdev->wiphy, "data packet too short\n");
		goto drop;
	}

	flags_0 = le16_to_cpu(hdr->flags_0);
	flags_1 = le16_to_cpu(hdr->flags_1);
	seq_info = le16_to_cpu(hdr->seq_info);

	wiphy_dbg(sdev->wiphy,
		  "msdu info: tid %d seqno 0x%03x flags 0x%04x%04x\n",
		  u16_get_bits(seq_info, SC23XX_RX_TID),
		  u16_get_bits(seq_info, SC23XX_RX_SEQ_NUM),
		  flags_1, flags_0);

	if ((flags_1 & SC23XX_RX_BROADCAST) ||
	    !(flags_1 & SC23XX_RX_QOS_FLAG)) {
		wiphy_dbg(sdev->wiphy,
			  "rx: broadcast, multicast or non-QoS frame\n");
		goto receive;
	}

	sta_lut_idx = u16_get_bits(flags_0, SC23XX_RX_STA_LUT_IDX);
	if (sta_lut_idx < SC23XX_STA_IDX_MIN ||
	    sta_lut_idx > SC23XX_STA_IDX_MAX) {
		wiphy_err(sdev->wiphy, "rx: STA index %d out of range\n",
			  sta_lut_idx);
		goto drop;
	}

	/*
	 * sta_lock is also taken by the reorder timer (softirq) and other paths
	 * with IRQs disabled; take it the same way here so the timer cannot fire
	 * on this CPU mid-section and self-deadlock. Released frames are queued
	 * on @deliver and pushed to the stack only after the unlock below —
	 * sc23xx_rx_now() must never run under sta_lock (netif_rx() drains the
	 * NET_RX softirq inline and re-enters sc23xx_tx_prepare(), which retakes
	 * sta_lock -> recursive deadlock).
	 */
	spin_lock_irqsave(&sdev->sta_lock, flags);

	sta = &sdev->sta[sta_lut_idx - SC23XX_STA_IDX_MIN];
	if (!sta->valid) {
		wiphy_dbg(sdev->wiphy, "rx: STA %d not found, dropping\n",
			  sta_lut_idx);
		spin_unlock_irqrestore(&sdev->sta_lock, flags);
		goto drop;
	}

	r = &sta->r[u16_get_bits(seq_info, SC23XX_RX_TID)];
	if (!r->buf) {
		wiphy_dbg(sdev->wiphy, "rx: no BA session, receiving\n");
		spin_unlock_irqrestore(&sdev->sta_lock, flags);
		goto receive;
	}

	reorder_insert(r, skb, u16_get_bits(seq_info, SC23XX_RX_SEQ_NUM),
		       u16_get_bits(flags_0, SC23XX_RX_LAST_MSDU), &deliver);

	spin_unlock_irqrestore(&sdev->sta_lock, flags);

	while ((skb = __skb_dequeue(&deliver)))
		sc23xx_rx_now(sdev, skb);

	return;

receive:
	sc23xx_rx_now(sdev, skb);
	return;

drop:
	dev_kfree_skb_any(skb);
}

void sc23xx_rx_addba_req(struct sc23xx_dev *sdev, u8 sta_lut_idx, u8 tid,
			 u16 win_start, u16 win_size)
{
	struct sc23xx_reorder_data *r;
	struct list_head *reorder_buf;
	struct sc23xx_sta *sta;
	int i, ret;

	wiphy_dbg(sdev->wiphy, "rx addba: sta %d tid %d: start 0x%03x size %u\n",
		  sta_lut_idx, tid, win_start, win_size);

	/*
	 * win_size and tid come straight from the firmware event. A zero or
	 * out-of-range win_size is fatal: reorder_buf_advance() does
	 * `buf_pos % win_size` and indexes buf[buf_pos], so win_size == 0 makes
	 * `% 0` (which does not trap on arm64) run the reorder loop off the end
	 * of the buffer forever while holding sta_lock -> hard, IRQs-off, all-
	 * CPU hang. kcalloc(0, ...) also returns ZERO_SIZE_PTR (not NULL), so
	 * the !reorder_buf check below would not catch it. Reject bad values;
	 * frames then take the no-BA path in sc23xx_rx_reorder.
	 */
	if (tid >= SC23XX_MAX_TID_NUM || win_size == 0 ||
	    win_size > SC23XX_MAX_BA_WIN_SIZE) {
		wiphy_err(sdev->wiphy,
			  "rx addba: bad tid %u / win_size %u, ignoring\n",
			  tid, win_size);
		return;
	}

	reorder_buf = kcalloc(win_size, sizeof(struct list_head), GFP_KERNEL);
	if (!reorder_buf)
		return;

	for (i = 0; i < win_size; i++)
		INIT_LIST_HEAD(&reorder_buf[i]);

	spin_lock_irq(&sdev->sta_lock);

	sta = &sdev->sta[sta_lut_idx - SC23XX_STA_IDX_MIN];
	if (sta->valid) {
		r = &sta->r[tid];
		kfree(r->buf);

		/* The firmware provides a starting sequence number in
		 * win_start, but we cannot use it because this function runs
		 * from a delayed event handler. The starting position of the
		 * buffer has to be initialized when the first frame arrives
		 * after this handler is finished.
		 */
		r->seq_start = SC23XX_SEQ_INVALID;
		r->win_size = win_size;
		r->sdev = sdev;

		if (!r->buf)
			timer_setup(&r->timer, reorder_timeout, 0);
		r->buf = reorder_buf;
	} else {
		wiphy_err(sdev->wiphy, "addba for unknown sta %d\n",
			  sta_lut_idx);
		spin_unlock_irq(&sdev->sta_lock);
		kfree(reorder_buf);
		return;
	}

	spin_unlock_irq(&sdev->sta_lock);

	ret = sc23xx_cmd_addba_rsp(sdev, sta, tid);
	if (ret)
		wiphy_err(sdev->wiphy, "failed to send addba rsp: %d\n", ret);
}

void sc23xx_rx_delba_req(struct sc23xx_dev *sdev, u8 sta_lut_idx, u8 tid)
{
	struct sc23xx_sta *sta;

	wiphy_dbg(sdev->wiphy, "rx delba: sta %d tid %d\n", sta_lut_idx, tid);

	spin_lock_irq(&sdev->sta_lock);
	sta = &sdev->sta[sta_lut_idx - SC23XX_STA_IDX_MIN];
	kfree(sta->r[tid].buf);
	sta->r[tid].buf = NULL;
	timer_shutdown(&sta->r[tid].timer);
	spin_unlock_irq(&sdev->sta_lock);
}
