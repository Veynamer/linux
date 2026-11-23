// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2026 Veynamer <veyr1thalone@gmail.com>
 */

#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/string.h>

#include "core.h"
#include "sdio.h"

void *wcn_chan_get_drvdata(struct wcn_chan *chan)
{
	return chan->priv;
}
EXPORT_SYMBOL_GPL(wcn_chan_get_drvdata);

struct wcn_chan *wcn_chan_open(struct wcn_core *core, unsigned int id,
			       const struct wcn_chan_ops *ops, void *priv)
{
	struct wcn_chan *chan;

	if (id >= WCN_CHAN_NUM)
		return ERR_PTR(-EINVAL);

	mutex_lock(&core->lock);
	if (core->chans[id]) {
		mutex_unlock(&core->lock);
		return ERR_PTR(-EBUSY);
	}

	chan = kzalloc_obj(*chan);
	if (!chan) {
		mutex_unlock(&core->lock);
		return ERR_PTR(-ENOMEM);
	}

	chan->core = core;
	chan->id = id;
	chan->tx = id < WCN_CHAN_TX_NUM;
	chan->priv = priv;
	refcount_set(&chan->refs, 1);
	init_completion(&chan->idle);
	if (ops)
		chan->ops = *ops;
	core->chans[id] = chan;
	mutex_unlock(&core->lock);

	return chan;
}
EXPORT_SYMBOL_GPL(wcn_chan_open);

static void wcn_chan_put(struct wcn_chan *chan)
{
	/*
	 * The last reference holder only signals completion; the memory is
	 * always freed by wcn_chan_close(). Freeing here would race with the
	 * closer still spinning inside wait_for_completion() on chan->idle.
	 */
	if (refcount_dec_and_test(&chan->refs))
		complete(&chan->idle);
}

void wcn_chan_close(struct wcn_chan *chan)
{
	struct wcn_core *core;

	if (!chan)
		return;

	core = chan->core;
	mutex_lock(&core->lock);
	if (core->chans[chan->id] != chan || chan->closing) {
		mutex_unlock(&core->lock);
		return;
	}

	chan->closing = true;
	mutex_unlock(&core->lock);
	/*
	 * Keep the port reserved until existing operations finish. New TX
	 * and RX reject closing channels. Callbacks may need core->lock,
	 * so wait outside it.
	 */
	if (!refcount_dec_and_test(&chan->refs))
		wait_for_completion(&chan->idle);
	mutex_lock(&core->lock);
	core->chans[chan->id] = NULL;
	mutex_unlock(&core->lock);

	kfree(chan);
}
EXPORT_SYMBOL_GPL(wcn_chan_close);

int wcn_chan_tx_batch(struct wcn_chan *chan, const struct wcn_tx_packet *packets,
		      unsigned int num)
{
	struct wcn_core *core;
	struct wcn_puh *puh, *eof;
	size_t off = 0, total = sizeof(*eof), xfer;
	unsigned int i;
	u8 *buf;
	int ret;

	if (!chan || !packets || !num || num > WCN_TX_BATCH_MAX)
		return -EINVAL;

	core = chan->core;
	for (i = 0; i < num; i++) {
		size_t len = packets[i].len;

		if (!packets[i].data || !len || len > 0xffff)
			return -EINVAL;
		/* BSP limits each PUH plus payload to one CP packet buffer. */
		if (core->tx_packet_mode && len > core->chip->sdio.packet_size - sizeof(*puh))
			return -EMSGSIZE;
		total += ALIGN(sizeof(*puh) + len, 4);
	}
	xfer = roundup(total, core->chip->sdio.block_size);
	if (num > 1 && xfer > WCN_TX_CHUNK)
		return -EMSGSIZE;
	mutex_lock(&core->lock);
	if (!chan->tx || chan->closing || core->chans[chan->id] != chan) {
		mutex_unlock(&core->lock);
		return -ESHUTDOWN;
	}
	refcount_inc(&chan->refs);
	mutex_unlock(&core->lock);
	buf = kzalloc(xfer, GFP_KERNEL);
	if (!buf) {
		wcn_chan_put(chan);
		return -ENOMEM;
	}

	/* Match BSP's packed SG list with one EOF for the entire transfer. */
	for (i = 0; i < num; i++) {
		puh = (struct wcn_puh *)(buf + off);
		puh->type = 0;
		puh->subtype = chan->id;
		puh->len = packets[i].len;
		memcpy(buf + off + sizeof(*puh), packets[i].data, packets[i].len);
		off += ALIGN(sizeof(*puh) + packets[i].len, 4);
	}

	eof = (struct wcn_puh *)(buf + off);
	eof->eof = 1;

	ret = wcn_sdio_packet_write(core, buf, xfer);
	mutex_lock(&core->lock);
	if (ret)
		chan->tx_dropped += num;
	else
		chan->tx_packets += num;
	mutex_unlock(&core->lock);

	kfree(buf);
	wcn_chan_put(chan);
	return ret;
}
EXPORT_SYMBOL_GPL(wcn_chan_tx_batch);

int wcn_chan_tx(struct wcn_chan *chan, const void *data, size_t len)
{
	const struct wcn_tx_packet packet = { .data = data, .len = len };

	return wcn_chan_tx_batch(chan, &packet, 1);
}
EXPORT_SYMBOL_GPL(wcn_chan_tx);

void wcn_chan_rx(struct wcn_core *core, unsigned int id,
		 const void *data, size_t len)
{
	struct wcn_chan *chan;
	struct sk_buff *skb;

	if (id >= WCN_CHAN_NUM)
		return;

	mutex_lock(&core->lock);
	chan = core->chans[id];
	if (chan && !chan->closing) {
		refcount_inc(&chan->refs);
		chan->rx_packets++;
	} else {
		chan = NULL;
	}
	mutex_unlock(&core->lock);

	if (!chan)
		return;

	skb = alloc_skb(len, GFP_KERNEL);
	if (!skb) {
		chan->rx_dropped++;
		wcn_chan_put(chan);
		return;
	}

	skb_put_data(skb, data, len);
	if (chan->ops.rx)
		chan->ops.rx(chan, skb);
	else
		kfree_skb(skb);

	wcn_chan_put(chan);
}
