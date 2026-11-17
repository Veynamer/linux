// SPDX-License-Identifier: GPL-2.0
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/string.h>

#include "wcn.h"

void wcn_chan_set_drvdata(struct wcn_chan *chan, void *data)
{
	chan->priv = data;
}
EXPORT_SYMBOL_GPL(wcn_chan_set_drvdata);

void *wcn_chan_get_drvdata(struct wcn_chan *chan)
{
	return chan->priv;
}
EXPORT_SYMBOL_GPL(wcn_chan_get_drvdata);

struct wcn_chan *wcn_chan_open(struct wcn_core *core, unsigned int id,
			       const struct wcn_chan_ops *ops)
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
	if (core->chans[chan->id] != chan) {
		mutex_unlock(&core->lock);
		return;
	}

	core->chans[chan->id] = NULL;
	chan->closing = true;
	/*
	 * Drop our own reference first: if in-flight RX still holds one,
	 * wait for the last putter to signal idle, then free. Taking the
	 * reference under core->lock guarantees no new references appear
	 * after chans[] is cleared.
	 */
	if (!refcount_dec_and_test(&chan->refs))
		wait_for_completion(&chan->idle);
	mutex_unlock(&core->lock);

	kfree(chan);
}
EXPORT_SYMBOL_GPL(wcn_chan_close);

int wcn_chan_tx(struct wcn_chan *chan, const void *data, size_t len)
{
	struct wcn_core *core;
	struct wcn_puh *puh, *eof;
	size_t eof_off, total, xfer;
	u8 *buf;
	int ret;

	if (!chan || !len || len > 0xffff)
		return -EINVAL;

	core = chan->core;
	mutex_lock(&core->lock);
	if (!chan->tx || chan->closing || core->chans[chan->id] != chan) {
		mutex_unlock(&core->lock);
		return -ESHUTDOWN;
	}
	refcount_inc(&chan->refs);
	mutex_unlock(&core->lock);
	eof_off = ALIGN(sizeof(*puh) + len, 4);
	total = eof_off + sizeof(*eof);
	xfer = ALIGN(total, WCN_BLK_SIZE);

	buf = kzalloc(xfer, GFP_KERNEL);
	if (!buf) {
		wcn_chan_put(chan);
		return -ENOMEM;
	}

	puh = (struct wcn_puh *)buf;
	puh->type = 0;
	puh->subtype = chan->id;
	puh->len = len;
	puh->eof = 0;
	memcpy(buf + sizeof(*puh), data, len);

	eof = (struct wcn_puh *)(buf + eof_off);
	eof->eof = 1;

	ret = wcn_sdio_packet_write(core, buf, xfer);
	if (ret)
		chan->tx_dropped++;
	else
		chan->tx_packets++;

	kfree(buf);
	wcn_chan_put(chan);
	return ret;
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
	if (chan) {
		refcount_inc(&chan->refs);
		chan->rx_packets++;
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
