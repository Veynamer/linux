// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2025 Otto Pflüger
 */

#include <linux/dma-direct.h>
#include <linux/dma-mapping.h>
#include <linux/genalloc.h>
#include <linux/io.h>
#include <linux/kthread.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/overflow.h>
#include <linux/platform_device.h>
#include <linux/xarray.h>

#include "sprd_sipc.h"

struct sipc_msg {
	u8 channel;
	u8 type;
	u16 cmd;
	u32 value;
} __packed;

struct sprd_sipc {
	struct device dev;
	struct mbox_client mbox_client;
	struct mbox_chan *mbox_chan;
	struct mbox_chan *mbox_oob_chan;
	struct workqueue_struct *wq;

	struct gen_pool *smem_pool;
	void *smem_virt;

	struct sipc_sipx *sipx;
	struct xarray channels;

	spinlock_t queue_lock;
	struct list_head queue;
	bool tx_in_progress;
};

static bool sipc_smem_dma_valid(struct device *dev, struct reserved_mem *rmem)
{
	const struct bus_dma_region *map;
	dma_addr_t dma_base, dma_end;
	phys_addr_t end;
	u64 offset;

	if (check_add_overflow(rmem->base, rmem->size - 1, &end))
		return false;

	/* All buffers must use one contiguous remote address range. */
	if (dev->dma_range_map) {
		for (map = dev->dma_range_map; map->size; map++) {
			if (rmem->base < map->cpu_start)
				continue;
			offset = rmem->base - map->cpu_start;
			if (offset < map->size && rmem->size <= map->size - offset)
				break;
		}
		if (!map->size)
			return false;
	}

	dma_base = phys_to_dma(dev, rmem->base);
	dma_end = phys_to_dma(dev, end);

	/* SIPC messages and ring headers contain 32-bit remote addresses. */
	return dma_base != DMA_MAPPING_ERROR && dma_end != DMA_MAPPING_ERROR &&
	       dma_end >= dma_base && dma_end - dma_base == rmem->size - 1 &&
	       dma_end <= U32_MAX && dma_capable(dev, dma_base, rmem->size, false);
}

static int sipc_smem_init(struct sprd_sipc *sipc)
{
	struct device *dev = &sipc->dev;
	struct device_node *np;
	struct reserved_mem *rmem;

	np = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!np)
		return -EINVAL;

	rmem = of_reserved_mem_lookup(np);
	if (!rmem || !of_property_read_bool(np, "no-map") ||
	    of_property_read_bool(np, "reusable")) {
		of_node_put(np);
		return -EINVAL;
	}
	of_node_put(np);

	if (!rmem->size || rmem->size > SIZE_MAX ||
	    !IS_ALIGNED(rmem->base, PAGE_SIZE) || !IS_ALIGNED(rmem->size, PAGE_SIZE))
		return -EINVAL;

	if (!sipc_smem_dma_valid(dev, rmem)) {
		dev_err(dev, "shared memory is outside the remote address range\n");
		return -EINVAL;
	}

	sipc->smem_virt = memremap(rmem->base, rmem->size, MEMREMAP_WC);
	if (!sipc->smem_virt)
		return -ENOMEM;

	/* Stock rings fit with page granularity, not power-of-two blocks. */
	sipc->smem_pool = gen_pool_create(PAGE_SHIFT, NUMA_NO_NODE);
	if (!sipc->smem_pool)
		return -ENOMEM;

	return gen_pool_add_virt(sipc->smem_pool, (unsigned long)sipc->smem_virt,
				 rmem->base, rmem->size, NUMA_NO_NODE);
}

void *sipc_smem_alloc(struct device *dev, size_t size, dma_addr_t *dma_addr)
{
	struct sprd_sipc *sipc = container_of(dev, struct sprd_sipc, dev);
	unsigned long addr;

	if (!size || size > gen_pool_size(sipc->smem_pool))
		return NULL;

	addr = gen_pool_alloc(sipc->smem_pool, size);
	if (!addr)
		return NULL;

	*dma_addr = phys_to_dma(dev, gen_pool_virt_to_phys(sipc->smem_pool, addr));
	memset((void *)addr, 0, size);
	/* Keep the mapping alive until deferred child releases finish. */
	get_device(dev);

	return (void *)addr;
}

void sipc_smem_free(struct device *dev, size_t size, void *addr)
{
	struct sprd_sipc *sipc = container_of(dev, struct sprd_sipc, dev);

	if (!addr)
		return;

	gen_pool_free(sipc->smem_pool, (unsigned long)addr, size);
	put_device(dev);
}

int __sipc_send(struct sprd_sipc *sipc, u8 channel, u8 type, u16 cmd, u32 value)
{
	struct sipc_msg *msg;
	int ret;

	msg = kmalloc_obj(*msg, GFP_ATOMIC);
	msg->channel = channel;
	msg->type = type;
	msg->cmd = cmd;
	msg->value = value;

	ret = mbox_send_message(sipc->mbox_chan, msg);
	if (ret < 0) {
		dev_err(&sipc->dev, "failed to send %02x %04x on channel %d: %d\n",
			type, cmd, channel, ret);
		kfree(msg);
		return ret;
	}

	return 0;
}

void sipc_queue(struct sprd_sipc *sipc, struct sipc_queued_message *msg)
{
	unsigned long flags;

	spin_lock_irqsave(&sipc->queue_lock, flags);
	if (list_empty(&msg->node)) {
		if (sipc->tx_in_progress) {
			list_add_tail(&msg->node, &sipc->queue);
		} else {
			__sipc_send(sipc, msg->channel, msg->type, msg->cmd, msg->value);
			sipc->tx_in_progress = true;
		}
	}
	spin_unlock_irqrestore(&sipc->queue_lock, flags);
}

void sipc_cancel(struct sprd_sipc *sipc, struct sipc_queued_message *msg)
{
	unsigned long flags;

	spin_lock_irqsave(&sipc->queue_lock, flags);
	if (!list_empty(&msg->node))
		list_del_init(&msg->node);
	spin_unlock_irqrestore(&sipc->queue_lock, flags);
}

static void sipc_channel_state_work(struct work_struct *work)
{
	struct sipc_channel *channel = container_of(work, struct sipc_channel, state_work);

	if (!test_bit(SIPC_FORCE_CLOSE, &channel->state) &&
	    test_bit(SIPC_REMOTE_OPEN, &channel->state) &&
	    !test_and_set_bit(SIPC_OPEN, &channel->state)) {
		if (channel->ops) {
			int ret = channel->ops->open(channel);

			if (ret) {
				dev_err(channel->parent, "failed to open channel %d: %d\n",
					channel->id, ret);
			}
		}

		dev_dbg(channel->parent, "channel %d: opened\n", channel->id);
	}

	if ((test_bit(SIPC_FORCE_CLOSE, &channel->state) ||
	     !test_bit(SIPC_REMOTE_OPEN, &channel->state)) &&
	    test_and_clear_bit(SIPC_OPEN, &channel->state)) {
		if (channel->ops)
			channel->ops->close(channel);

		reinit_completion(&channel->remote_init);

		dev_dbg(channel->parent, "channel %d: closed\n", channel->id);
	}
}

static void sipc_free_channel(struct sipc_channel *channel)
{
	if (channel->ops)
		channel->ops->free(channel);
	of_node_put(channel->node);
	kfree(channel);
}

static struct sipc_channel *sipc_alloc_channel(struct sprd_sipc *sipc, u8 id,
					       struct device_node *node,
					       const char *name)
{
	struct sipc_channel *channel;

	channel = kzalloc_obj(*channel, GFP_KERNEL);
	if (!channel)
		return NULL;

	channel->parent = &sipc->dev;
	channel->sipc = sipc;
	channel->id = id;
	channel->node = of_node_get(node);
	channel->name = name;
	init_completion(&channel->remote_init);
	INIT_WORK(&channel->state_work, sipc_channel_state_work);

	return channel;
}

struct rpmsg_endpoint *sipc_create_ept(struct sprd_sipc *sipc,
				       struct rpmsg_device *rpdev,
				       rpmsg_rx_cb_t cb, void *priv,
				       struct rpmsg_channel_info *chinfo)
{
	struct sipc_channel *channel = NULL, *ch;
	unsigned long id;

	xa_for_each_start(&sipc->channels, id, ch, 0) {
		if ((ch->name && !strncmp(chinfo->name, ch->name, RPMSG_NAME_SIZE)) ||
		    chinfo->dst == id || chinfo->src == id) {
			channel = ch;
		}
	}

	if (!channel) {
		dev_err(&sipc->dev, "channel %d/%d/%s not found\n",
			chinfo->dst, chinfo->src, chinfo->name);
		return NULL;
	}

	if (!channel->ops->setup_ept) {
		dev_err(&sipc->dev, "channel %d is not an rpmsg endpoint\n", channel->id);
		return NULL;
	}

	/* We only allow predefined source/destination combinations. */
	if (chinfo->src == RPMSG_ADDR_ANY)
		chinfo->src = channel->id;
	else if (chinfo->src != channel->id)
		return NULL;

	if (chinfo->dst == RPMSG_ADDR_ANY)
		chinfo->dst = channel->id;
	else if (chinfo->dst != channel->id)
		return NULL;

	return channel->ops->setup_ept(channel, rpdev, cb, priv);
}

static void sipc_rx_callback(struct mbox_client *mbox_client, void *msg_data)
{
	struct sprd_sipc *sipc =
		container_of(mbox_client, struct sprd_sipc, mbox_client);
	struct sipc_msg *msg = msg_data;
	struct sipc_channel *channel;

	channel = xa_load(&sipc->channels, msg->channel);

	if (msg->type == SMSG_TYPE_OPEN) {
		dev_dbg(&sipc->dev, "remote side is opening channel %d\n", msg->channel);

		if (channel) {
			if (!test_and_set_bit(SIPC_REMOTE_OPEN, &channel->state))
				queue_work(sipc->wq, &channel->state_work);

			sipc_send(channel, SMSG_TYPE_OPEN, SMSG_OPEN_MAGIC, 0);
		}
	} else if (msg->type == SMSG_TYPE_CLOSE) {
		dev_dbg(&sipc->dev, "remote side is closing channel %d\n", msg->channel);

		if (channel) {
			if (test_and_clear_bit(SIPC_REMOTE_OPEN, &channel->state))
				queue_work(sipc->wq, &channel->state_work);
		}
	} else if (channel && channel->ops) {
		channel->ops->rx(channel, msg->type, msg->cmd, msg->value);
	}
}

static void sipc_tx_done(struct mbox_client *mbox_client, void *msg_data, int r)
{
	struct sprd_sipc *sipc =
		container_of(mbox_client, struct sprd_sipc, mbox_client);
	struct sipc_msg *msg = msg_data;

	if (r)
		dev_warn(mbox_client->dev, "message on channel %d was not delivered: %d\n",
			 msg->channel, r);

	kfree(msg_data);

	spin_lock(&sipc->queue_lock);
	if (list_empty(&sipc->queue)) {
		sipc->tx_in_progress = false;
	} else {
		struct sipc_queued_message *qmsg =
			list_first_entry(&sipc->queue, struct sipc_queued_message, node);

		list_del_init(&qmsg->node);
		__sipc_send(sipc, qmsg->channel, qmsg->type, qmsg->cmd, qmsg->value);
	}
	spin_unlock(&sipc->queue_lock);
}

static int sipc_populate(struct sprd_sipc *sipc)
{
	struct sipc_channel *channel;
	struct device_node *child;
	const char *name, *chtype;
	int ret;
	u32 id;

	for_each_available_child_of_node(sipc->dev.of_node, child) {
		ret = of_property_read_u32(child, "reg", &id);
		if (ret)
			continue;

		ret = of_property_read_string(child, "sprd,channel-type", &chtype);
		if (ret)
			continue;

		ret = of_property_read_string(child, "sprd,channel-name", &name);
		if (ret)
			continue;

		channel = sipc_alloc_channel(sipc, id, child, name);
		if (!channel)
			return -ENOMEM;

		if (!strcmp(chtype, "sbuf")) {
			ret = sipc_sbuf_init(channel);
		} else if (!strcmp(chtype, "sblock")) {
			ret = sipc_sblock_init(channel);
		} else if (!strcmp(chtype, "sipx")) {
			ret = 0;
			if (!sipc->sipx) {
				sipc->sipx = sipx_init(&sipc->dev);
				if (IS_ERR(sipc->sipx)) {
					ret = PTR_ERR(sipc->sipx);
					sipc->sipx = NULL;
				}
			}
			if (!ret)
				ret = sipx_channel_init(sipc->sipx, channel);
		} else {
			ret = 0;
		}

		if (ret) {
			dev_err(&sipc->dev, "failed to initialize channel %d: %d\n", id, ret);
			sipc_free_channel(channel);
			continue;
		}

		channel = xa_store(&sipc->channels, id, channel, GFP_KERNEL);
		if (channel) {
			dev_warn(&sipc->dev, "duplicate channel ID %d\n", id);
			sipc_free_channel(channel);
		}
	}

	return 0;
}

static void sipc_depopulate(struct sprd_sipc *sipc)
{
	struct sipc_channel *channel;
	unsigned long id;

	xa_for_each_start(&sipc->channels, id, channel, 0) {
		if (!test_and_set_bit(SIPC_FORCE_CLOSE, &channel->state))
			queue_work(sipc->wq, &channel->state_work);
	}

	flush_workqueue(sipc->wq);

	xa_for_each_start(&sipc->channels, id, channel, 0) {
		if (channel->ops)
			channel->ops->free(channel);
	}

	xa_destroy(&sipc->channels);

	if (sipc->sipx) {
		sipx_destroy(sipc->sipx);
		sipc->sipx = NULL;
	}
}

static void sprd_sipc_release(struct device *dev)
{
	struct sprd_sipc *sipc = container_of(dev, struct sprd_sipc, dev);

	WARN_ON(!xa_empty(&sipc->channels));
	WARN_ON(!list_empty(&sipc->queue));

	mbox_free_channel(sipc->mbox_chan);
	mbox_free_channel(sipc->mbox_oob_chan);

	if (sipc->smem_pool)
		gen_pool_destroy(sipc->smem_pool);
	if (sipc->smem_virt)
		memunmap(sipc->smem_virt);
	destroy_workqueue(sipc->wq);
	kfree(sipc);
}

static ssize_t rpmsg_name_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	const char *name;

	ret = of_property_read_string(dev->of_node, "label", &name);
	if (ret < 0)
		name = dev->of_node->name;

	return sysfs_emit(buf, "%s\n", name);
}
static DEVICE_ATTR_RO(rpmsg_name);

static struct attribute *sprd_sipc_attrs[] = {
	&dev_attr_rpmsg_name.attr,
	NULL
};
ATTRIBUTE_GROUPS(sprd_sipc);

struct sprd_sipc *sprd_sipc_register(struct device *parent, struct device_node *node)
{
	struct sprd_sipc *sipc;
	struct device *dev;
	int ret, count;

	sipc = kzalloc_obj(*sipc, GFP_KERNEL);
	if (!sipc)
		return ERR_PTR(-ENOMEM);

	spin_lock_init(&sipc->queue_lock);
	xa_init(&sipc->channels);
	INIT_LIST_HEAD(&sipc->queue);

	sipc->wq = alloc_ordered_workqueue("%s-%pOFn", 0, dev_name(parent), node);
	if (!sipc->wq) {
		kfree(sipc);
		return ERR_PTR(-ENOMEM);
	}

	dev = &sipc->dev;
	dev->parent = parent;
	dev->release = sprd_sipc_release;
	dev->of_node = node;
	dev->groups = sprd_sipc_groups;
	dev->coherent_dma_mask = DMA_BIT_MASK(32);
	dev->dma_mask = &dev->coherent_dma_mask;
	dev_set_name(dev, "%s:%pOFn", dev_name(parent), node);

	ret = device_register(dev);
	if (ret) {
		dev_err(dev, "failed to register\n");
		put_device(dev);
		return ERR_PTR(ret);
	}

	ret = of_dma_configure(dev, node->parent, true);
	if (ret) {
		dev_err(dev, "failed to initialize address mapping: %d\n", ret);
		goto unregister;
	}

	ret = sipc_smem_init(sipc);
	if (ret) {
		dev_err(dev, "failed to initialize shared memory: %d\n", ret);
		goto unregister;
	}

	sipc->mbox_client.dev = dev;
	sipc->mbox_client.rx_callback = sipc_rx_callback;
	sipc->mbox_client.tx_done = sipc_tx_done;

	count = of_count_phandle_with_args(node, "mboxes", "#mbox-cells");
	if (count <= 0) {
		dev_err(dev, "no mailboxes specified\n");
		ret = count ?: -ENOENT;
		goto unregister;
	}

	if (count >= 2) {
		sipc->mbox_oob_chan = mbox_request_channel(&sipc->mbox_client, 1);
		if (IS_ERR(sipc->mbox_oob_chan)) {
			ret = PTR_ERR(sipc->mbox_oob_chan);
			sipc->mbox_oob_chan = NULL;
			dev_err(dev, "failed to request supplementary mailbox: %d\n", ret);
			goto unregister;
		}
	}

	sipc->mbox_chan = mbox_request_channel(&sipc->mbox_client, 0);
	if (IS_ERR(sipc->mbox_chan)) {
		ret = PTR_ERR(sipc->mbox_chan);
		sipc->mbox_chan = NULL;
		dev_err(dev, "failed to request mailbox: %d\n", ret);
		goto unregister;
	}

	ret = sipc_populate(sipc);
	if (ret) {
		dev_err(dev, "failed to populate channels: %d\n", ret);
		goto depopulate;
	}

	return sipc;

depopulate:
	sipc_depopulate(sipc);
unregister:
	device_unregister(dev);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(sprd_sipc_register);

void sprd_sipc_unregister(struct sprd_sipc *sipc)
{
	sipc_depopulate(sipc);
	device_unregister(&sipc->dev);
}
EXPORT_SYMBOL_GPL(sprd_sipc_unregister);

MODULE_DESCRIPTION("Spreadtrum IPC driver");
MODULE_LICENSE("GPL");
