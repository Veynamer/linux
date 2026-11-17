// SPDX-License-Identifier: GPL-2.0
#include <linux/devcoredump.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/mmc/card.h>
#include <linux/mmc/core.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#include "wcn.h"
#include "ports.h"
#include "regs.h"

static void wcn_core_reset_work(struct work_struct *work);
static void wcn_core_assert_rx(struct wcn_chan *chan, struct sk_buff *skb);

/*
 * The CP2 pushes status strings onto the AT channel on its own, most
 * notably the firmware version banner right after boot:
 * "WCN_VER:Platform Version:...~Project Version:...~" (BSP logs the
 * same data as "at cmd read:..."). Print everything we receive so
 * boot progress stays visible in dmesg.
 */
static void wcn_core_at_rx(struct wcn_chan *chan, struct sk_buff *skb)
{
	struct wcn_core *core = wcn_chan_get_drvdata(chan);

	dev_info(core->dev, "at cmd read:%.*s\n", (int)skb->len,
		 (char *)skb->data);

	kfree_skb(skb);
}

static int wcn_fw_reload_locked(struct wcn_core *core)
{
	struct device *fwdev = core->btwf_dev ?: core->dev;
	const char *name = "sprd/wcnmodem.bin";

	device_property_read_string(fwdev, "firmware-name", &name);
	return wcn_fw_load(core, name);
}

struct wcn_core *wcn_core_alloc(struct device *dev, struct sdio_func *func)
{
	struct wcn_core *core;

	core = kzalloc_obj(*core);
	if (!core)
		return NULL;

	core->dev = dev;
	core->func = func;
	core->fw_base = WCN_DEFAULT_FW_BASE;
	mutex_init(&core->lock);
	mutex_init(&core->power_lock);
	mutex_init(&core->irq_lock);
	INIT_WORK(&core->rx_work, wcn_sdio_rx_work);
	INIT_WORK(&core->reset_work, wcn_core_reset_work);
	ATOMIC_INIT_NOTIFIER_HEAD(&core->reset_chain);
	atomic_set(&core->reset_pending, 0);

	return core;
}

void wcn_core_free(struct wcn_core *core)
{
	if (!core)
		return;

	cancel_work_sync(&core->reset_work);
	cancel_work_sync(&core->rx_work);
	kfree(core->rx_buf);
	kfree(core);
}

static const struct wcn_chan_ops wcn_assert_ops = {
	.rx = wcn_core_assert_rx,
};

static const struct wcn_chan_ops wcn_at_ops = {
	.rx = wcn_core_at_rx,
};

int wcn_core_assert_start(struct wcn_core *core)
{
	struct wcn_chan *chan;

	chan = wcn_chan_open(core, WCN_ASSERT_RX_PORT, &wcn_assert_ops);
	if (IS_ERR(chan))
		return PTR_ERR(chan);

	wcn_chan_set_drvdata(chan, core);
	core->assert_chan = chan;
	return 0;
}

void wcn_core_assert_stop(struct wcn_core *core)
{
	wcn_chan_close(core->assert_chan);
	core->assert_chan = NULL;
}

/*
 * Non-fatal: losing the AT channel only costs us the firmware version
 * banner and other CP2 status messages.
 */
void wcn_core_at_start(struct wcn_core *core)
{
	struct wcn_chan *chan;

	chan = wcn_chan_open(core, WCN_AT_RX_PORT, &wcn_at_ops);
	if (IS_ERR(chan)) {
		dev_warn(core->dev, "failed to open AT channel: %ld\n",
			 PTR_ERR(chan));
		core->at_chan = NULL;
		return;
	}

	wcn_chan_set_drvdata(chan, core);
	core->at_chan = chan;
}

void wcn_core_at_stop(struct wcn_core *core)
{
	wcn_chan_close(core->at_chan);
	core->at_chan = NULL;
}

int wcn_power_get(struct wcn_core *core)
{
	int ret = 0;

	mutex_lock(&core->power_lock);
	if (core->power_users++ == 0 && !core->fw_loaded) {
		ret = wcn_fw_reload_locked(core);
		if (ret) {
			core->power_users--;
			goto out;
		}
		core->fw_loaded = true;
		wcn_sdio_irq_enable(core);
	}
out:
	mutex_unlock(&core->power_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(wcn_power_get);

void wcn_power_put(struct wcn_core *core)
{
	mutex_lock(&core->power_lock);
	if (core->power_users)
		core->power_users--;
	mutex_unlock(&core->power_lock);
}
EXPORT_SYMBOL_GPL(wcn_power_put);

static void wcn_core_assert_rx(struct wcn_chan *chan, struct sk_buff *skb)
{
	struct wcn_core *core = wcn_chan_get_drvdata(chan);
	void *dump;

	dev_err(core->dev, "firmware assert, %u bytes\n", skb->len);

	if (skb->len) {
		dump = vmalloc(skb->len);
		if (dump) {
			memcpy(dump, skb->data, skb->len);
			dev_coredumpv(core->dev, dump, skb->len, GFP_KERNEL);
		}
	}

	kfree_skb(skb);
	wcn_core_reset(core);
}

static void wcn_core_reset_work(struct work_struct *work)
{
	struct wcn_core *core = container_of(work, struct wcn_core, reset_work);
	struct mmc_card *card = core->func->card;
	int ret;

	if (atomic_cmpxchg(&core->reset_pending, 0, 1))
		return;

	dev_warn(core->dev, "resetting WCN chip\n");
	atomic_notifier_call_chain(&core->reset_chain, WCN_RESET_PREPARE, core);

	wcn_sdio_irq_disable(core);
	cancel_work_sync(&core->rx_work);

	ret = mmc_hw_reset(card);

	mutex_lock(&core->power_lock);
	core->fw_loaded = false;
	mutex_unlock(&core->power_lock);

	if (ret > 0) {
		dev_info(core->dev, "WCN reset scheduled by MMC rescan\n");
		goto out_irq;
	}

	if (ret) {
		dev_err(core->dev, "mmc_hw_reset failed: %d\n", ret);
		goto out_irq;
	}

	sdio_claim_host(core->func);
	ret = sdio_enable_func(core->func);
	if (!ret) {
		ret = sdio_set_block_size(core->func, WCN_BLK_SIZE);
		if (!ret)
			core->func->max_blksize = WCN_BLK_SIZE;
	}
	sdio_release_host(core->func);
	if (ret) {
		dev_err(core->dev, "failed to re-enable SDIO function: %d\n",
			ret);
		goto out_irq;
	}

	ret = wcn_sdio_enable_slave_irq(core);
	if (ret) {
		dev_err(core->dev, "failed to re-enable slave IRQ: %d\n", ret);
		goto out_irq;
	}

	mutex_lock(&core->power_lock);
	if (core->power_users) {
		ret = wcn_fw_reload_locked(core);
		if (ret)
			dev_err(core->dev, "firmware reload failed: %d\n", ret);
		else
			core->fw_loaded = true;
	}
	mutex_unlock(&core->power_lock);

out_irq:
	wcn_sdio_irq_enable(core);
	atomic_notifier_call_chain(&core->reset_chain, WCN_RESET_DONE, core);
	atomic_set(&core->reset_pending, 0);
}

int wcn_core_reset(struct wcn_core *core)
{
	if (!core->started)
		return -ENODEV;

	schedule_work(&core->reset_work);
	return 0;
}
EXPORT_SYMBOL_GPL(wcn_core_reset);

int wcn_reset_notifier_register(struct wcn_core *core,
				struct notifier_block *nb)
{
	return atomic_notifier_chain_register(&core->reset_chain, nb);
}
EXPORT_SYMBOL_GPL(wcn_reset_notifier_register);

int wcn_reset_notifier_unregister(struct wcn_core *core,
				  struct notifier_block *nb)
{
	return atomic_notifier_chain_unregister(&core->reset_chain, nb);
}
EXPORT_SYMBOL_GPL(wcn_reset_notifier_unregister);
