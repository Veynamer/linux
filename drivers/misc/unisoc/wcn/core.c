// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2026 Veynamer <veyr1thalone@gmail.com>
 */

#include <linux/devcoredump.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/mmc/card.h>
#include <linux/mmc/core.h>
#include <linux/mmc/host.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#include "core.h"
#include "sdio.h"

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
	const char *name = core->chip->fw.name;

	device_property_read_string(fwdev, "firmware-name", &name);
	return wcn_fw_load(core, name);
}

struct wcn_core *wcn_core_alloc(struct device *dev, struct sdio_func *func,
				const struct wcn_chip_data *chip)
{
	struct wcn_core *core;

	core = kzalloc_obj(*core);
	if (!core)
		return NULL;

	core->dev = dev;
	core->func = func;
	core->chip = chip;
	core->fw_base = core->chip->fw.base;
	mutex_init(&core->lock);
	mutex_init(&core->power_lock);
	mutex_init(&core->irq_lock);
	mutex_init(&core->rx_lock);
	mutex_init(&core->io_lock);
	wcn_pm_init(core);
	core->rx_packet_mode = device_property_read_bool(dev, "adma-rx");
	core->tx_packet_mode = device_property_read_bool(dev, "adma-tx");
	core->rx_size = wcn_sdio_rx_size(core, 0);
	atomic_set(&core->pub_events, 0);
	init_completion(&core->mem_save);
	init_completion(&core->subsys_open[WCN_CHILD_WLAN]);
	init_completion(&core->subsys_open[WCN_CHILD_BT]);
	INIT_WORK(&core->rx_work, wcn_sdio_rx_work);
	INIT_WORK(&core->reset_work, wcn_core_reset_work);
	INIT_WORK(&core->pub_fault_work, wcn_sdio_pub_fault_work);
	BLOCKING_INIT_NOTIFIER_HEAD(&core->reset_chain);
	atomic_set(&core->reset_pending, 0);

	return core;
}

void wcn_core_free(struct wcn_core *core)
{
	if (!core)
		return;

	cancel_work_sync(&core->reset_work);
	wcn_pm_disable(core);
	cancel_work_sync(&core->rx_work);
	cancel_work_sync(&core->pub_fault_work);
	wcn_mem_clear(core);
	kvfree(core->gnss_image);
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

	chan = wcn_chan_open(core, core->chip->assert_rx_port, &wcn_assert_ops, core);
	if (IS_ERR(chan))
		return PTR_ERR(chan);

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

	chan = wcn_chan_open(core, core->chip->at_rx_port, &wcn_at_ops, core);
	if (IS_ERR(chan)) {
		dev_warn(core->dev, "failed to open AT channel: %ld\n",
			 PTR_ERR(chan));
		core->at_chan = NULL;
		return;
	}

	core->at_chan = chan;
}

void wcn_core_at_stop(struct wcn_core *core)
{
	wcn_chan_close(core->at_chan);
	core->at_chan = NULL;
}

/* power_lock held; drain packet I/O before releasing the MMC power vote. */
static void wcn_power_off_locked(struct wcn_core *core)
{
	int ret = 0, pm_ret;

	wcn_pm_disable(core);
	mutex_lock(&core->io_lock);
	core->tx_ready = false;
	mutex_unlock(&core->io_lock);
	wcn_sdio_irq_disable(core);
	wcn_sdio_pub_irq_disable(core);
	cancel_work_sync(&core->pub_fault_work);
	cancel_work_sync(&core->rx_work);
	mutex_lock(&core->io_lock);
	if (core->fw_loaded)
		ret = wcn_fw_stop(core);
	WRITE_ONCE(core->fw_loaded, false);
	if (!ret)
		wcn_pm_reset(core);
	wcn_mem_clear(core);
	mutex_unlock(&core->io_lock);
	WRITE_ONCE(core->power_error, ret);
	if (core->power_ref) {
		core->power_ref = false;
		pm_ret = pm_runtime_put_sync(core->dev);
		if (pm_ret < 0 && !ret)
			ret = READ_ONCE(core->power_error) ?: pm_ret;
	}
	WRITE_ONCE(core->power_error, ret);
	if (ret)
		dev_err(core->dev, "WCN power off failed: %d\n", ret);
}

int wcn_power_get(struct wcn_core *core, unsigned int subsys)
{
	bool first = false;
	int ret = 0, reset_error = 0;

	if (subsys >= ARRAY_SIZE(core->subsys_users))
		return -EINVAL;
	mutex_lock(&core->power_lock);
	if (READ_ONCE(core->removing)) {
		ret = -ENODEV;
		goto out;
	}
	if (READ_ONCE(core->system_suspended)) {
		ret = -EHOSTDOWN;
		goto out;
	}
	if (!core->power_users) {
		if (core->func->card->host->caps & MMC_CAP_POWER_OFF_CARD) {
			ret = pm_runtime_resume_and_get(core->dev);
			if (ret < 0) {
				if (ret == -EAGAIN && READ_ONCE(core->power_error))
					ret = READ_ONCE(core->power_error);
				goto out;
			}
			core->power_ref = true;
		}
		first = true;
		/* A failed shutdown needs a known hardware state before retry. */
		if (READ_ONCE(core->power_error)) {
			ret = mmc_hw_reset(core->func->card);
			if (ret > 0)
				ret = -EAGAIN;
			if (!ret)
				ret = wcn_sdio_configure(core);
			if (ret) {
				reset_error = ret;
				goto out;
			}
			wcn_pm_reset(core);
			WRITE_ONCE(core->power_error, 0);
		}
	}
	if (!core->fw_loaded) {
		if (core->power_users) {
			ret = -ESHUTDOWN;
			goto out;
		}
		mutex_lock(&core->io_lock);
		ret = wcn_fw_reload_locked(core);
		WRITE_ONCE(core->fw_loaded, !ret);
		mutex_unlock(&core->io_lock);
		if (ret)
			goto out;
	}
	/* Drain boot banners before MEM_PD creates the requested CP thread. */
	wcn_sdio_irq_enable(core);
	ret = READ_ONCE(core->pub_error);
	if (!ret) {
		ret = wcn_pm_get(core);
		if (!ret) {
			if (subsys < WCN_CHILD_FM)
				ret = wcn_mem_start(core, subsys);
			if (!ret && subsys >= WCN_CHILD_FM) {
				mutex_lock(&core->io_lock);
				ret = wcn_coexist_update_locked(core, subsys, true);
				mutex_unlock(&core->io_lock);
			}
			wcn_pm_put(core);
		}
	}
	if (ret) {
		dev_err(core->dev, "failed to start %s: %d, PUB_INT events %#x\n",
			core->chip->children[subsys].name, ret,
			atomic_read(&core->pub_events));
		goto out;
	}
	core->subsys_users[subsys]++;
	core->power_users++;
	mutex_lock(&core->io_lock);
	core->tx_ready = true;
	mutex_unlock(&core->io_lock);
	wcn_pm_enable(core);
out:
	if (ret && first) {
		wcn_power_off_locked(core);
		if (reset_error)
			WRITE_ONCE(core->power_error, reset_error);
	}
	mutex_unlock(&core->power_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(wcn_power_get);

void wcn_power_put(struct wcn_core *core, unsigned int subsys)
{
	if (subsys >= ARRAY_SIZE(core->subsys_users))
		return;
	mutex_lock(&core->power_lock);
	if (core->subsys_users[subsys]) {
		core->subsys_users[subsys]--;
		core->power_users--;
		if (core->power_users && subsys >= WCN_CHILD_FM &&
		    !core->system_suspended && core->fw_loaded) {
			int ret = wcn_pm_get(core);

			if (!ret) {
				mutex_lock(&core->io_lock);
				ret = wcn_coexist_update_locked(core, subsys, false);
				mutex_unlock(&core->io_lock);
				wcn_pm_put(core);
			}
			if (ret)
				dev_warn(core->dev, "RF coexistence update failed: %d\n", ret);
		}
		if (!core->power_users)
			wcn_power_off_locked(core);
	}
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
	struct wcn_reset_info info = { .core = core };
	unsigned int i;
	bool runtime_ref = false;
	int ret, pm_ret;

	if (atomic_cmpxchg(&core->reset_pending, 0, 1))
		return;

	blocking_notifier_call_chain(&core->reset_chain, WCN_RESET_PREPARE, &info);
	mutex_lock(&core->power_lock);
	if (READ_ONCE(core->removing)) {
		ret = -ENODEV;
		goto out_power;
	}
	if (READ_ONCE(core->system_suspended)) {
		ret = -EHOSTDOWN;
		goto out_power;
	}
	if (core->func->card->host->caps & MMC_CAP_POWER_OFF_CARD) {
		ret = pm_runtime_resume_and_get(core->dev);
		if (ret < 0) {
			if (ret == -EAGAIN && READ_ONCE(core->power_error))
				ret = READ_ONCE(core->power_error);
			goto out_power;
		}
		runtime_ref = true;
	}
	wcn_pm_disable(core);
	wcn_sdio_irq_disable(core);
	wcn_sdio_pub_irq_disable(core);
	cancel_work_sync(&core->pub_fault_work);
	cancel_work_sync(&core->rx_work);

	mutex_lock(&core->io_lock);
	core->tx_ready = false;
	WRITE_ONCE(core->fw_loaded, false);
	wcn_mem_clear(core);
	ret = mmc_hw_reset(core->func->card);
	if (ret > 0) {
		info.rebind = true;
		ret = -EAGAIN;
		goto out_io;
	}
	if (ret)
		goto out_io;
	wcn_pm_reset(core);
	WRITE_ONCE(core->power_error, 0);

	sdio_claim_host(core->func);
	ret = sdio_enable_func(core->func);
	if (!ret) {
		ret = sdio_set_block_size(core->func, core->chip->sdio.block_size);
		if (!ret)
			core->func->max_blksize = core->chip->sdio.block_size;
	}
	sdio_release_host(core->func);
	if (ret || !core->power_users)
		goto out_io;

	core->rx_size = wcn_sdio_rx_size(core, 0);
	ret = wcn_fw_reload_locked(core);
	if (ret)
		goto out_io;
	WRITE_ONCE(core->fw_loaded, true);
	mutex_unlock(&core->io_lock);
	wcn_sdio_irq_enable(core);
	ret = wcn_pm_get(core);
	if (!ret) {
		for (i = 0; i < WCN_CHILD_FM; i++) {
			if (!core->subsys_users[i])
				continue;
			ret = wcn_mem_start(core, i);
			if (ret)
				break;
		}
		wcn_pm_put(core);
	}
	mutex_lock(&core->io_lock);
	if (!ret && core->gnss_prepared)
		ret = wcn_gnss_restart_locked(core);
	if (!ret && (core->subsys_users[WCN_CHILD_FM] || core->subsys_users[WCN_CHILD_GNSS]))
		ret = wcn_coexist_update_locked(core, WCN_CHILD_FM, false);
	mutex_unlock(&core->io_lock);
	if (ret) {
		wcn_sdio_irq_disable(core);
		wcn_sdio_pub_irq_disable(core);
		cancel_work_sync(&core->pub_fault_work);
	}
	mutex_lock(&core->io_lock);
	if (!ret) {
		core->tx_ready = true;
		wcn_pm_enable(core);
	} else {
		WRITE_ONCE(core->fw_loaded, false);
		if (wcn_fw_stop(core))
			dev_err(core->dev, "failed to hold CP2 after reset failure\n");
		wcn_mem_clear(core);
	}

out_io:
	mutex_unlock(&core->io_lock);
out_power:
	if (runtime_ref) {
		pm_ret = pm_runtime_put_sync(core->dev);
		if (pm_ret < 0 && !ret)
			ret = READ_ONCE(core->power_error) ?: pm_ret;
	}
	info.ready = !ret && core->tx_ready;
	mutex_unlock(&core->power_lock);
	info.error = ret;
	if (ret)
		dev_err(core->dev, "WCN reset failed: %d%s\n", ret,
			info.rebind ? " (MMC rebind required)" : "");
	blocking_notifier_call_chain(&core->reset_chain,
		ret ? WCN_RESET_FAILED : WCN_RESET_DONE, &info);
	atomic_set(&core->reset_pending, 0);
}

int wcn_core_reset(struct wcn_core *core)
{
	if (!READ_ONCE(core->started) || READ_ONCE(core->removing))
		return -ENODEV;
	if (READ_ONCE(core->system_suspended))
		return -EHOSTDOWN;

	schedule_work(&core->reset_work);
	return 0;
}
EXPORT_SYMBOL_GPL(wcn_core_reset);

int wcn_reset_notifier_register(struct wcn_core *core,
				struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&core->reset_chain, nb);
}
EXPORT_SYMBOL_GPL(wcn_reset_notifier_register);

int wcn_reset_notifier_unregister(struct wcn_core *core,
				  struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&core->reset_chain, nb);
}
EXPORT_SYMBOL_GPL(wcn_reset_notifier_unregister);

int wcn_core_start(struct wcn_core *core)
{
	int ret;

	core->rx_buf = kzalloc(WCN_RX_BUF_SIZE, GFP_KERNEL);
	if (!core->rx_buf)
		return -ENOMEM;

	ret = wcn_sdio_start(core);
	if (ret)
		goto err_buf;
	ret = wcn_core_assert_start(core);
	if (ret)
		goto err_sdio;
	wcn_core_at_start(core);
	ret = wcn_children_create(core);
	if (ret)
		goto err_assert;

	core->started = true;
	return 0;

err_assert:
	wcn_core_at_stop(core);
	wcn_core_assert_stop(core);
err_sdio:
	wcn_sdio_stop(core);
err_buf:
	kfree(core->rx_buf);
	core->rx_buf = NULL;
	return ret;
}

void wcn_core_stop(struct wcn_core *core)
{
	mutex_lock(&core->power_lock);
	WRITE_ONCE(core->removing, true);
	mutex_unlock(&core->power_lock);
	core->started = false;
	cancel_work_sync(&core->reset_work);
	wcn_pm_disable(core);
	wcn_sdio_stop(core);
	wcn_core_assert_stop(core);
	wcn_core_at_stop(core);
	wcn_children_destroy(core);

	kfree(core->rx_buf);
	core->rx_buf = NULL;
}
