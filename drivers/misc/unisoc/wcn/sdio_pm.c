// SPDX-License-Identifier: GPL-2.0-only
/* Marlin3 CP sleep and SDIO power management. */

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/mmc/host.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/pm_wakeup.h>

#include "core.h"
#include "sdio.h"

/* Caller holds sleep_lock; F0 remains accessible while CP2 is asleep. */
static int wcn_allow_sleep(struct wcn_core *core)
{
	int ret = 0;

	/* A failed acknowledgment can still leave CP2 asleep. */
	core->cp_sleeping = true;
	sdio_claim_host(core->func);
	sdio_f0_writeb(core->func, 1, core->chip->sdio.sleep_ctl, &ret);
	if (!ret)
		sdio_f0_writeb(core->func, WCN_AP_ALLOW_SLEEP, core->chip->sdio.ap_int,
			       &ret);
	if (!ret)
		/* BSP requires SLP_CTL high for two 32 kHz cycles. */
		udelay(65);
	sdio_release_host(core->func);
	return ret;
}

static void wcn_sleep_work(struct work_struct *work)
{
	struct wcn_core *core = container_of(work, struct wcn_core, sleep_work);
	int ret;

	mutex_lock(&core->sleep_lock);
	if (!core->sleep_enabled || !core->sleep_requested || core->io_users ||
	    !READ_ONCE(core->fw_loaded) || READ_ONCE(core->removing) ||
	    core->system_suspended)
		goto out;
	core->sleep_requested = false;
	ret = wcn_allow_sleep(core);
	if (ret)
		dev_err_ratelimited(core->dev, "CP sleep acknowledgment failed: %d\n",
				    ret);
out:
	mutex_unlock(&core->sleep_lock);
}

void wcn_pm_init(struct wcn_core *core)
{
	mutex_init(&core->sleep_lock);
	INIT_WORK(&core->sleep_work, wcn_sleep_work);
}

void wcn_pm_disable(struct wcn_core *core)
{
	mutex_lock(&core->sleep_lock);
	core->sleep_enabled = false;
	mutex_unlock(&core->sleep_lock);
	cancel_work_sync(&core->sleep_work);
}

void wcn_pm_enable(struct wcn_core *core)
{
	mutex_lock(&core->sleep_lock);
	core->sleep_enabled = true;
	if (core->sleep_requested && !core->io_users)
		schedule_work(&core->sleep_work);
	mutex_unlock(&core->sleep_lock);
}

/* Only after a hardware reset/power cycle or a successful CP stop. */
void wcn_pm_reset(struct wcn_core *core)
{
	mutex_lock(&core->sleep_lock);
	core->cp_sleeping = false;
	core->sleep_requested = false;
	mutex_unlock(&core->sleep_lock);
}

int wcn_pm_get(struct wcn_core *core)
{
	int ret = 0;

	mutex_lock(&core->sleep_lock);
	if (core->system_suspended) {
		ret = -EHOSTDOWN;
		goto out;
	}
	if (core->cp_sleeping) {
		ret = wcn_sdio_wake(core);
		if (ret)
			goto out;
		core->cp_sleeping = false;
	}
	core->io_users++;
out:
	mutex_unlock(&core->sleep_lock);
	return ret;
}

void wcn_pm_put(struct wcn_core *core)
{
	mutex_lock(&core->sleep_lock);
	if (WARN_ON(!core->io_users))
		goto out;
	core->io_users--;
	if (!core->io_users && core->sleep_enabled && core->sleep_requested)
		schedule_work(&core->sleep_work);
out:
	mutex_unlock(&core->sleep_lock);
}

void wcn_pm_request_sleep(struct wcn_core *core)
{
	mutex_lock(&core->sleep_lock);
	core->sleep_requested = true;
	if (core->sleep_enabled && !core->io_users)
		schedule_work(&core->sleep_work);
	mutex_unlock(&core->sleep_lock);
}

static void wcn_resume_irqs(struct wcn_core *core)
{
	if (core->data_was_enabled)
		wcn_sdio_irq_enable(core);
	if (core->pub_was_enabled) {
		mutex_lock(&core->irq_lock);
		enable_irq(core->pub_irq);
		core->pub_irq_enabled = true;
		mutex_unlock(&core->irq_lock);
	}
}

static int wcn_system_suspend(struct device *dev)
{
	struct wcn_core *core = dev_get_drvdata(dev);
	bool keep_power;
	int ret, wake_ret;

	mutex_lock(&core->power_lock);
	if (!core->fw_loaded || READ_ONCE(core->removing)) {
		ret = 0;
		goto out;
	}
	if (core->gnss_prepared && !core->gnss_running) {
		ret = -EBUSY;
		goto out;
	}
	ret = READ_ONCE(core->pub_error);
	if (ret)
		goto out;
	keep_power = core->func->card->host->pm_flags & MMC_PM_KEEP_POWER;
	wcn_pm_disable(core);
	mutex_lock(&core->irq_lock);
	core->data_was_enabled = core->irq_enabled;
	core->pub_was_enabled = core->pub_irq_enabled;
	mutex_unlock(&core->irq_lock);
	wcn_sdio_irq_disable(core);
	wcn_sdio_pub_irq_disable(core);
	cancel_work_sync(&core->rx_work);
	cancel_work_sync(&core->pub_fault_work);

	mutex_lock(&core->io_lock);
	mutex_lock(&core->sleep_lock);
	if (core->io_users) {
		ret = -EBUSY;
	} else {
		WRITE_ONCE(core->system_suspended, true);
		ret = wcn_allow_sleep(core);
	}
	mutex_unlock(&core->sleep_lock);
	mutex_unlock(&core->io_lock);
	if (ret)
		goto rollback;

	ret = sdio_set_host_pm_flags(core->func, MMC_PM_KEEP_POWER);
	if (ret)
		goto rollback;
	if (device_may_wakeup(dev) && core->data_was_enabled) {
		ret = enable_irq_wake(core->data_irq);
		if (ret)
			goto rollback;
		core->wake_armed = true;
		/* The primary handler records wakeup without touching SDIO. */
		wcn_sdio_irq_enable(core);
	}
	dev_dbg(dev, "CP2 suspended, SDIO power retained\n");
	goto out;

rollback:
	mutex_lock(&core->sleep_lock);
	wake_ret = core->cp_sleeping ? wcn_sdio_wake(core) : 0;
	if (!wake_ret)
		core->cp_sleeping = false;
	WRITE_ONCE(core->system_suspended, false);
	mutex_unlock(&core->sleep_lock);
	if (!keep_power)
		core->func->card->host->pm_flags &= ~MMC_PM_KEEP_POWER;
	if (wake_ret) {
		WRITE_ONCE(core->fw_loaded, false);
		core->tx_ready = false;
		dev_err(dev, "CP2 suspend rollback failed: %d\n", wake_ret);
	} else {
		wcn_resume_irqs(core);
		wcn_pm_enable(core);
	}
out:
	mutex_unlock(&core->power_lock);
	return ret;
}

static int wcn_system_resume(struct device *dev)
{
	struct wcn_core *core = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&core->power_lock);
	if (!core->system_suspended)
		goto out;
	wcn_sdio_irq_disable(core);
	/* Balance the primary handler's one-shot wakeup mask, if it ran. */
	if (test_and_clear_bit(0, &core->wake_pending))
		enable_irq(core->data_irq);
	if (core->wake_armed) {
		disable_irq_wake(core->data_irq);
		core->wake_armed = false;
	}
	mutex_lock(&core->io_lock);
	mutex_lock(&core->sleep_lock);
	ret = wcn_sdio_wake(core);
	if (!ret)
		core->cp_sleeping = false;
	WRITE_ONCE(core->system_suspended, false);
	mutex_unlock(&core->sleep_lock);
	if (ret) {
		WRITE_ONCE(core->fw_loaded, false);
		core->tx_ready = false;
		dev_err(dev, "CP2 resume failed: %d\n", ret);
	}
	mutex_unlock(&core->io_lock);
	if (!ret) {
		wcn_resume_irqs(core);
		wcn_pm_enable(core);
	}
out:
	mutex_unlock(&core->power_lock);
	return ret;
}

static int wcn_runtime_suspend(struct device *dev)
{
	struct wcn_core *core = dev_get_drvdata(dev);
	int ret;

	if (!core)
		return 0;
	if (READ_ONCE(core->power_users) || READ_ONCE(core->fw_loaded))
		return -EBUSY;
	sdio_claim_host(core->func);
	ret = sdio_disable_func(core->func);
	sdio_release_host(core->func);
	if (ret) {
		WRITE_ONCE(core->power_error, ret);
		dev_err(dev, "failed to disable SDIO function: %d\n", ret);
		/* Keep runtime PM retryable instead of latching a fatal error. */
		return -EBUSY;
	}
	/* The MMC card's runtime callback now cuts power through pwrseq. */
	return 0;
}

static int wcn_runtime_resume(struct device *dev)
{
	struct wcn_core *core = dev_get_drvdata(dev);
	int ret;

	if (!core)
		return 0;
	ret = wcn_sdio_configure(core);
	if (ret) {
		WRITE_ONCE(core->power_error, ret);
		dev_err(dev, "failed to restore SDIO function: %d\n", ret);
		return -EAGAIN;
	}
	return 0;
}

const struct dev_pm_ops wcn_sdio_pm_ops = {
	SYSTEM_SLEEP_PM_OPS(wcn_system_suspend, wcn_system_resume)
	RUNTIME_PM_OPS(wcn_runtime_suspend, wcn_runtime_resume, NULL)
};
