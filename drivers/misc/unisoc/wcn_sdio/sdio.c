// SPDX-License-Identifier: GPL-2.0
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/mmc/card.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/unaligned.h>

#include "wcn.h"
#include "regs.h"

static int wcn_sdio_dt_set_addr(struct wcn_core *core, u32 addr)
{
	u8 address[4];
	int err = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(address); i++)
		address[i] = (addr >> (8 * i)) & 0xff;

	for (i = 0; i < ARRAY_SIZE(address); i++) {
		sdio_f0_writeb(core->func, address[i],
			       WCN_SDIO_FBR_SYSADDR0 + i, &err);
		if (err)
			break;
	}

	return err;
}

int wcn_sdio_enable_slave_irq(struct wcn_core *core)
{
	u8 val;
	int err = 0;

	sdio_claim_host(core->func);
	val = sdio_f0_readb(core->func, WCN_SDIO_FBR_DEINT_EN, &err);
	if (!err)
		sdio_f0_writeb(core->func, val | WCN_SDIO_DEINT_ENABLE,
			       WCN_SDIO_FBR_DEINT_EN, &err);
	sdio_release_host(core->func);

	return err;
}

static void wcn_sdio_abort(struct wcn_core *core)
{
	int err = 0;

	sdio_claim_host(core->func);
	sdio_f0_writeb(core->func, WCN_SDIO_CCCR_ABORT_TRANS,
		       WCN_SDIO_CCCR_ABORT, &err);
	sdio_release_host(core->func);

	if (err)
		dev_err_ratelimited(core->dev,
				    "failed to abort SDIO transfer: %d\n", err);
}

static int wcn_sdio_cp_wake_locked(struct wcn_core *core)
{
	unsigned int i;
	u8 status;
	int err = 0;

	sdio_f0_writeb(core->func, 0, WCN_CP_SLP_CTL_REG, &err);
	if (err)
		return err;

	for (i = 0; i < 100; i++) {
		status = sdio_f0_readb(core->func, WCN_BTWF_SLP_STS_REG,
				       &err);
		if (err)
			return err;

		status &= 0xf0;
		if (status != WCN_BTWF_IN_DEEPSLEEP &&
		    status != WCN_BTWF_XLT_WAIT &&
		    status != WCN_BTWF_XLTBUF_WAIT &&
		    status != WCN_BTWF_PLL_PWR_WAIT &&
		    status != WCN_BTWF_IN_DEEPSLEEP_XTL_ON)
			return 0;

		udelay(50);
	}

	dev_warn_ratelimited(core->dev,
			     "WCN CP wake timeout, status 0x%02x\n", status);
	return -ETIMEDOUT;
}

int wcn_sdio_cp_init(struct wcn_core *core)
{
	__le32 *pin;
	int ret;

	pin = kmalloc_obj(*pin);
	if (!pin)
		return -ENOMEM;

	ret = wcn_sdio_dt_read(core, WCN_CP_GPIO1_REG, pin, sizeof(*pin));
	if (!ret && (le32_to_cpu(*pin) & WCN_CP_PIN_FUNC_WPU)) {
		*pin = cpu_to_le32(le32_to_cpu(*pin) & ~WCN_CP_PIN_FUNC_WPU);
		ret = wcn_sdio_dt_write(core, WCN_CP_GPIO1_REG, pin, sizeof(*pin));
	}
	kfree(pin);
	if (ret)
		return dev_err_probe(core->dev, ret, "failed to configure CP data pin\n");

	ret = wcn_sdio_enable_slave_irq(core);
	if (ret)
		return dev_err_probe(core->dev, ret, "failed to enable CP slave IRQ\n");

	return 0;
}

void wcn_sdio_irq_enable(struct wcn_core *core)
{
	mutex_lock(&core->irq_lock);
	if (core->data_irq && !core->irq_enabled) {
		enable_irq(core->data_irq);
		core->irq_enabled = true;
	}
	mutex_unlock(&core->irq_lock);
}

void wcn_sdio_irq_disable(struct wcn_core *core)
{
	mutex_lock(&core->irq_lock);
	if (core->data_irq && core->irq_enabled) {
		disable_irq(core->data_irq);
		core->irq_enabled = false;
	}
	mutex_unlock(&core->irq_lock);
}

int wcn_sdio_dt_write(struct wcn_core *core, u32 addr,
		      const void *buf, size_t len)
{
	const u8 *p = buf;
	unsigned int blksz;
	int ret = 0;

	sdio_claim_host(core->func);
	if (core->fw_loaded) {
		ret = wcn_sdio_cp_wake_locked(core);
		if (ret)
			goto out;
	}

	ret = wcn_sdio_dt_set_addr(core, addr);
	if (ret)
		goto out;

	blksz = core->func->cur_blksize ?: WCN_BLK_SIZE;
	while (len) {
		size_t chunk;

		if (len >= blksz)
			chunk = blksz;
		else
			chunk = min_t(size_t, len, 512);

		ret = sdio_memcpy_toio(core->func, WCN_SDIO_DT_MODE_ADDR,
				       (void *)p, chunk);
		if (ret)
			break;
		p += chunk;
		len -= chunk;
	}
out:
	sdio_release_host(core->func);
	if (ret)
		wcn_sdio_abort(core);
	return ret;
}

int wcn_sdio_dt_read(struct wcn_core *core, u32 addr, void *buf, size_t len)
{
	u8 *p = buf;
	unsigned int blksz;
	int ret = 0;

	sdio_claim_host(core->func);
	if (core->fw_loaded) {
		ret = wcn_sdio_cp_wake_locked(core);
		if (ret)
			goto out;
	}

	ret = wcn_sdio_dt_set_addr(core, addr);
	if (ret)
		goto out;

	blksz = core->func->cur_blksize ?: WCN_BLK_SIZE;
	while (len) {
		size_t chunk;

		if (len >= blksz)
			chunk = blksz;
		else
			chunk = min_t(size_t, len, 512);

		ret = sdio_memcpy_fromio(core->func, p,
					 WCN_SDIO_DT_MODE_ADDR, chunk);
		if (ret)
			break;
		p += chunk;
		len -= chunk;
	}
out:
	sdio_release_host(core->func);
	if (ret)
		wcn_sdio_abort(core);
	return ret;
}

int wcn_sdio_packet_write(struct wcn_core *core, const void *buf, size_t len)
{
	int ret;

	sdio_claim_host(core->func);
	if (core->fw_loaded) {
		ret = wcn_sdio_cp_wake_locked(core);
		if (ret)
			goto out;
	}

	ret = sdio_writesb(core->func, WCN_SDIO_PK_MODE_ADDR,
			   (void *)buf, len);
out:
	sdio_release_host(core->func);
	if (ret)
		wcn_sdio_abort(core);

	return ret;
}

int wcn_sdio_packet_read(struct wcn_core *core, void *buf, size_t len)
{
	int ret;

	sdio_claim_host(core->func);
	ret = sdio_readsb(core->func, buf, WCN_SDIO_PK_MODE_ADDR, len);
	sdio_release_host(core->func);
	if (ret)
		wcn_sdio_abort(core);

	return ret;
}

static void wcn_sdio_rx(struct wcn_core *core)
{
	unsigned int retry;

	for (retry = 0; retry < 64; retry++) {
		u32 valid_len, rx_dtbs;
		unsigned int off;
		int ret;

		ret = wcn_sdio_packet_read(core, core->rx_buf,
					   WCN_MAX_PAC_SIZE);
		if (ret) {
			dev_err_ratelimited(core->dev,
					    "packet read failed: %d\n", ret);
			return;
		}

		valid_len = get_unaligned_le32(core->rx_buf +
					       WCN_MAX_PAC_SIZE - 8);
		rx_dtbs = get_unaligned_le32(core->rx_buf +
					     WCN_MAX_PAC_SIZE - 4);

		if (valid_len > WCN_MAX_PAC_SIZE - 8) {
			dev_err_ratelimited(core->dev,
					    "invalid RX length %u\n", valid_len);
			return;
		}

		for (off = 0; off + sizeof(struct wcn_puh) <= valid_len;) {
			struct wcn_puh *puh;
			unsigned int plen, next, id;

			puh = (struct wcn_puh *)(core->rx_buf + off);
			if (puh->eof || !puh->len)
				break;

			plen = puh->len;
			next = plen + (puh->check_sum ? 2 : 0);
			if (next > valid_len - off - sizeof(*puh))
				break;

			if (puh->type != 0xf) {
				id = WCN_CHAN_TX_NUM + puh->subtype;
				wcn_chan_rx(core, id,
					    core->rx_buf + off + sizeof(*puh),
					    plen);
			}

			off += sizeof(*puh) + ALIGN(next, 4);
		}

		if (!rx_dtbs)
			break;
	}
}

void wcn_sdio_rx_work(struct work_struct *work)
{
	struct wcn_core *core = container_of(work, struct wcn_core, rx_work);

	wcn_sdio_rx(core);
}

static irqreturn_t wcn_sdio_data_irq(int irq, void *data)
{
	return IRQ_WAKE_THREAD;
}

static irqreturn_t wcn_sdio_data_thread(int irq, void *data)
{
	struct wcn_core *core = data;

	wcn_sdio_rx(core);

	return IRQ_HANDLED;
}

int wcn_core_start(struct wcn_core *core)
{
	int ret;

	core->rx_buf = kzalloc(WCN_RX_BUF_SIZE, GFP_KERNEL);
	if (!core->rx_buf)
		return -ENOMEM;

	ret = wcn_sdio_enable_slave_irq(core);
	if (ret) {
		dev_err(core->dev, "failed to enable slave IRQ: %d\n", ret);
		goto err_buf;
	}

	/*
	 * Prefer the explicit GPIO: requesting it through gpiolib puts
	 * the pad into digital input mode (DMSK/DIR/INEN), which the
	 * sprd gpiochip does not do for interrupt-only mappings. Without
	 * this the data IRQ never fires if the bootloader left the pin
	 * configured differently (BSP does gpio_request +
	 * gpio_direction_input before gpio_to_irq).
	 */
	core->data_gpio = devm_gpiod_get_optional(core->dev, "data", GPIOD_IN);
	if (IS_ERR(core->data_gpio)) {
		ret = PTR_ERR(core->data_gpio);
		dev_err_probe(core->dev, ret, "failed to get data GPIO\n");
		core->data_gpio = NULL;
		goto err_buf;
	}

	if (core->data_gpio) {
		ret = gpiod_to_irq(core->data_gpio);
		if (ret < 0) {
			dev_err(core->dev, "failed to map data GPIO IRQ: %d\n", ret);
			goto err_buf;
		}
		core->data_irq = ret;
	} else {
		core->data_irq = of_irq_get_byname(core->dev->of_node, "data");
		if (core->data_irq < 0) {
			dev_err(core->dev, "failed to get data IRQ: %d\n",
				core->data_irq);
			ret = core->data_irq;
			goto err_buf;
		}
	}

	ret = request_threaded_irq(core->data_irq, wcn_sdio_data_irq,
				   wcn_sdio_data_thread,
				   IRQF_ONESHOT | IRQF_TRIGGER_HIGH |
				   IRQF_NO_SUSPEND | IRQF_NO_AUTOEN,
				   "wcn-sdio-data", core);
	if (ret)
		goto err_buf;

	core->pub_irq = of_irq_get_byname(core->dev->of_node, "pub-int");
	if (core->pub_irq < 0)
		core->pub_irq = 0;

	ret = wcn_core_assert_start(core);
	if (ret)
		goto err_irq;

	wcn_core_at_start(core);

	ret = wcn_children_create(core);
	if (ret)
		goto err_assert;

	core->started = true;
	return 0;

err_assert:
	wcn_core_at_stop(core);
	wcn_core_assert_stop(core);
err_irq:
	mutex_lock(&core->irq_lock);
	free_irq(core->data_irq, core);
	core->data_irq = 0;
	core->irq_enabled = false;
	mutex_unlock(&core->irq_lock);
err_buf:
	kfree(core->rx_buf);
	core->rx_buf = NULL;
	return ret;
}

void wcn_core_stop(struct wcn_core *core)
{
	core->started = false;
	cancel_work_sync(&core->reset_work);

	mutex_lock(&core->irq_lock);
	if (core->data_irq) {
		if (core->irq_enabled) {
			disable_irq(core->data_irq);
			core->irq_enabled = false;
		}
		free_irq(core->data_irq, core);
		core->data_irq = 0;
	}
	mutex_unlock(&core->irq_lock);

	cancel_work_sync(&core->rx_work);
	wcn_core_assert_stop(core);
	wcn_core_at_stop(core);
	wcn_children_destroy(core);

	kfree(core->rx_buf);
	core->rx_buf = NULL;
}

static int wcn_sdio_probe(struct sdio_func *func,
			  const struct sdio_device_id *id)
{
	struct wcn_core *core;
	int ret;

	if (func->num != 1)
		return -ENODEV;

	if (!func->dev.of_node) {
		dev_err(&func->dev, "missing SDIO function OF node\n");
		return -ENODEV;
	}

	if (!of_device_is_compatible(func->dev.of_node, "unisoc,marlin3lite")) {
		dev_err(&func->dev, "unsupported SDIO function compatible\n");
		return -ENODEV;
	}

	func->card->quirks |= MMC_QUIRK_LENIENT_FN0;

	core = wcn_core_alloc(&func->dev, func);
	if (!core)
		return -ENOMEM;

	sdio_set_drvdata(func, core);

	sdio_claim_host(func);
	ret = sdio_enable_func(func);
	if (!ret) {
		ret = sdio_set_block_size(func, WCN_BLK_SIZE);
		if (!ret)
			func->max_blksize = WCN_BLK_SIZE;
	}
	sdio_release_host(func);
	if (ret) {
		dev_err_probe(&func->dev, ret,
			      "failed to enable SDIO function\n");
		goto err_free;
	}

	ret = wcn_core_start(core);
	if (ret) {
		dev_err_probe(&func->dev, ret, "failed to start WCN core\n");
		goto err_disable;
	}

	dev_info(&func->dev, "Unisoc WCN SDIO core ready\n");
	return 0;

err_disable:
	sdio_claim_host(func);
	sdio_disable_func(func);
	sdio_release_host(func);
err_free:
	sdio_set_drvdata(func, NULL);
	wcn_core_free(core);
	return ret;
}

static void wcn_sdio_remove(struct sdio_func *func)
{
	struct wcn_core *core = sdio_get_drvdata(func);

	if (!core)
		return;

	wcn_core_stop(core);

	sdio_claim_host(func);
	sdio_disable_func(func);
	sdio_release_host(func);

	sdio_set_drvdata(func, NULL);
	wcn_core_free(core);
}

static const struct sdio_device_id wcn_sdio_ids[] = {
	{ SDIO_DEVICE(0x0000, 0x0000) },
	{ }
};
MODULE_DEVICE_TABLE(sdio, wcn_sdio_ids);

static struct sdio_driver wcn_sdio_driver = {
	.probe = wcn_sdio_probe,
	.remove = wcn_sdio_remove,
	.name = "unisoc_wcn_sdio",
	.id_table = wcn_sdio_ids,
};
module_sdio_driver(wcn_sdio_driver);

MODULE_DESCRIPTION("Unisoc SC2355 SDIO WCN transport core");
MODULE_LICENSE("GPL");
