// SPDX-License-Identifier: GPL-2.0
/*
 * This file is a implementation for wcn sdio transport
 *
 * Copyright (C) 2026 Veynamer <veyr1thalone@gmail.com>
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/pm_wakeup.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/unaligned.h>

#include "core.h"
#include "sdio.h"

int wcn_sdio_aon_read(struct wcn_core *core, unsigned int addr, u8 *val)
{
	int ret;

	sdio_claim_host(core->func);
	*val = sdio_f0_readb(core->func, addr, &ret);
	sdio_release_host(core->func);
	return ret;
}

int wcn_sdio_aon_write(struct wcn_core *core, unsigned int addr, u8 val)
{
	int ret;

	sdio_claim_host(core->func);
	sdio_f0_writeb(core->func, val, addr, &ret);
	sdio_release_host(core->func);
	return ret;
}

void wcn_sdio_pub_irq_disable(struct wcn_core *core)
{
	mutex_lock(&core->irq_lock);
	if (core->pub_irq && core->pub_irq_enabled) {
		disable_irq(core->pub_irq);
		core->pub_irq_enabled = false;
	}
	mutex_unlock(&core->irq_lock);
}

void wcn_sdio_pub_fault_work(struct work_struct *work)
{
	struct wcn_core *core = container_of(work, struct wcn_core, pub_fault_work);

	wcn_sdio_pub_irq_disable(core);
}

int wcn_sdio_pub_prepare(struct wcn_core *core)
{
	u8 mask;
	unsigned int i;
	int ret;

	wcn_sdio_pub_irq_disable(core);
	cancel_work_sync(&core->pub_fault_work);
	WRITE_ONCE(core->pub_error, 0);
	atomic_set(&core->pub_events, 0);
	reinit_completion(&core->mem_save);
	for (i = 0; i < ARRAY_SIZE(core->subsys_open); i++)
		reinit_completion(&core->subsys_open[i]);

	ret = wcn_sdio_aon_write(core, core->chip->sdio.pub_int_en, 0);
	if (!ret)
		ret = wcn_sdio_aon_write(core, core->chip->sdio.pub_int_clr, WCN_PUB_BTWF_MASK);
	if (!ret)
		ret = wcn_sdio_aon_write(core, core->chip->sdio.sleep_ctl, 0);
	if (!ret)
		ret = wcn_sdio_aon_write(core, core->chip->sdio.pub_int_en, WCN_PUB_BTWF_MASK);
	if (!ret)
		ret = wcn_sdio_aon_read(core, core->chip->sdio.pub_int_en, &mask);
	if (ret)
		return ret;
	if ((mask & WCN_PUB_BTWF_MASK) != WCN_PUB_BTWF_MASK)
		return -EIO;

	mutex_lock(&core->irq_lock);
	enable_irq(core->pub_irq);
	core->pub_irq_enabled = true;
	mutex_unlock(&core->irq_lock);
	dev_info(core->dev, "PUB_INT IRQ %d enabled before CP2 release\n",
		 core->pub_irq);
	return 0;
}

static irqreturn_t wcn_sdio_pub_thread(int irq, void *data)
{
	struct wcn_core *core = data;
	u8 status;
	unsigned int i;
	int ret;

	ret = wcn_sdio_aon_read(core, core->chip->sdio.pub_int_status, &status);
	if (ret)
		goto fault;
	status &= WCN_PUB_BTWF_MASK;
	if (!status) {
		/* Bound retries if the GPIO is high without a CP event. */
		msleep(20);
		return IRQ_HANDLED;
	}
	/* Publish observation before clearing the CP status for boot timeout checks. */
	atomic_or(status, &core->pub_events);
	ret = wcn_sdio_aon_write(core, core->chip->sdio.pub_int_clr, status);
	if (ret)
		goto fault;

	dev_dbg(core->dev, "PUB_INT events %#02x\n", status);
	if (status & WCN_PUB_MEM_SAVE)
		complete(&core->mem_save);
	if (status & WCN_PUB_WIFI_OPEN)
		complete(&core->subsys_open[WCN_CHILD_WLAN]);
	if (status & WCN_PUB_BT_OPEN)
		complete(&core->subsys_open[WCN_CHILD_BT]);
	if (status & WCN_PUB_REQ_SLEEP)
		wcn_pm_request_sleep(core);
	return IRQ_HANDLED;

fault:
	if (!cmpxchg(&core->pub_error, 0, ret)) {
		dev_err(core->dev, "PUB_INT access failed: %d\n", ret);
		complete_all(&core->mem_save);
		for (i = 0; i < ARRAY_SIZE(core->subsys_open); i++)
			complete_all(&core->subsys_open[i]);
		schedule_work(&core->pub_fault_work);
	}
	/* Let the worker mask this level IRQ without spinning on a bus fault. */
	msleep(20);
	return IRQ_HANDLED;
}

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

	sdio_f0_writeb(core->func, 0, core->chip->sdio.sleep_ctl, &err);
	if (err)
		return err;

	for (i = 0; i < 100; i++) {
		status = sdio_f0_readb(core->func, core->chip->sdio.sleep_status,
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

int wcn_sdio_wake(struct wcn_core *core)
{
	int ret;

	sdio_claim_host(core->func);
	ret = wcn_sdio_cp_wake_locked(core);
	sdio_release_host(core->func);
	return ret;
}

int wcn_sdio_configure(struct wcn_core *core)
{
	int ret;

	sdio_claim_host(core->func);
	ret = sdio_enable_func(core->func);
	if (!ret)
		ret = sdio_set_block_size(core->func, core->chip->sdio.block_size);
	if (!ret)
		core->func->max_blksize = core->chip->sdio.block_size;
	sdio_release_host(core->func);
	return ret ?: wcn_sdio_enable_slave_irq(core);
}

int wcn_sdio_cp_init(struct wcn_core *core)
{
	__le32 *pin;
	int ret;

	pin = kmalloc_obj(*pin);
	if (!pin)
		return -ENOMEM;

	ret = wcn_sdio_dt_read(core, core->chip->data_pin_reg, pin, sizeof(*pin));
	if (!ret && (le32_to_cpu(*pin) & core->chip->data_pin_mask)) {
		*pin = cpu_to_le32(le32_to_cpu(*pin) & ~core->chip->data_pin_mask);
		ret = wcn_sdio_dt_write(core, core->chip->data_pin_reg, pin, sizeof(*pin));
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
		dev_info(core->dev, "data IRQ %d enabled, GPIO level %d\n",
			 core->data_irq, gpiod_get_value_cansleep(core->data_gpio));
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

	ret = wcn_pm_get(core);
	if (ret)
		return ret;
	sdio_claim_host(core->func);

	ret = wcn_sdio_dt_set_addr(core, addr);
	if (ret)
		goto out;

	blksz = core->func->cur_blksize ?: core->chip->sdio.block_size;
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
	wcn_pm_put(core);
	return ret;
}

int wcn_sdio_dt_read(struct wcn_core *core, u32 addr, void *buf, size_t len)
{
	u8 *p = buf;
	unsigned int blksz;
	int ret = 0;

	ret = wcn_pm_get(core);
	if (ret)
		return ret;
	sdio_claim_host(core->func);

	ret = wcn_sdio_dt_set_addr(core, addr);
	if (ret)
		goto out;

	blksz = core->func->cur_blksize ?: core->chip->sdio.block_size;
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
	wcn_pm_put(core);
	return ret;
}

int wcn_sdio_packet_write(struct wcn_core *core, const void *buf, size_t len)
{
	int ret;

	mutex_lock(&core->io_lock);
	if (!core->tx_ready || READ_ONCE(core->removing)) {
		ret = -ESHUTDOWN;
		goto unlock;
	}
	ret = wcn_pm_get(core);
	if (ret)
		goto unlock;
	sdio_claim_host(core->func);

	ret = sdio_writesb(core->func, WCN_SDIO_PK_MODE_ADDR,
			   (void *)buf, len);
	sdio_release_host(core->func);
	if (ret)
		wcn_sdio_abort(core);
	wcn_pm_put(core);

unlock:
	mutex_unlock(&core->io_lock);
	return ret;
}

int wcn_sdio_packet_read(struct wcn_core *core, void *buf, size_t len)
{
	int ret;

	mutex_lock(&core->io_lock);
	if (!core->fw_loaded || READ_ONCE(core->removing)) {
		ret = -ESHUTDOWN;
		goto unlock;
	}
	ret = wcn_pm_get(core);
	if (ret)
		goto unlock;
	sdio_claim_host(core->func);
	ret = sdio_readsb(core->func, buf, WCN_SDIO_PK_MODE_ADDR, len);
	sdio_release_host(core->func);
	if (ret)
		wcn_sdio_abort(core);
	wcn_pm_put(core);

unlock:
	mutex_unlock(&core->io_lock);
	return ret;
}

size_t wcn_sdio_rx_size(struct wcn_core *core, u32 dtbs)
{
	u64 len;

	if (core->rx_packet_mode) {
		unsigned int slots = clamp_t(u32, dtbs, 1, wcn_sdio_max_rx_slots(core));

		/* BSP ADMA: fixed packet slots followed by one DTBS block. */
		return slots * core->chip->sdio.packet_size + core->chip->sdio.block_size;
	}
	if (!dtbs)
		return core->chip->sdio.packet_size;

	/* BSP SDMA: DTBS counts bytes instead of packet slots. */
	len = (u64)dtbs + 8 * ((dtbs >> 10) + 1) + 64;
	return min_t(u64, roundup(len, core->chip->sdio.block_size),
		     rounddown(WCN_RX_BUF_SIZE, core->chip->sdio.block_size));
}

static int wcn_sdio_rx_parse_slots(struct wcn_core *core, size_t read_len)
{
	size_t off, parsed = 0, slots_len;
	u32 valid_len;

	if (read_len < core->chip->sdio.packet_size + core->chip->sdio.block_size ||
	    read_len > WCN_RX_BUF_SIZE)
		return -EPROTO;
	slots_len = read_len - core->chip->sdio.block_size;
	if (slots_len % core->chip->sdio.packet_size)
		return -EPROTO;
	valid_len = get_unaligned_le32(core->rx_buf + read_len - 8);
	if (valid_len > slots_len / core->chip->sdio.packet_size *
	    (core->chip->sdio.packet_size - sizeof(struct wcn_puh)))
		return -EPROTO;

	for (off = 0; off < slots_len && parsed < valid_len;
	     off += core->chip->sdio.packet_size) {
		struct wcn_puh *puh = (void *)(core->rx_buf + off);
		size_t plen = puh->len;

		/* An unused slot does not terminate the following packet slots. */
		if (puh->eof || puh->type == 0xf)
			continue;
		if (!plen || plen > valid_len - parsed ||
		    plen + (puh->check_sum ? 2 : 0) >
		    core->chip->sdio.packet_size - sizeof(*puh))
			return -EPROTO;
		parsed += plen;
		wcn_chan_rx(core, WCN_CHAN_TX_NUM + puh->subtype,
			    core->rx_buf + off + sizeof(*puh), plen);
	}

	return parsed == valid_len ? 0 : -EPROTO;
}

static int wcn_sdio_rx_parse(struct wcn_core *core, size_t read_len)
{
	u32 valid_len;
	size_t off = 0, parsed = 0, limit = read_len - 8;

	if (core->rx_packet_mode)
		return wcn_sdio_rx_parse_slots(core, read_len);

	valid_len = get_unaligned_le32(core->rx_buf + limit);
	if (valid_len > limit)
		return -EPROTO;

	/* BSP counts payload bytes, excluding PUH, checksum and alignment. */
	while (parsed < valid_len) {
		struct wcn_puh *puh;
		size_t plen, padded;
		unsigned int id;

		if (sizeof(*puh) > limit - off)
			return -EPROTO;
		puh = (struct wcn_puh *)(core->rx_buf + off);
		if (puh->eof)
			break;

		plen = puh->len;
		padded = ALIGN(plen + (puh->check_sum ? 2 : 0), 4);
		if (padded > limit - off - sizeof(*puh))
			return -EPROTO;

		if (puh->type != 0xf) {
			if (!plen || plen > valid_len - parsed ||
			    plen + (puh->check_sum ? 2 : 0) >
			    core->chip->sdio.packet_size - sizeof(*puh))
				return -EPROTO;
			id = WCN_CHAN_TX_NUM + puh->subtype;
			parsed += plen;
			wcn_chan_rx(core, id,
				    core->rx_buf + off + sizeof(*puh), plen);
		}

		off += sizeof(*puh) + padded;
	}

	return parsed == valid_len ? 0 : -EPROTO;
}

static void wcn_sdio_rx(struct wcn_core *core)
{
	unsigned int retry, empty = 0;

	mutex_lock(&core->rx_lock);

	for (retry = 0; retry < 64; retry++) {
		size_t read_len = core->rx_size;
		u32 rx_dtbs, valid_len;
		int ret;

		ret = wcn_sdio_packet_read(core, core->rx_buf,
					   read_len);
		if (ret) {
			dev_err_ratelimited(core->dev,
					    "packet read failed: %d\n", ret);
			break;
		}

		rx_dtbs = get_unaligned_le32(core->rx_buf +
					     read_len - 4);
		valid_len = get_unaligned_le32(core->rx_buf + read_len - 8);
		dev_dbg(core->dev, "RX read %zu valid %u DTBS %u header %#08x\n",
			read_len, valid_len, rx_dtbs,
			get_unaligned_le32(core->rx_buf));
		ret = wcn_sdio_rx_parse(core, read_len);
		if (ret)
			dev_err_ratelimited(core->dev,
					    "invalid RX framing (%zu bytes)\n", read_len);

		if (!rx_dtbs) {
			core->rx_size = wcn_sdio_rx_size(core, 0);
			break;
		}

		core->rx_size = wcn_sdio_rx_size(core, rx_dtbs);

		/* A stuck DTBS must not keep a level IRQ hammering CMD53. */
		if (!valid_len || ret) {
			if (++empty >= 3) {
				dev_warn_ratelimited(core->dev,
					"RX made no progress: read %zu valid %u DTBS %u header %#08x\n",
					read_len, valid_len, rx_dtbs,
					get_unaligned_le32(core->rx_buf));
				msleep(20);
				break;
			}
		} else {
			empty = 0;
		}
	}
	mutex_unlock(&core->rx_lock);
}

void wcn_sdio_rx_work(struct work_struct *work)
{
	struct wcn_core *core = container_of(work, struct wcn_core, rx_work);

	wcn_sdio_rx(core);
}

static irqreturn_t wcn_sdio_data_irq(int irq, void *data)
{
	struct wcn_core *core = data;

	if (READ_ONCE(core->system_suspended)) {
		if (!test_and_set_bit(0, &core->wake_pending))
			disable_irq_nosync(irq);
		pm_wakeup_event(core->dev, 0);
		return IRQ_HANDLED;
	}
	return IRQ_WAKE_THREAD;
}

static irqreturn_t wcn_sdio_data_thread(int irq, void *data)
{
	struct wcn_core *core = data;

	wcn_sdio_rx(core);

	return IRQ_HANDLED;
}

int wcn_sdio_start(struct wcn_core *core)
{
	unsigned long irq_flags;
	int ret;

	ret = wcn_sdio_enable_slave_irq(core);
	if (ret) {
		dev_err(core->dev, "failed to enable slave IRQ: %d\n", ret);
		goto err_out;
	}

	/*
	 * Request the GPIO as an input before mapping its IRQ. The SPRD
	 * gpiochip needs this to configure the pad's digital input path.
	 */
	core->data_gpio = devm_gpiod_get(core->dev, "data", GPIOD_IN);
	if (IS_ERR(core->data_gpio)) {
		ret = PTR_ERR(core->data_gpio);
		dev_err_probe(core->dev, ret, "failed to get data GPIO\n");
		core->data_gpio = NULL;
		goto err_out;
	}

	ret = gpiod_to_irq(core->data_gpio);
	if (ret < 0) {
		dev_err_probe(core->dev, ret, "failed to map data GPIO IRQ\n");
		goto err_out;
	}
	core->data_irq = ret;

	irq_flags = gpiod_is_active_low(core->data_gpio) ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH;
	irq_flags |= IRQF_ONESHOT | IRQF_NO_AUTOEN;
	ret = request_threaded_irq(core->data_irq, wcn_sdio_data_irq,
				   wcn_sdio_data_thread, irq_flags, "wcn-sdio-data", core);
	if (ret)
		goto err_out;

	core->pub_gpio = devm_gpiod_get(core->dev, "pub-int", GPIOD_IN);
	if (IS_ERR(core->pub_gpio)) {
		ret = PTR_ERR(core->pub_gpio);
		dev_err_probe(core->dev, ret, "failed to get PUB_INT GPIO\n");
		core->pub_gpio = NULL;
		goto err_irq;
	}
	ret = gpiod_to_irq(core->pub_gpio);
	if (ret < 0) {
		dev_err_probe(core->dev, ret, "failed to map PUB_INT GPIO IRQ\n");
		goto err_irq;
	}
	core->pub_irq = ret;
	irq_flags = gpiod_is_active_low(core->pub_gpio) ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH;
	irq_flags |= IRQF_ONESHOT | IRQF_NO_AUTOEN;
	ret = request_threaded_irq(core->pub_irq, NULL, wcn_sdio_pub_thread,
				   irq_flags, "wcn-pub-int", core);
	if (ret) {
		core->pub_irq = 0;
		goto err_irq;
	}

	return 0;

err_irq:
	mutex_lock(&core->irq_lock);
	free_irq(core->data_irq, core);
	core->data_irq = 0;
	core->irq_enabled = false;
	mutex_unlock(&core->irq_lock);
err_out:
	return ret;
}

void wcn_sdio_stop(struct wcn_core *core)
{
	wcn_sdio_pub_irq_disable(core);
	cancel_work_sync(&core->pub_fault_work);
	if (core->pub_irq) {
		free_irq(core->pub_irq, core);
		core->pub_irq = 0;
	}

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
}

static int wcn_sdio_validate_data(const struct wcn_sdio_data *sdio)
{
	if (sdio->block_size < 8 || sdio->packet_size <= sizeof(struct wcn_puh) ||
	    sdio->block_size + sdio->packet_size > WCN_RX_BUF_SIZE)
		return -EINVAL;

	return 0;
}

static int wcn_sdio_probe(struct sdio_func *func,
			  const struct sdio_device_id *id)
{
	const struct wcn_chip_data *chip;
	struct wcn_core *core;
	int ret;

	if (func->num != 1)
		return -ENODEV;

	if (!func->dev.of_node) {
		dev_err(&func->dev, "missing SDIO function OF node\n");
		return -ENODEV;
	}

	chip = device_get_match_data(&func->dev);
	if (!chip) {
		dev_err(&func->dev, "unsupported SDIO function compatible\n");
		return -ENODEV;
	}
	ret = wcn_sdio_validate_data(&chip->sdio);
	if (ret)
		return dev_err_probe(&func->dev, ret, "invalid SDIO hardware data\n");

	func->card->quirks |= MMC_QUIRK_LENIENT_FN0;

	core = wcn_core_alloc(&func->dev, func, chip);
	if (!core)
		return -ENOMEM;

	sdio_set_drvdata(func, core);

	ret = device_init_wakeup(&func->dev,
				device_property_read_bool(&func->dev, "wakeup-source"));
	if (ret)
		goto err_free;
	ret = wcn_sdio_configure(core);
	if (ret) {
		dev_err_probe(&func->dev, ret,
			      "failed to enable SDIO function\n");
		goto err_disable;
	}

	ret = wcn_core_start(core);
	if (ret) {
		dev_err_probe(&func->dev, ret, "failed to start WCN core\n");
		goto err_disable;
	}

	dev_info(&func->dev, "Unisoc WCN SDIO core ready (%s RX, %s TX)\n",
		 core->rx_packet_mode ? "packet slot" : "byte stream",
		 core->tx_packet_mode ? "packed packet" : "byte stream");
	if (func->card->host->caps & MMC_CAP_POWER_OFF_CARD)
		pm_runtime_put(&func->dev);
	return 0;

err_disable:
	sdio_claim_host(func);
	sdio_disable_func(func);
	sdio_release_host(func);
err_free:
	device_init_wakeup(&func->dev, false);
	sdio_set_drvdata(func, NULL);
	wcn_core_free(core);
	return ret;
}

static void wcn_sdio_remove(struct sdio_func *func)
{
	struct wcn_core *core = sdio_get_drvdata(func);

	if (!core)
		return;

	if (func->card->host->caps & MMC_CAP_POWER_OFF_CARD)
		pm_runtime_get_noresume(&func->dev);
	wcn_core_stop(core);

	sdio_claim_host(func);
	sdio_disable_func(func);
	sdio_release_host(func);

	sdio_set_drvdata(func, NULL);
	device_init_wakeup(&func->dev, false);
	wcn_core_free(core);
}

static const struct of_device_id wcn_sdio_of_match[] = {
	{ .compatible = "unisoc,marlin3lite", .data = &wcn_marlin3lite_data },
	{ }
};
MODULE_DEVICE_TABLE(of, wcn_sdio_of_match);

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
	.drv = {
		.of_match_table = wcn_sdio_of_match,
		.pm = pm_ptr(&wcn_sdio_pm_ops),
	},
};
module_sdio_driver(wcn_sdio_driver);

MODULE_DESCRIPTION("Unisoc Marlin3 SDIO WCN transport core");
MODULE_LICENSE("GPL");
