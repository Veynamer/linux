// SPDX-License-Identifier: GPL-2.0-only
/* Marlin3 GNSS firmware, calibration and power-domain control over SDIO. */

#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/jiffies.h>
#include <linux/limits.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/unaligned.h>
#include <linux/vmalloc.h>

#include "core.h"
#include "sdio.h"

#define WCN_GNSS_IMAGE_MAGIC	0x42544844
#define WCN_GNSS_IMAGE_HEADER	512

/* The host log dictionary contains one ASCII @<id>:<message> record per line. */
static bool wcn_gnss_log_dictionary(const u8 *data, size_t size)
{
	const u8 *end = data + size;

	if (size < 4 || memcmp(data, "@1:", 3))
		return false;
	while (data < end) {
		if (*data++ != '@' || data == end || !isdigit(*data))
			return false;
		do {
			data++;
		} while (data < end && isdigit(*data));
		if (data == end || *data++ != ':')
			return false;
		while (data < end && *data != '\n') {
			if ((*data < ' ' && *data != '\r' && *data != '\t') || *data > '~')
				return false;
			data++;
		}
		if (data == end)
			return false;
		data++;
	}
	return true;
}

static int wcn_gnss_image(struct wcn_core *core, const void *data, size_t size,
			  const void **image, size_t *image_size)
{
	const struct wcn_gnss_data *g = &core->chip->gnss;
	const u8 *bytes = data;
	u32 real, signed_size;

	if (!data || size < sizeof(u32) || !g->max_size)
		return -EINVAL;
	*image = data;
	*image_size = size;
	if (get_unaligned_le32(bytes) == WCN_GNSS_IMAGE_MAGIC) {
		if (size < WCN_GNSS_IMAGE_HEADER)
			return -EINVAL;
		real = get_unaligned_le32(bytes + 76);
		signed_size = get_unaligned_le32(bytes + 80);
		if (!real || signed_size > size || signed_size <= WCN_GNSS_IMAGE_HEADER ||
		    real >= signed_size - WCN_GNSS_IMAGE_HEADER)
			return -EINVAL;
		*image = bytes + WCN_GNSS_IMAGE_HEADER;
		*image_size = real;
	}
	if (*image_size > g->max_size) {
		/* Board files pad the CP image before appending host-only log strings. */
		if (g->log_offset <= g->max_size || *image_size < g->log_offset)
			return -EFBIG;
		bytes = *image;
		if (memchr_inv(bytes + g->max_size, 0, g->log_offset - g->max_size))
			return -EINVAL;
		if (*image_size > g->log_offset &&
		    !wcn_gnss_log_dictionary(bytes + g->log_offset,
					     *image_size - g->log_offset))
			return -EINVAL;
		*image_size = g->max_size;
	}
	if (*image_size - 1 > U32_MAX - g->base)
		return -EFBIG;
	return 0;
}

int wcn_gnss_validate_fw(struct wcn_core *core, const void *data, size_t size)
{
	const void *image;
	size_t image_size;

	return wcn_gnss_image(core, data, size, &image, &image_size);
}
EXPORT_SYMBOL_GPL(wcn_gnss_validate_fw);

static int wcn_gnss_read32(struct wcn_core *core, u32 reg, u32 *val, void *buf)
{
	int ret = wcn_sdio_dt_read(core, reg, buf, sizeof(__le32));

	if (!ret)
		*val = get_unaligned_le32(buf);
	return ret;
}

static int wcn_gnss_write32(struct wcn_core *core, u32 reg, u32 val, void *buf)
{
	put_unaligned_le32(val, buf);
	return wcn_sdio_dt_write(core, reg, buf, sizeof(__le32));
}

static int wcn_gnss_update(struct wcn_core *core, u32 reg, u32 clear, u32 set,
			   void *buf)
{
	u32 val;
	int ret = wcn_gnss_read32(core, reg, &val, buf);

	return ret ?: wcn_gnss_write32(core, reg, (val & ~clear) | set, buf);
}

static int wcn_gnss_poll(struct wcn_core *core, u32 reg, u32 mask, u32 value,
			 unsigned int timeout_ms, bool any, void *buf)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(timeout_ms);
	u32 val;
	int ret;

	do {
		ret = wcn_gnss_read32(core, reg, &val, buf);
		if (ret)
			return ret;
		if (any ? (val & mask) != 0 : (val & mask) == value)
			return 0;
		usleep_range(3000, 6000);
	} while (time_before(jiffies, timeout));
	dev_err(core->dev, "GNSS timeout at %#x: %#x\n", reg, val);
	return -ETIMEDOUT;
}

static int wcn_gnss_wait(struct wcn_core *core, u32 reg, u32 mask, u32 value,
			 unsigned int timeout_ms, void *buf)
{
	return wcn_gnss_poll(core, reg, mask, value, timeout_ms, false, buf);
}

static int wcn_gnss_domain_on(struct wcn_core *core, void *buf)
{
	const struct wcn_gnss_data *g = &core->chip->gnss;
	int ret;

	ret = wcn_gnss_update(core, g->power_reg, g->force_sleep, 0, buf);
	if (!ret)
		/* The BSP accepts either GNSS power-on finish indication. */
		ret = wcn_gnss_poll(core, g->power_status, g->power_ready,
				    0, 500, true, buf);
	if (!ret)
		ret = wcn_gnss_update(core, g->reset_reg, 0, g->reset_mask, buf);
	if (!ret)
		core->gnss_powered = true;
	return ret;
}

static int wcn_gnss_domain_off(struct wcn_core *core, void *buf)
{
	const struct wcn_gnss_data *g = &core->chip->gnss;
	int ret;

	ret = wcn_gnss_update(core, g->reset_reg, 0, g->reset_mask, buf);
	if (!ret)
		ret = wcn_gnss_update(core, g->fake_clock, g->fake_clock_mask, 0, buf);
	if (!ret)
		ret = wcn_gnss_wait(core, g->fake_clock, g->fake_clock_mask, 0, 100, buf);
	if (!ret)
		ret = wcn_gnss_update(core, g->apb_clock, g->apb_clock_mask, 0, buf);
	if (!ret)
		ret = wcn_gnss_update(core, g->power_reg, g->chip_sleep,
				      g->force_sleep | g->auto_enable, buf);
	if (!ret)
		core->gnss_powered = false;
	return ret;
}

static int wcn_gnss_write_image(struct wcn_core *core, const void *image,
				size_t size, void *buf)
{
	size_t off;
	int ret;

	for (off = 0; off < size; off += WCN_TX_CHUNK) {
		size_t len = min_t(size_t, size - off, WCN_TX_CHUNK);

		memcpy(buf, image + off, len);
		ret = wcn_sdio_dt_write(core, core->chip->gnss.base + off, buf, len);
		if (ret)
			return ret;
	}
	return 0;
}

/* All register I/O uses the kmalloc bounce buffer, including small values. */
static int wcn_gnss_boot(struct wcn_core *core, void *buf)
{
	const struct wcn_gnss_data *g = &core->chip->gnss;
	int ret;

	ret = wcn_gnss_write_image(core, core->gnss_image, core->gnss_image_size, buf);
	if (ret)
		return ret;
	if (core->gnss_cali_valid) {
		memcpy(buf, core->gnss_cali, g->cali_size);
		ret = wcn_sdio_dt_write(core, g->cali_addr, buf, g->cali_size);
		if (!ret) {
			memcpy(buf, core->gnss_efuse, sizeof(core->gnss_efuse));
			ret = wcn_sdio_dt_write(core, g->efuse_addr, buf,
						sizeof(core->gnss_efuse));
		}
	} else {
		ret = wcn_gnss_write32(core, g->cali_addr, 0, buf);
	}
	if (!ret)
		ret = wcn_gnss_write32(core, g->boot_addr, 0, buf);
	if (!ret)
		ret = wcn_gnss_update(core, g->reset_reg, g->reset_mask, 0, buf);
	if (ret)
		return ret;

	if (!core->gnss_cali_valid) {
		ret = wcn_gnss_wait(core, g->cali_addr, U32_MAX, g->cali_done, 3000, buf);
		if (ret)
			return ret;
		ret = wcn_sdio_dt_read(core, g->cali_addr, buf, g->cali_size);
		if (ret)
			return ret;
		memcpy(core->gnss_cali, buf, g->cali_size);
		ret = wcn_sdio_dt_read(core, g->efuse_addr, buf, sizeof(core->gnss_efuse));
		if (ret)
			return ret;
		memcpy(core->gnss_efuse, buf, sizeof(core->gnss_efuse));
		core->gnss_cali_valid = true;
		/* BSP requires a new boot with restored data after first calibration. */
		ret = wcn_gnss_domain_off(core, buf);
		if (!ret)
			ret = wcn_gnss_domain_on(core, buf);
		if (ret)
			return ret;
		return wcn_gnss_boot(core, buf);
	}
	return wcn_gnss_wait(core, g->boot_addr, U32_MAX, g->boot_done, 5000, buf);
}

int wcn_gnss_restart_locked(struct wcn_core *core)
{
	void *buf;
	int ret, rollback;

	if (!core->gnss_image || !core->gnss_image_size)
		return -EINVAL;
	buf = kmalloc(WCN_TX_CHUNK, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	core->gnss_running = false;
	ret = wcn_gnss_domain_on(core, buf);
	if (!ret)
		ret = wcn_gnss_boot(core, buf);
	if (ret) {
		rollback = wcn_gnss_domain_off(core, buf);
		if (rollback)
			dev_err(core->dev, "GNSS boot rollback failed: %d\n", rollback);
	} else {
		core->gnss_running = true;
	}
	kfree(buf);
	return ret;
}

static int wcn_gnss_io_begin(struct wcn_core *core, bool shutdown)
{
	int ret;

	mutex_lock(&core->power_lock);
	if (!core->gnss_prepared || (!shutdown && core->removing)) {
		ret = -ENODEV;
		goto unlock;
	}
	if (!shutdown && !core->fw_loaded) {
		ret = -ESHUTDOWN;
		goto unlock;
	}
	mutex_lock(&core->io_lock);
	ret = wcn_pm_get(core);
	if (!ret)
		return 0;
	mutex_unlock(&core->io_lock);
unlock:
	mutex_unlock(&core->power_lock);
	return ret;
}

static void wcn_gnss_io_end(struct wcn_core *core)
{
	wcn_pm_put(core);
	mutex_unlock(&core->io_lock);
	mutex_unlock(&core->power_lock);
}

int wcn_gnss_prepare(struct wcn_core *core)
{
	const struct wcn_gnss_data *g = &core->chip->gnss;
	void *buf;
	int ret, rollback;

	if (!g->max_size || !g->cali_size || g->cali_size > sizeof(core->gnss_cali))
		return -EOPNOTSUPP;
	buf = kmalloc(WCN_TX_CHUNK, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	ret = wcn_power_get(core, WCN_CHILD_GNSS);
	if (ret)
		goto free;
	mutex_lock(&core->power_lock);
	mutex_lock(&core->io_lock);
	ret = core->gnss_prepared ? -EBUSY : wcn_pm_get(core);
	if (!ret) {
		ret = wcn_gnss_domain_on(core, buf);
		if (!ret) {
			core->gnss_prepared = true;
		} else {
			rollback = wcn_gnss_domain_off(core, buf);
			if (rollback)
				dev_err(core->dev, "GNSS prepare rollback failed: %d\n", rollback);
		}
		wcn_pm_put(core);
	}
	mutex_unlock(&core->io_lock);
	mutex_unlock(&core->power_lock);
	if (ret)
		wcn_power_put(core, WCN_CHILD_GNSS);
free:
	kfree(buf);
	return ret;
}
EXPORT_SYMBOL_GPL(wcn_gnss_prepare);

int wcn_gnss_load(struct wcn_core *core, const void *data, size_t size)
{
	const void *image;
	void *copy;
	size_t image_size;
	int ret;

	ret = wcn_gnss_image(core, data, size, &image, &image_size);
	if (ret)
		return ret;
	copy = kvmalloc(image_size, GFP_KERNEL);
	if (!copy)
		return -ENOMEM;
	memcpy(copy, image, image_size);
	ret = wcn_gnss_io_begin(core, false);
	if (ret)
		goto free;
	if (core->gnss_running) {
		ret = -EBUSY;
	} else {
		kvfree(core->gnss_image);
		core->gnss_image = copy;
		core->gnss_image_size = image_size;
		copy = NULL;
	}
	wcn_gnss_io_end(core);
	if (!ret)
		dev_info(core->dev, "GNSS firmware: using %zu of %zu bytes\n", image_size, size);
free:
	kvfree(copy);
	return ret;
}
EXPORT_SYMBOL_GPL(wcn_gnss_load);

int wcn_gnss_start(struct wcn_core *core)
{
	int ret = wcn_gnss_io_begin(core, false);

	if (ret)
		return ret;
	ret = wcn_gnss_restart_locked(core);
	wcn_gnss_io_end(core);
	return ret;
}
EXPORT_SYMBOL_GPL(wcn_gnss_start);

int wcn_gnss_stop(struct wcn_core *core)
{
	void *buf;
	int ret;

	buf = kmalloc(WCN_TX_CHUNK, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	ret = wcn_gnss_io_begin(core, true);
	if (!ret) {
		ret = wcn_gnss_domain_off(core, buf);
		if (!ret)
			core->gnss_running = false;
		wcn_gnss_io_end(core);
	}
	kfree(buf);
	return ret;
}
EXPORT_SYMBOL_GPL(wcn_gnss_stop);

int wcn_gnss_unprepare(struct wcn_core *core)
{
	int ret = 0;
	bool prepared, powered;

	mutex_lock(&core->power_lock);
	prepared = core->gnss_prepared;
	powered = core->gnss_powered;
	mutex_unlock(&core->power_lock);
	if (!prepared)
		return 0;
	/* Also called on a firmware/start failure before rproc invokes stop. */
	if (powered)
		ret = wcn_gnss_stop(core);
	if (ret)
		dev_err(core->dev, "GNSS shutdown failed: %d\n", ret);
	mutex_lock(&core->power_lock);
	core->gnss_prepared = false;
	core->gnss_powered = false;
	core->gnss_running = false;
	kvfree(core->gnss_image);
	core->gnss_image = NULL;
	core->gnss_image_size = 0;
	mutex_unlock(&core->power_lock);
	wcn_power_put(core, WCN_CHILD_GNSS);
	return ret;
}
EXPORT_SYMBOL_GPL(wcn_gnss_unprepare);

int wcn_coexist_update_locked(struct wcn_core *core, unsigned int subsys,
			      bool enable)
{
	bool fm = core->subsys_users[WCN_CHILD_FM];
	bool gnss = core->subsys_users[WCN_CHILD_GNSS];
	__le32 *buf;
	int ret;

	if (!core->chip->fm_spur_reg)
		return 0;
	if (enable && subsys == WCN_CHILD_FM)
		fm = true;
	if (enable && subsys == WCN_CHILD_GNSS)
		gnss = true;
	buf = kmalloc_obj(*buf);
	if (!buf)
		return -ENOMEM;
	ret = wcn_gnss_write32(core, core->chip->fm_spur_reg,
			       fm && gnss ? core->chip->fm_spur_enable :
					    core->chip->fm_spur_disable, buf);
	kfree(buf);
	return ret;
}
