// SPDX-License-Identifier: GPL-2.0-only
/*
 * omnivision TCM touchscreen driver.
 *
 * The Omnivision TCM is TDDI TD4160 panels.
 *
 * Copyright (C) 2026 Veynamer <veyr1thalone@gmail.com>
 */

#include <linux/bitfield.h>
#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#include <drm/drm_panel.h>

#define TCM_MARKER			0xa5
#define TCM_PADDING			0x5a
#define TCM_HEADER_SIZE			4
#define TCM_WRITE_HEADER_SIZE		3
#define TCM_CONT_HEADER_SIZE		2
#define TCM_READ_SIZE			256
#define TCM_MAX_PAYLOAD			U16_MAX
#define TCM_MAX_OBJECTS			16
#define TCM_TIMEOUT_MS			3000
#define TCM_BOOT_TIMEOUT_MS		5000

#define TCM_CMD_CONTINUE_WRITE		0x01
#define TCM_CMD_GET_APP_INFO		0x20
#define TCM_CMD_SET_REPORT_CONFIG		0x26
#define TCM_CMD_SLEEP			0x2c
#define TCM_CMD_DOWNLOAD_CONFIG		0x30
#define TCM_CMD_ROMBOOT_RUN		0x42
#define TCM_CMD_ROMBOOT_DOWNLOAD		0x45

#define TCM_STATUS_IDLE			0x00
#define TCM_STATUS_OK			0x01
#define TCM_STATUS_BUSY			0x02
#define TCM_STATUS_CONTINUED_READ		0x03
#define TCM_STATUS_ERROR			0x0f
#define TCM_STATUS_INVALID		0xff

#define TCM_REPORT_IDENTIFY		0x10
#define TCM_REPORT_TOUCH			0x11
#define TCM_REPORT_STATUS			0x1b
#define TCM_REPORT_ROMBOOT		0xfd

#define TCM_MODE_APP			0x01
#define TCM_MODE_HOST_DOWNLOAD		0x02
#define TCM_MODE_ROMBOOT			0x04

#define TCM_FW_NEED_DISPLAY		BIT(1)
#define TCM_FW_NEED_APP			BIT(2)
#define TCM_FW_HDL_VERSION		GENMASK(6, 3)
#define TCM_FW_NEED_OPEN_SHORT		BIT(7)
#define TCM_FW_NEED_CONFIG		(TCM_FW_NEED_APP | \
					 TCM_FW_NEED_DISPLAY | \
					 TCM_FW_NEED_OPEN_SHORT)

#define TCM_IMAGE_MAGIC			0x4818472b
#define TCM_AREA_MAGIC			0x7c05e516
#define TCM_IMAGE_HEADER_SIZE		8
#define TCM_CODE_BLOCK_SIZE		16
#define TCM_ROMBOOT_RESERVED		14
#define TCM_CONFIG_HEADER_SIZE		2
#define TCM_CONFIG_ALIGN			8
#define TCM_AREA_CODE_ID			"ROMBOOT_APP_CODE"
#define TCM_AREA_TOUCH_ID			"APP_CONFIG"
#define TCM_AREA_DISPLAY_ID		"DISPLAY"
#define TCM_AREA_OPEN_SHORT_ID		"OPEN_SHORT"

#define TCM_TOUCH_COUNT_SIZE		1
#define TCM_TOUCH_INDEX_MASK		GENMASK(3, 0)
#define TCM_TOUCH_TYPE_MASK		GENMASK(7, 4)
#define TCM_TOUCH_FINGER			1
#define TCM_TOUCH_GLOVED_FINGER		2

#define TCM_APP_STATUS_OK			0
#define TCM_APP_STATUS_BOOTING		1
#define TCM_APP_STATUS_UPDATING		2

enum ovt_tcm_touch_report_code {
	TCM_TOUCH_END = 0x00,
	TCM_TOUCH_FOREACH_ACTIVE = 0x01,
	TCM_TOUCH_FOREACH_END = 0x03,
	TCM_TOUCH_INDEX = 0x06,
	TCM_TOUCH_CLASSIFICATION = 0x07,
	TCM_TOUCH_X_POSITION = 0x08,
	TCM_TOUCH_Y_POSITION = 0x09,
	TCM_TOUCH_PRESSURE = 0x0a,
	TCM_TOUCH_X_WIDTH = 0x0b,
	TCM_TOUCH_Y_WIDTH = 0x0c,
	TCM_TOUCH_ACTIVE_COUNT = 0x18,
};

enum ovt_tcm_config_type {
	TCM_CONFIG_TOUCH = 1,
	TCM_CONFIG_DISPLAY,
	TCM_CONFIG_OPEN_SHORT,
	TCM_CONFIG_COUNT,
};

struct ovt_tcm_identify_info {
	u8 version;
	u8 mode;
	u8 part_number[16];
	__le32 build_id;
	__le16 max_write_size;
} __packed;

struct ovt_tcm_touch_object {
	u8 index_status;
	__le16 x;
	__le16 y;
	u8 pressure;
	u8 x_width;
	u8 y_width;
} __packed;

/* One count byte, followed by fixed-size records for active objects. */
static const u8 ovt_tcm_report_config[] = {
	TCM_TOUCH_ACTIVE_COUNT, 8,
	TCM_TOUCH_FOREACH_ACTIVE,
	TCM_TOUCH_INDEX, 4,
	TCM_TOUCH_CLASSIFICATION, 4,
	TCM_TOUCH_X_POSITION, 16,
	TCM_TOUCH_Y_POSITION, 16,
	TCM_TOUCH_PRESSURE, 8,
	TCM_TOUCH_X_WIDTH, 8,
	TCM_TOUCH_Y_WIDTH, 8,
	TCM_TOUCH_FOREACH_END,
	TCM_TOUCH_END,
};

struct ovt_tcm_area_descriptor {
	__le32 magic;
	u8 id[16];
	__le32 flags;
	__le32 address;
	__le32 length;
	__le32 checksum;
} __packed;

struct ovt_tcm_firmware_area {
	const u8 *data;
	size_t size;
};

struct ovt_tcm_app_info {
	__le16 version;
	__le16 status;
	__le16 static_config_size;
	__le16 dynamic_config_size;
	__le16 app_config_start;
	__le16 app_config_size;
	__le16 max_report_config_size;
	__le16 max_report_payload_size;
	u8 config_id[16];
	__le16 max_x;
	__le16 max_y;
	__le16 max_objects;
} __packed;

struct ovt_tcm_data {
	struct spi_device *spi;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *irq_gpio;
	struct input_dev *input_dev;
	struct touchscreen_properties prop;
	struct drm_panel_follower panel_follower;
	/* Lock order: state_lock, then lock. Never hold lock in disable_irq(). */
	struct mutex state_lock;
	/* Serialize SPI messages, commands and input reporting. */
	struct mutex lock;
	struct work_struct fw_reload_work;
	const struct firmware *firmware;
	struct ovt_tcm_firmware_area code;
	struct ovt_tcm_firmware_area config[TCM_CONFIG_COUNT - 1];
	u8 *tx;
	u8 *rx;
	u8 *payload;
	size_t tx_size;
	u16 payload_size;
	u16 fw_status;
	u16 max_write_size;
	u16 read_size;
	u16 max_x;
	u16 max_y;
	u32 fw_build_id;
	u8 report;
	u8 mode;
	u8 max_objects;
	bool have_fw_status;
	bool input_registered;
	bool fw_running;
	bool irq_enabled;
	bool following_panel;
	bool removing;
	int irq;
	int start_error;
};

/* SPI core splits large transfers while keeping CS asserted for the message. */
static int ovt_tcm_spi_transfer(struct ovt_tcm_data *ts, size_t length, bool read)
{
	struct spi_transfer xfer = {
		.tx_buf = ts->tx,
		.rx_buf = read ? ts->rx : NULL,
		.len = length,
	};

	lockdep_assert_held(&ts->lock);

	if (!length || length > ts->tx_size ||
	    (read && length > TCM_READ_SIZE) ||
	    length > spi_max_message_size(ts->spi))
		return -EMSGSIZE;

	if (read)
		memset(ts->tx, 0xff, length);

	return spi_sync_transfer(ts->spi, &xfer, 1);
}

static int ovt_tcm_spi_read(struct ovt_tcm_data *ts, size_t length)
{
	return ovt_tcm_spi_transfer(ts, length, true);
}

static int ovt_tcm_spi_write(struct ovt_tcm_data *ts, size_t length)
{
	return ovt_tcm_spi_transfer(ts, length, false);
}

/* Called with lock held; payload contains the complete report on success. */
static int ovt_tcm_read_message(struct ovt_tcm_data *ts)
{
	const struct ovt_tcm_identify_info *info;
	size_t read_size = ts->read_size ?: TCM_READ_SIZE;
	size_t remaining, offset = 0, size;
	unsigned int attempt;
	u8 *payload = ts->payload;
	int ret;

	ret = gpiod_get_value_cansleep(ts->irq_gpio);
	if (ret < 0)
		return ret;
	if (!ret)
		return -EAGAIN;

	/* The first transfer after reset or idle may only wake the SPI port. */
	for (attempt = 0; attempt < 2; attempt++) {
		ret = ovt_tcm_spi_read(ts, read_size);
		if (ret)
			return ret;
		if (ts->rx[0] == TCM_MARKER)
			break;
		if (!attempt)
			usleep_range(10000, 11000);
	}
	if (ts->rx[0] != TCM_MARKER) {
		dev_dbg(&ts->spi->dev, "invalid message header: %4ph\n", ts->rx);
		return -EPROTO;
	}

	ts->report = ts->rx[1];
	ts->payload_size = get_unaligned_le16(ts->rx + 2);
	if (ts->report == TCM_STATUS_IDLE ||
	    ts->report == TCM_STATUS_BUSY || ts->report == TCM_STATUS_INVALID)
		return -EAGAIN;
	if (ts->report == TCM_STATUS_CONTINUED_READ)
		return -EPROTO;
	remaining = ts->payload_size + 1;
	size = min_t(size_t, remaining, read_size - TCM_HEADER_SIZE);
	memcpy(payload, ts->rx + TCM_HEADER_SIZE, size);
	offset = size;
	remaining -= size;

	while (remaining) {
		size = min_t(size_t, remaining, TCM_READ_SIZE - TCM_CONT_HEADER_SIZE);
		/* A final padding byte is not sent in a separate transaction. */
		if (size == 1) {
			payload[offset] = TCM_PADDING;
			break;
		}
		ret = ovt_tcm_spi_read(ts, size + TCM_CONT_HEADER_SIZE);
		if (ret)
			return ret;
		if (ts->rx[0] != TCM_MARKER ||
		    ts->rx[1] != TCM_STATUS_CONTINUED_READ) {
			dev_dbg(&ts->spi->dev, "invalid continued read header: %4ph\n", ts->rx);
			return -EPROTO;
		}
		memcpy(payload + offset, ts->rx + TCM_CONT_HEADER_SIZE, size);
		offset += size;
		remaining -= size;
	}

	if (payload[ts->payload_size] != TCM_PADDING)
		return -EPROTO;

	if (ts->report == TCM_REPORT_IDENTIFY) {
		if (ts->payload_size < sizeof(*info))
			return -EPROTO;
		info = (const void *)payload;
		ts->mode = info->mode;
		ts->fw_build_id = le32_to_cpu(info->build_id);
		ts->max_write_size = le16_to_cpu(info->max_write_size);
	} else if (ts->report == TCM_REPORT_STATUS) {
		if (ts->payload_size < 2)
			return -EPROTO;
		ts->fw_status = get_unaligned_le16(payload);
		ts->have_fw_status = true;
	}

	return 0;
}

static int ovt_tcm_write_command(struct ovt_tcm_data *ts, u8 command,
				 const u8 *payload, size_t length)
{
	size_t header = TCM_WRITE_HEADER_SIZE;
	size_t offset = 0, size, limit;
	int ret;

	if (length > TCM_MAX_PAYLOAD && command != TCM_CMD_ROMBOOT_DOWNLOAD)
		return -EMSGSIZE;
	limit = ts->max_write_size ?: TCM_READ_SIZE;
	if (command == TCM_CMD_ROMBOOT_DOWNLOAD)
		limit = length + header;
	if (limit < header || length + header > ts->tx_size)
		return -EMSGSIZE;

	ts->tx[0] = command;
	put_unaligned_le16(length, ts->tx + 1);
	do {
		size = min(length - offset, limit - header);
		if (size)
			memcpy(ts->tx + header, payload + offset, size);
		ret = ovt_tcm_spi_write(ts, size + header);
		if (ret)
			return ret;
		offset += size;
		header = 1;
		ts->tx[0] = TCM_CMD_CONTINUE_WRITE;
		if (offset < length)
			usleep_range(500, 1000);
	} while (offset < length);

	return 0;
}

/* Serialize commands and IRQ reads; poll during boot while IRQ is disabled. */
static int ovt_tcm_send_command(struct ovt_tcm_data *ts, u8 command,
				const u8 *payload, size_t length)
{
	unsigned long deadline = jiffies + msecs_to_jiffies(TCM_TIMEOUT_MS);
	int ret;

	ret = ovt_tcm_write_command(ts, command, payload, length);
	if (ret)
		return ret;

	do {
		usleep_range(5000, 6000);
		ret = ovt_tcm_read_message(ts);
		if (ret == -EAGAIN)
			continue;
		if (ret)
			return ret;
		if (ts->report == TCM_STATUS_OK)
			return 0;
		if (ts->report <= TCM_STATUS_ERROR)
			return -EIO;
		if (ts->report == TCM_REPORT_IDENTIFY) {
			if (command == TCM_CMD_ROMBOOT_RUN)
				return 0;
			return -EPIPE;
		}
	} while (time_before(jiffies, deadline));

	return -ETIMEDOUT;
}

static int ovt_tcm_parse_firmware(struct ovt_tcm_data *ts)
{
	const struct firmware *fw = ts->firmware;
	const struct ovt_tcm_area_descriptor *desc;
	struct ovt_tcm_firmware_area *area;
	size_t offset, length;
	u32 count, i;
	const u8 *data;

	if (fw->size < TCM_IMAGE_HEADER_SIZE ||
	    get_unaligned_le32(fw->data) != TCM_IMAGE_MAGIC)
		return -EINVAL;

	count = get_unaligned_le32(fw->data + 4);
	if (count > (fw->size - TCM_IMAGE_HEADER_SIZE) / sizeof(__le32))
		return -EINVAL;

	for (i = 0; i < count; i++) {
		offset = get_unaligned_le32(fw->data + TCM_IMAGE_HEADER_SIZE +
					  i * sizeof(__le32));
		if (offset > fw->size || fw->size - offset < sizeof(__le32))
			return -EINVAL;

		/* Images may also contain non-flash areas, such as debug data. */
		if (get_unaligned_le32(fw->data + offset) != TCM_AREA_MAGIC)
			continue;
		if (fw->size - offset < sizeof(*desc))
			return -EINVAL;

		desc = (const void *)(fw->data + offset);
		length = le32_to_cpu(desc->length);
		offset += sizeof(*desc);
		if (length > fw->size - offset)
			return -EINVAL;

		/* Area IDs are fixed-width and may be padded with spaces. */
		if (!memcmp(desc->id, TCM_AREA_CODE_ID, sizeof(TCM_AREA_CODE_ID) - 1))
			area = &ts->code;
		else if (!memcmp(desc->id, TCM_AREA_TOUCH_ID, sizeof(TCM_AREA_TOUCH_ID) - 1))
			area = &ts->config[TCM_CONFIG_TOUCH - 1];
		else if (!memcmp(desc->id, TCM_AREA_DISPLAY_ID, sizeof(TCM_AREA_DISPLAY_ID) - 1))
			area = &ts->config[TCM_CONFIG_DISPLAY - 1];
		else if (!memcmp(desc->id, TCM_AREA_OPEN_SHORT_ID,
				 sizeof(TCM_AREA_OPEN_SHORT_ID) - 1))
			area = &ts->config[TCM_CONFIG_OPEN_SHORT - 1];
		else
			continue;

		data = fw->data + offset;
		if (!length || area->data ||
		    (crc32(~0, data, length) ^ ~0) != le32_to_cpu(desc->checksum))
			return -EINVAL;

		area->data = data;
		area->size = length;
	}

	/* The ROM code image consists of complete 16-byte blocks. */
	if (!ts->code.size || ts->code.size % TCM_CODE_BLOCK_SIZE ||
	    ts->code.size > GENMASK(23, 0) - TCM_ROMBOOT_RESERVED ||
	    !ts->config[TCM_CONFIG_TOUCH - 1].size)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(ts->config); i++)
		if (ALIGN(ts->config[i].size, TCM_CONFIG_ALIGN) >
		    TCM_MAX_PAYLOAD - TCM_CONFIG_HEADER_SIZE)
			return -EFBIG;

	return 0;
}

static void ovt_tcm_release_firmware(void *data)
{
	struct ovt_tcm_data *ts = data;

	release_firmware(ts->firmware);
}

static int ovt_tcm_request_firmware(struct ovt_tcm_data *ts)
{
	struct device *dev = &ts->spi->dev;
	const char *firmware_name;
	int ret;

	ret = device_property_read_string(dev, "firmware-name", &firmware_name);
	if (ret)
		return dev_err_probe(dev, ret, "firmware-name is required\n");
	ret = request_firmware(&ts->firmware, firmware_name, dev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request firmware '%s'\n", firmware_name);
	ret = devm_add_action_or_reset(dev, ovt_tcm_release_firmware, ts);
	if (ret)
		return ret;
	ret = ovt_tcm_parse_firmware(ts);
	if (ret)
		return dev_err_probe(dev, ret, "invalid firmware image\n");

	ts->tx_size = max_t(size_t, TCM_MAX_PAYLOAD + TCM_WRITE_HEADER_SIZE,
			    ts->code.size + TCM_ROMBOOT_RESERVED + TCM_WRITE_HEADER_SIZE);
	ts->tx = devm_kmalloc(dev, ts->tx_size, GFP_KERNEL);
	ts->rx = devm_kmalloc(dev, TCM_READ_SIZE, GFP_KERNEL);
	ts->payload = devm_kmalloc(dev, TCM_MAX_PAYLOAD + 1, GFP_KERNEL);
	if (!ts->tx || !ts->rx || !ts->payload)
		return -ENOMEM;

	return 0;
}

static int ovt_tcm_download_code(struct ovt_tcm_data *ts)
{
	size_t length = ts->code.size + TCM_ROMBOOT_RESERVED;
	u8 *payload;
	int ret;

	payload = kzalloc(length, GFP_KERNEL);
	if (!payload)
		return -ENOMEM;

	/* Low 16 bits are in the command length; ROM takes the upper bits here. */
	payload[0] = ts->code.size >> 16;
	memcpy(payload + TCM_ROMBOOT_RESERVED, ts->code.data, ts->code.size);
	ret = ovt_tcm_send_command(ts, TCM_CMD_ROMBOOT_DOWNLOAD, payload, length);
	kfree(payload);
	if (ret)
		return ret;

	return ovt_tcm_send_command(ts, TCM_CMD_ROMBOOT_RUN, NULL, 0);
}

static int ovt_tcm_download_config(struct ovt_tcm_data *ts, enum ovt_tcm_config_type type)
{
	const struct ovt_tcm_firmware_area *area = &ts->config[type - 1];
	u8 version = FIELD_GET(TCM_FW_HDL_VERSION, ts->fw_status);
	size_t length = area->size;
	u8 *payload;
	int ret;

	if (!length || version > 2)
		return -EINVAL;
	if (type == TCM_CONFIG_TOUCH)
		length = ALIGN(length, TCM_CONFIG_ALIGN);
	payload = kzalloc(length + TCM_CONFIG_HEADER_SIZE, GFP_KERNEL);
	if (!payload)
		return -ENOMEM;
	payload[0] = version == 2 ? 2 : 1;
	payload[1] = type;
	memcpy(payload + TCM_CONFIG_HEADER_SIZE, area->data, area->size);
	ret = ovt_tcm_send_command(ts, TCM_CMD_DOWNLOAD_CONFIG, payload,
				   length + TCM_CONFIG_HEADER_SIZE);
	kfree(payload);
	if (ret)
		return ret;
	if (ts->payload_size < 2)
		return -EPROTO;
	ts->fw_status = get_unaligned_le16(ts->payload);
	ts->have_fw_status = true;
	return 0;
}

static int ovt_tcm_load_firmware(struct ovt_tcm_data *ts)
{
	unsigned long deadline;
	int ret;

	ts->have_fw_status = false;
	ts->max_write_size = TCM_READ_SIZE;
	/* Control responses fit in this read without a continued transaction. */
	ts->read_size = TCM_READ_SIZE;
	gpiod_set_value_cansleep(ts->reset_gpio, 1);
	msleep(20);
	gpiod_set_value_cansleep(ts->reset_gpio, 0);
	msleep(200);

	deadline = jiffies + msecs_to_jiffies(TCM_BOOT_TIMEOUT_MS);
	do {
		ret = ovt_tcm_read_message(ts);
		if (ret != -EAGAIN)
			break;
		usleep_range(5000, 6000);
	} while (time_before(jiffies, deadline));
	if (ret)
		return ret == -EAGAIN ? -ETIMEDOUT : ret;
	if (ts->report != TCM_REPORT_IDENTIFY || ts->mode != TCM_MODE_ROMBOOT)
		return -ENODEV;

	ret = ovt_tcm_download_code(ts);
	if (ret)
		return ret;
	if (ts->mode != TCM_MODE_HOST_DOWNLOAD && ts->mode != TCM_MODE_APP)
		return -ENODEV;

	deadline = jiffies + msecs_to_jiffies(TCM_BOOT_TIMEOUT_MS);
	do {
		if (!ts->have_fw_status) {
			ret = ovt_tcm_read_message(ts);
			if (ret && ret != -EAGAIN)
				return ret;
		} else if (ts->fw_status & TCM_FW_NEED_APP) {
			ret = ovt_tcm_download_config(ts, TCM_CONFIG_TOUCH);
			if (ret)
				return ret;
		} else if (ts->fw_status & TCM_FW_NEED_DISPLAY) {
			ret = ovt_tcm_download_config(ts, TCM_CONFIG_DISPLAY);
			if (ret)
				return ret;
		} else if (ts->fw_status & TCM_FW_NEED_OPEN_SHORT) {
			ret = ovt_tcm_download_config(ts, TCM_CONFIG_OPEN_SHORT);
			if (ret)
				return ret;
		} else {
			return 0;
		}
		usleep_range(5000, 6000);
	} while (time_before(jiffies, deadline));

	return -ETIMEDOUT;
}

static int ovt_tcm_get_app_info(struct ovt_tcm_data *ts, struct ovt_tcm_app_info *info)
{
	unsigned long deadline = jiffies + msecs_to_jiffies(TCM_BOOT_TIMEOUT_MS);
	u16 status;
	int ret;

	do {
		ret = ovt_tcm_send_command(ts, TCM_CMD_GET_APP_INFO, NULL, 0);
		if (ret)
			return ret;
		if (ts->payload_size < sizeof(*info))
			return -EPROTO;

		memcpy(info, ts->payload, sizeof(*info));
		status = le16_to_cpu(info->status);
		if (status == TCM_APP_STATUS_OK)
			return 0;
		if (status != TCM_APP_STATUS_BOOTING && status != TCM_APP_STATUS_UPDATING)
			return -EIO;

		msleep(20);
	} while (time_before(jiffies, deadline));

	return -ETIMEDOUT;
}

static int ovt_tcm_configure_touch(struct ovt_tcm_data *ts,
				   const struct ovt_tcm_app_info *info)
{
	u16 objects = le16_to_cpu(info->max_objects);
	u16 config_size = le16_to_cpu(info->max_report_config_size);
	u16 max_x = le16_to_cpu(info->max_x);
	u16 max_y = le16_to_cpu(info->max_y);
	u8 *config;
	int ret;

	if (!objects || objects > TCM_MAX_OBJECTS || !max_x || !max_y ||
	    config_size < sizeof(ovt_tcm_report_config))
		return -EINVAL;
	if (ts->input_registered && (objects != ts->max_objects ||
				     max_x != ts->max_x || max_y != ts->max_y))
		return -EINVAL;

	config = kzalloc(config_size, GFP_KERNEL);
	if (!config)
		return -ENOMEM;
	memcpy(config, ovt_tcm_report_config, sizeof(ovt_tcm_report_config));
	ret = ovt_tcm_send_command(ts, TCM_CMD_SET_REPORT_CONFIG, config, config_size);
	kfree(config);
	if (ret)
		return ret;

	ts->max_objects = objects;
	ts->max_x = max_x;
	ts->max_y = max_y;
	/* Include every slot, so adding a finger never needs a continued read. */
	ts->read_size = TCM_HEADER_SIZE + 1 +
		max_t(size_t, sizeof(struct ovt_tcm_identify_info),
		      TCM_TOUCH_COUNT_SIZE + objects * sizeof(struct ovt_tcm_touch_object));

	return 0;
}

static int ovt_tcm_init_input(struct ovt_tcm_data *ts)
{
	struct input_dev *input_dev = ts->input_dev;
	int ret;

	if (ts->input_registered)
		return 0;

	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, ts->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, ts->max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, U8_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, U8_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MINOR, 0, U8_MAX, 0, 0);
	touchscreen_parse_properties(input_dev, true, &ts->prop);

	ret = input_mt_init_slots(input_dev, ts->max_objects,
				  INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (ret)
		return ret;
	ret = input_register_device(input_dev);
	if (ret)
		return ret;

	ts->input_registered = true;
	dev_info(&ts->spi->dev, "firmware initialized (build_id=%u, mode=0x%02x)\n",
		 ts->fw_build_id, ts->mode);

	return 0;
}

static int ovt_tcm_initialize(struct ovt_tcm_data *ts)
{
	struct device *dev = &ts->spi->dev;
	struct ovt_tcm_app_info info;
	int ret;

	lockdep_assert_held(&ts->state_lock);
	lockdep_assert_held(&ts->lock);

	ret = ovt_tcm_load_firmware(ts);
	if (ret) {
		dev_err(dev, "failed to load firmware: %d\n", ret);
		return ret;
	}
	ret = ovt_tcm_get_app_info(ts, &info);
	if (ret) {
		dev_err(dev, "failed to get application information: %d\n", ret);
		return ret;
	}
	ret = ovt_tcm_configure_touch(ts, &info);
	if (ret) {
		dev_err(dev, "failed to configure touch reports: %d\n", ret);
		return ret;
	}
	ret = ovt_tcm_init_input(ts);
	if (ret) {
		dev_err(dev, "failed to initialize input device: %d\n", ret);
		return ret;
	}

	ts->fw_running = true;
	ts->irq_enabled = true;
	enable_irq(ts->irq);
	return 0;
}

static int ovt_tcm_report_touch(struct ovt_tcm_data *ts)
{
	struct input_dev *input_dev = ts->input_dev;
	const struct ovt_tcm_touch_object *objects;
	unsigned int count, i, slot, type, x_width, y_width;
	unsigned long seen = 0;

	if (!ts->fw_running)
		return 0;
	if (ts->payload_size < TCM_TOUCH_COUNT_SIZE)
		return -EPROTO;
	count = ts->payload[0];
	if (count > ts->max_objects ||
	    ts->payload_size != TCM_TOUCH_COUNT_SIZE + count * sizeof(*objects))
		return -EPROTO;

	objects = (const void *)(ts->payload + TCM_TOUCH_COUNT_SIZE);
	/* Validate the complete report before changing any slot state. */
	for (i = 0; i < count; i++) {
		slot = FIELD_GET(TCM_TOUCH_INDEX_MASK, objects[i].index_status);
		type = FIELD_GET(TCM_TOUCH_TYPE_MASK, objects[i].index_status);
		if (slot >= ts->max_objects || test_and_set_bit(slot, &seen))
			return -EPROTO;
		if ((type == TCM_TOUCH_FINGER || type == TCM_TOUCH_GLOVED_FINGER) &&
		    (le16_to_cpu(objects[i].x) > ts->max_x ||
		     le16_to_cpu(objects[i].y) > ts->max_y))
			return -EPROTO;
	}

	for (i = 0; i < count; i++) {
		slot = FIELD_GET(TCM_TOUCH_INDEX_MASK, objects[i].index_status);
		type = FIELD_GET(TCM_TOUCH_TYPE_MASK, objects[i].index_status);
		input_mt_slot(input_dev, slot);
		if (!input_mt_report_slot_state(input_dev, MT_TOOL_FINGER,
						type == TCM_TOUCH_FINGER ||
						type == TCM_TOUCH_GLOVED_FINGER))
			continue;

		touchscreen_report_pos(input_dev, &ts->prop,
				       le16_to_cpu(objects[i].x),
				       le16_to_cpu(objects[i].y), true);
		input_report_abs(input_dev, ABS_MT_PRESSURE, objects[i].pressure);
		x_width = objects[i].x_width;
		y_width = objects[i].y_width;
		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, max(x_width, y_width));
		input_report_abs(input_dev, ABS_MT_TOUCH_MINOR, min(x_width, y_width));
	}
	input_mt_sync_frame(input_dev);
	input_sync(input_dev);
	return 0;
}

static void ovt_tcm_release_touches(struct ovt_tcm_data *ts)
{
	unsigned int i;

	if (!ts->input_registered)
		return;
	for (i = 0; i < ts->max_objects; i++) {
		input_mt_slot(ts->input_dev, i);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
	}
	input_mt_sync_frame(ts->input_dev);
	input_sync(ts->input_dev);
}

static int ovt_tcm_start(struct ovt_tcm_data *ts)
{
	int ret;

	mutex_lock(&ts->state_lock);
	mutex_lock(&ts->lock);
	if (ts->removing || ts->fw_running) {
		mutex_unlock(&ts->lock);
		mutex_unlock(&ts->state_lock);
		return 0;
	}
	ret = ovt_tcm_initialize(ts);
	ts->start_error = ret;
	mutex_unlock(&ts->lock);
	mutex_unlock(&ts->state_lock);
	return ret;
}

static void ovt_tcm_stop(struct ovt_tcm_data *ts)
{
	int ret;

	mutex_lock(&ts->state_lock);
	/* Do not wait for an IRQ thread while holding its mutex. */
	if (ts->irq_enabled) {
		disable_irq(ts->irq);
		ts->irq_enabled = false;
	}
	mutex_lock(&ts->lock);
	if (ts->fw_running) {
		ts->fw_running = false;
		ret = ovt_tcm_send_command(ts, TCM_CMD_SLEEP, NULL, 0);
		if (ret)
			dev_warn(&ts->spi->dev, "failed to enter sleep: %d\n", ret);
		ovt_tcm_release_touches(ts);
	}
	mutex_unlock(&ts->lock);
	mutex_unlock(&ts->state_lock);
	cancel_work_sync(&ts->fw_reload_work);
}

static void ovt_tcm_fw_work(struct work_struct *work)
{
	struct ovt_tcm_data *ts = container_of(work, struct ovt_tcm_data, fw_reload_work);

	mutex_lock(&ts->state_lock);
	if (!ts->irq_enabled)
		goto unlock_state;
	disable_irq(ts->irq);
	ts->irq_enabled = false;
	mutex_lock(&ts->lock);
	if (!ts->fw_running || ts->removing)
		goto out;
	ts->fw_running = false;
	ovt_tcm_release_touches(ts);
	ovt_tcm_initialize(ts);
out:
	mutex_unlock(&ts->lock);
unlock_state:
	mutex_unlock(&ts->state_lock);
}

static irqreturn_t ovt_tcm_threaded_irq(int irq, void *dev_id)
{
	struct ovt_tcm_data *ts = dev_id;
	int ret;

	mutex_lock(&ts->lock);
	if (!ts->fw_running || ts->removing)
		goto out;
	ret = ovt_tcm_read_message(ts);
	if (ret == -EAGAIN)
		goto out;
	if (!ret && ts->report == TCM_REPORT_TOUCH)
		ret = ovt_tcm_report_touch(ts);
	if (ret) {
		dev_err_ratelimited(&ts->spi->dev, "failed to process touch report: %d\n", ret);
		schedule_work(&ts->fw_reload_work);
		goto out;
	}
	if (ts->report == TCM_REPORT_IDENTIFY || ts->report == TCM_REPORT_ROMBOOT ||
	    (ts->report == TCM_REPORT_STATUS && (ts->fw_status & TCM_FW_NEED_CONFIG)))
		schedule_work(&ts->fw_reload_work);
out:
	mutex_unlock(&ts->lock);
	return IRQ_HANDLED;
}

static int ovt_tcm_panel_prepared(struct drm_panel_follower *follower)
{
	struct ovt_tcm_data *ts = container_of(follower, struct ovt_tcm_data, panel_follower);

	return ovt_tcm_start(ts);
}

static int ovt_tcm_panel_unpreparing(struct drm_panel_follower *follower)
{
	struct ovt_tcm_data *ts = container_of(follower, struct ovt_tcm_data, panel_follower);

	ovt_tcm_stop(ts);
	return 0;
}

static const struct drm_panel_follower_funcs ovt_tcm_panel_follower_funcs = {
	.panel_prepared = ovt_tcm_panel_prepared,
	.panel_unpreparing = ovt_tcm_panel_unpreparing,
};

static int ovt_tcm_suspend(struct device *dev)
{
	struct ovt_tcm_data *ts = dev_get_drvdata(dev);

	if (!ts->following_panel)
		ovt_tcm_stop(ts);
	return 0;
}

static int ovt_tcm_resume(struct device *dev)
{
	struct ovt_tcm_data *ts = dev_get_drvdata(dev);

	return ts->following_panel ? 0 : ovt_tcm_start(ts);
}

static DEFINE_SIMPLE_DEV_PM_OPS(ovt_tcm_pm_ops, ovt_tcm_suspend, ovt_tcm_resume);

static int ovt_tcm_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct ovt_tcm_data *ts;
	unsigned long irq_flags;
	int ret;

	if ((spi->mode & SPI_MODE_X_MASK) != SPI_MODE_3)
		return dev_err_probe(dev, -EINVAL, "SPI mode 3 is required\n");
	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set up SPI\n");

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;
	ts->spi = spi;
	mutex_init(&ts->state_lock);
	mutex_init(&ts->lock);
	INIT_WORK(&ts->fw_reload_work, ovt_tcm_fw_work);
	spi_set_drvdata(spi, ts);

	ret = ovt_tcm_request_firmware(ts);
	if (ret)
		return ret;
	/* Firmware reload must be able to return the controller to ROMBOOT. */
	ts->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ts->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ts->reset_gpio), "failed to get reset GPIO\n");
	ts->irq_gpio = devm_gpiod_get(dev, "irq", GPIOD_IN);
	if (IS_ERR(ts->irq_gpio))
		return dev_err_probe(dev, PTR_ERR(ts->irq_gpio), "failed to get IRQ GPIO\n");
	ts->irq = gpiod_to_irq(ts->irq_gpio);
	if (ts->irq < 0)
		return dev_err_probe(dev, ts->irq, "failed to get GPIO IRQ\n");

	ts->input_dev = devm_input_allocate_device(dev);
	if (!ts->input_dev)
		return -ENOMEM;
	ts->input_dev->name = "omnivision_tcm_touch";
	ts->input_dev->id.bustype = BUS_SPI;

	irq_flags = gpiod_is_active_low(ts->irq_gpio) ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH;
	irq_flags |= IRQF_ONESHOT | IRQF_NO_AUTOEN;
	ret = devm_request_threaded_irq(dev, ts->irq, NULL, ovt_tcm_threaded_irq,
					irq_flags, dev_name(dev), ts);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request threaded IRQ\n");

	if (drm_is_panel_follower(dev)) {
		ts->panel_follower.funcs = &ovt_tcm_panel_follower_funcs;
		ts->following_panel = true;
		ret = drm_panel_add_follower(dev, &ts->panel_follower);
		if (ret)
			return dev_err_probe(dev, ret, "failed to add panel follower\n");
		/* add_follower() logs callback errors without returning them. */
		mutex_lock(&ts->lock);
		ret = ts->start_error;
		mutex_unlock(&ts->lock);
		if (ret) {
			drm_panel_remove_follower(&ts->panel_follower);
			return ret;
		}
	} else {
		ret = ovt_tcm_start(ts);
		if (ret)
			return ret;
	}

	return 0;
}

static void ovt_tcm_remove(struct spi_device *spi)
{
	struct ovt_tcm_data *ts = spi_get_drvdata(spi);

	mutex_lock(&ts->lock);
	ts->removing = true;
	mutex_unlock(&ts->lock);
	if (ts->following_panel)
		drm_panel_remove_follower(&ts->panel_follower);
	ovt_tcm_stop(ts);
}

static const struct of_device_id ovt_tcm_of_match[] = {
	{ .compatible = "omnivision,tcm-spi" },
	{ }
};
MODULE_DEVICE_TABLE(of, ovt_tcm_of_match);

static const struct spi_device_id ovt_tcm_spi_ids[] = {
	{ "tcm-spi" },
	{ }
};
MODULE_DEVICE_TABLE(spi, ovt_tcm_spi_ids);

static struct spi_driver ovt_tcm_driver = {
	.driver = {
		.name = "omnivision-tcm-spi",
		.of_match_table = ovt_tcm_of_match,
		.pm = pm_sleep_ptr(&ovt_tcm_pm_ops),
	},
	.probe = ovt_tcm_probe,
	.remove = ovt_tcm_remove,
	.id_table = ovt_tcm_spi_ids,
};
module_spi_driver(ovt_tcm_driver);

MODULE_DESCRIPTION("OmniVision TCM SPI touchscreen driver");
MODULE_AUTHOR("Veynamer <veyr1thalone@gmail.com>");
MODULE_LICENSE("GPL");
