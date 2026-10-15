// SPDX-License-Identifier: GPL-2.0-only
/*
 * Omnivision TouchComm touchscreen driver, SPI transport.
 *
 * Minimal mainline implementation covering application-firmware touch
 * reporting: power/reset, identify, touch report configuration and
 * multitouch (Type B) input. Bootloader, firmware update, diagnostics
 * and factory testing are intentionally not supported.
 *
 * Protocol notes (TouchComm message framing):
 *   message = 0xA5 MARKER, CODE, LEN_LO, LEN_HI, payload..., 0x5A PAD
 *   host command = CMD, LEN_LO, LEN_HI, payload...
 *   command responses carry a code < 0x10 (STATUS_OK on success),
 *   unsolicited reports carry codes >= 0x10 (touch = 0x11).
 */

#include <linux/bitops.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/unaligned.h>

#define TCM_MSG_MARKER		0xa5
#define TCM_MSG_PADDING		0x5a
#define TCM_MSG_HEADER_SIZE	4

#define TCM_CMD_IDENTIFY		0x02
#define TCM_CMD_ENABLE_REPORT		0x05
#define TCM_CMD_DISABLE_REPORT		0x06
#define TCM_CMD_GET_APPLICATION_INFO	0x20
#define TCM_CMD_SET_TOUCH_REPORT_CONFIG	0x26

#define TCM_STATUS_OK		0x01
#define TCM_REPORT_IDENTIFY	0x10
#define TCM_REPORT_TOUCH	0x11

#define TCM_MODE_APPLICATION	0x01
#define TCM_APP_STATUS_OK	0x00

/* Offsets into the identify response payload */
#define TCM_ID_MODE_OFF		1
#define TCM_ID_BUILD_ID_OFF	18
#define TCM_ID_MIN_LEN		24

/* Offsets into the application info response payload */
#define TCM_APP_STATUS_OFF	2
#define TCM_APP_MAX_CFG_SIZE_OFF	12
#define TCM_APP_MAX_X_OFF	32
#define TCM_APP_MAX_Y_OFF	34
#define TCM_APP_MAX_OBJECTS_OFF	36
#define TCM_APP_INFO_MIN_LEN	38

/* Touch report configuration programmed into the controller.
 * Per active object, LSB packed: INDEX:4 CLASS:4 X:12 Y:12 XW:16 YW:16 Z:16
 */
#define TCM_OBJ_BITS		80
#define TCM_OBJ_LIFT		0
#define TCM_OBJ_FINGER		1
#define TCM_OBJ_GLOVED_FINGER	2

#define TCM_MAX_PRESSURE	1000
#define TCM_MAX_TOUCH_MAJOR	255

#define TCM_RESPONSE_TIMEOUT_MS	3000
#define TCM_RESET_ASSERT_MS	20
#define TCM_STARTUP_SETTLE_MS	100
#define TCM_BUF_SIZE		512

struct tcm_data {
	struct spi_device *spi;
	struct input_dev *input;
	struct regulator *vdd;
	struct gpio_desc *reset;
	int irq;

	struct mutex io_lock;	/* serializes SPI transfers */
	struct mutex cmd_lock;	/* serializes command/response pairs */
	struct completion cmd_done;
	bool cmd_pending;
	u8 resp_code;
	u8 resp[TCM_BUF_SIZE];
	unsigned int resp_len;

	unsigned int max_x;
	unsigned int max_y;
	unsigned int max_objects;
	u8 *prev_status;

	bool swap_axes;
	bool invert_x;
	bool invert_y;

	u8 *msg_buf;
	u8 *cmd_buf;
	u8 *ff_buf;
};

static u32 tcm_get_bits(const u8 *buf, unsigned int offset, unsigned int bits)
{
	unsigned int byte = offset / 8;
	unsigned int bit = offset % 8;
	u32 val = 0;
	unsigned int i;

	for (i = 0; i < bits; i++) {
		if (buf[byte] & BIT(bit))
			val |= BIT(i);
		if (++bit == 8) {
			bit = 0;
			byte++;
		}
	}

	return val;
}

static int tcm_spi_read(struct tcm_data *ts, u8 *rx, unsigned int len)
{
	struct spi_transfer xfer = {
		.tx_buf = ts->ff_buf,
		.rx_buf = rx,
		.len = len,
	};
	struct spi_message msg;
	int ret;

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	mutex_lock(&ts->io_lock);
	ret = spi_sync(ts->spi, &msg);
	mutex_unlock(&ts->io_lock);

	return ret;
}

static int tcm_spi_write(struct tcm_data *ts, const u8 *tx, unsigned int len)
{
	struct spi_transfer xfer = {
		.tx_buf = tx,
		.len = len,
	};
	struct spi_message msg;
	int ret;

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	mutex_lock(&ts->io_lock);
	ret = spi_sync(ts->spi, &msg);
	mutex_unlock(&ts->io_lock);

	return ret;
}

/*
 * Read one full message into ts->msg_buf and return its code: 4-byte
 * header first to learn the payload length, then the remainder
 * (payload + padding) as a continuation of the same stream.
 */
static int tcm_read_message(struct tcm_data *ts, u8 *code,
			    unsigned int *payload_len)
{
	unsigned int len, total;
	int ret;

	ret = tcm_spi_read(ts, ts->msg_buf, TCM_MSG_HEADER_SIZE);
	if (ret)
		return ret;

	if (ts->msg_buf[0] != TCM_MSG_MARKER)
		return -ENXIO;

	*code = ts->msg_buf[1];
	len = ts->msg_buf[2] | (ts->msg_buf[3] << 8);

	switch (*code) {
	case 0x00: /* STATUS_IDLE */
	case 0x02: /* STATUS_BUSY */
	case 0x03: /* STATUS_CONTINUED_READ */
		*payload_len = 0;
		return 0;
	default:
		break;
	}

	total = TCM_MSG_HEADER_SIZE + len + 1;
	if (total > TCM_BUF_SIZE)
		return -EOVERFLOW;

	ret = tcm_spi_read(ts, &ts->msg_buf[TCM_MSG_HEADER_SIZE],
			   total - TCM_MSG_HEADER_SIZE);
	if (ret)
		return ret;

	if (ts->msg_buf[total - 1] != TCM_MSG_PADDING)
		return -EIO;

	*payload_len = len;
	return 0;
}

static int tcm_send_cmd(struct tcm_data *ts, u8 cmd,
			const u8 *payload, unsigned int paylen,
			u8 expect_code)
{
	int ret;

	if (paylen + 3 > TCM_BUF_SIZE)
		return -EINVAL;

	mutex_lock(&ts->cmd_lock);
	ts->cmd_pending = true;
	reinit_completion(&ts->cmd_done);

	ts->cmd_buf[0] = cmd;
	ts->cmd_buf[1] = paylen & 0xff;
	ts->cmd_buf[2] = (paylen >> 8) & 0xff;
	if (paylen)
		memcpy(&ts->cmd_buf[3], payload, paylen);

	ret = tcm_spi_write(ts, ts->cmd_buf, paylen + 3);
	if (ret)
		goto out;

	ret = wait_for_completion_timeout(&ts->cmd_done,
					  msecs_to_jiffies(TCM_RESPONSE_TIMEOUT_MS));
	if (!ret) {
		ret = -ETIMEDOUT;
		goto out;
	}

	if (ts->resp_code != expect_code) {
		ret = -EIO;
		goto out;
	}

	ret = 0;
out:
	ts->cmd_pending = false;
	mutex_unlock(&ts->cmd_lock);
	return ret;
}

/* Fixed touch report layout programmed below (no gesture bytes) */
static const u8 tcm_report_config[] = {
	1, 4,		/* FOREACH_ACTIVE_OBJECT, OBJ_INDEX:4 */
	7, 4,		/* OBJ_CLASSIFICATION:4 */
	8, 12,		/* OBJ_X:12 */
	9, 12,		/* OBJ_Y:12 */
	11, 16,	/* OBJ_X_WIDTH:16 */
	12, 16,	/* OBJ_Y_WIDTH:16 */
	10, 16,	/* OBJ_Z:16 */
	3,		/* FOREACH_END */
	0,		/* END */
};

static void tcm_apply_transform(struct tcm_data *ts, unsigned int *x,
				unsigned int *y)
{
	unsigned int tmp;

	if (ts->swap_axes) {
		tmp = *x;
		*x = *y;
		*y = tmp;
	}
	if (ts->invert_x)
		*x = ts->max_x - *x;
	if (ts->invert_y)
		*y = ts->max_y - *y;
}

static void tcm_report_touch(struct tcm_data *ts, const u8 *payload,
			     unsigned int len)
{
	unsigned int nobj = len * 8 / TCM_OBJ_BITS;
	unsigned int i, touch_count = 0;
	unsigned int bit = 0;
	unsigned int idx, status, x, y, xw, yw, z;

	for (i = 0; i < nobj; i++, bit += TCM_OBJ_BITS) {
		idx = tcm_get_bits(payload, bit, 4);
		status = tcm_get_bits(payload, bit + 4, 4);
		x = tcm_get_bits(payload, bit + 8, 12);
		y = tcm_get_bits(payload, bit + 20, 12);
		xw = tcm_get_bits(payload, bit + 32, 16);
		yw = tcm_get_bits(payload, bit + 48, 16);
		z = tcm_get_bits(payload, bit + 64, 16);

		if (idx >= ts->max_objects)
			continue;

		if (status == TCM_OBJ_LIFT &&
		    ts->prev_status[idx] == TCM_OBJ_LIFT)
			continue;

		tcm_apply_transform(ts, &x, &y);

		input_mt_slot(ts->input, idx);
		if (status == TCM_OBJ_LIFT) {
			input_mt_report_slot_state(ts->input, MT_TOOL_FINGER,
						   false);
		} else if (status == TCM_OBJ_FINGER ||
			   status == TCM_OBJ_GLOVED_FINGER) {
			input_mt_report_slot_state(ts->input, MT_TOOL_FINGER,
						   true);
			input_report_key(ts->input, BTN_TOUCH, 1);
			input_report_key(ts->input, BTN_TOOL_FINGER, 1);
			input_report_abs(ts->input, ABS_MT_POSITION_X, x);
			input_report_abs(ts->input, ABS_MT_POSITION_Y, y);
			input_report_abs(ts->input, ABS_MT_PRESSURE, z);
			input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR,
					 (xw + yw) / 2);
			touch_count++;
		} else {
			continue;
		}

		ts->prev_status[idx] = status;
	}

	if (!touch_count) {
		input_report_key(ts->input, BTN_TOUCH, 0);
		input_report_key(ts->input, BTN_TOOL_FINGER, 0);
	}

	input_sync(ts->input);
}

static void tcm_release_all(struct tcm_data *ts)
{
	unsigned int i;

	for (i = 0; i < ts->max_objects; i++) {
		input_mt_slot(ts->input, i);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);
		ts->prev_status[i] = TCM_OBJ_LIFT;
	}

	input_report_key(ts->input, BTN_TOUCH, 0);
	input_report_key(ts->input, BTN_TOOL_FINGER, 0);
	input_sync(ts->input);
}

static irqreturn_t tcm_irq_thread(int irq, void *data)
{
	struct tcm_data *ts = data;
	unsigned int len = 0;
	u8 code;
	int ret;

	ret = tcm_read_message(ts, &code, &len);
	if (ret)
		return IRQ_HANDLED;

	/*
	 * Command responses carry codes < 0x10; reports (>= 0x10) are
	 * never mistaken for responses, even if one arrives mid-command.
	 */
	if (ts->cmd_pending && code < TCM_REPORT_IDENTIFY) {
		if (len > sizeof(ts->resp))
			len = sizeof(ts->resp);
		memcpy(ts->resp, &ts->msg_buf[TCM_MSG_HEADER_SIZE], len);
		ts->resp_len = len;
		ts->resp_code = code;
		complete(&ts->cmd_done);
	} else if (code == TCM_REPORT_TOUCH) {
		tcm_report_touch(ts, &ts->msg_buf[TCM_MSG_HEADER_SIZE], len);
	}

	return IRQ_HANDLED;
}

static int tcm_identify(struct tcm_data *ts)
{
	u32 build_id;
	int ret;

	ret = tcm_send_cmd(ts, TCM_CMD_IDENTIFY, NULL, 0, TCM_STATUS_OK);
	if (ret) {
		/*
		 * Some firmware revisions answer the identify report
		 * directly instead of STATUS_OK.
		 */
		if (ts->resp_code != TCM_REPORT_IDENTIFY ||
		    ts->resp_len < TCM_ID_MIN_LEN)
			return ret;
	}

	if (ts->resp_len < TCM_ID_MIN_LEN ||
	    ts->resp[TCM_ID_MODE_OFF] != TCM_MODE_APPLICATION)
		return -ENODEV;

	build_id = get_unaligned_le32(&ts->resp[TCM_ID_BUILD_ID_OFF]);
	dev_info(&ts->spi->dev, "application firmware, build id %u\n",
		 build_id);
	return 0;
}

static int tcm_get_app_info(struct tcm_data *ts)
{
	unsigned int cfg_size;
	int ret;

	ret = tcm_send_cmd(ts, TCM_CMD_GET_APPLICATION_INFO, NULL, 0,
			   TCM_STATUS_OK);
	if (ret)
		return ret;

	if (ts->resp_len < TCM_APP_INFO_MIN_LEN)
		return -EIO;

	if (get_unaligned_le16(&ts->resp[TCM_APP_STATUS_OFF]) != TCM_APP_STATUS_OK)
		return -EIO;

	ts->max_x = get_unaligned_le16(&ts->resp[TCM_APP_MAX_X_OFF]);
	ts->max_y = get_unaligned_le16(&ts->resp[TCM_APP_MAX_Y_OFF]);
	ts->max_objects = get_unaligned_le16(&ts->resp[TCM_APP_MAX_OBJECTS_OFF]);
	cfg_size = get_unaligned_le16(&ts->resp[TCM_APP_MAX_CFG_SIZE_OFF]);

	if (!ts->max_x || !ts->max_y || !ts->max_objects ||
	    cfg_size < sizeof(tcm_report_config) ||
	    cfg_size > TCM_BUF_SIZE - 3)
		return -EINVAL;

	return cfg_size;
}

static int tcm_set_report_config(struct tcm_data *ts, unsigned int cfg_size)
{
	u8 *cfg;
	int ret;

	cfg = kzalloc(cfg_size, GFP_KERNEL);
	if (!cfg)
		return -ENOMEM;

	memcpy(cfg, tcm_report_config, sizeof(tcm_report_config));

	ret = tcm_send_cmd(ts, TCM_CMD_SET_TOUCH_REPORT_CONFIG,
			   cfg, cfg_size, TCM_STATUS_OK);
	kfree(cfg);
	return ret;
}

static int tcm_setup_input(struct tcm_data *ts)
{
	struct input_dev *input;
	int ret;

	ts->prev_status = devm_kzalloc(&ts->spi->dev, ts->max_objects,
				       GFP_KERNEL);
	if (!ts->prev_status)
		return -ENOMEM;

	input = devm_input_allocate_device(&ts->spi->dev);
	if (!input)
		return -ENOMEM;

	input->name = "omnivision_tcm-modules_touch";
	input->id.bustype = BUS_SPI;
	input->dev.parent = &ts->spi->dev;

	ts->swap_axes = device_property_read_bool(&ts->spi->dev,
						  "touchscreen-swapped-x-y");
	ts->invert_x = device_property_read_bool(&ts->spi->dev,
						 "touchscreen-inverted-x");
	ts->invert_y = device_property_read_bool(&ts->spi->dev,
						 "touchscreen-inverted-y");

	__set_bit(EV_SYN, input->evbit);
	__set_bit(EV_KEY, input->evbit);
	__set_bit(EV_ABS, input->evbit);
	__set_bit(BTN_TOUCH, input->keybit);
	__set_bit(BTN_TOOL_FINGER, input->keybit);
	__set_bit(INPUT_PROP_DIRECT, input->propbit);

	input_set_abs_params(input, ABS_MT_POSITION_X, 0, ts->max_x, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, ts->max_y, 0, 0);
	input_set_abs_params(input, ABS_MT_PRESSURE, 0, TCM_MAX_PRESSURE,
			     0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, TCM_MAX_TOUCH_MAJOR,
			     0, 0);

	ret = input_mt_init_slots(input, ts->max_objects, INPUT_MT_DIRECT);
	if (ret)
		return ret;

	ret = input_register_device(input);
	if (ret)
		return ret;

	ts->input = input;
	return 0;
}

static int omnivision_tcm_spi_probe(struct spi_device *spi)
{
	struct tcm_data *ts;
	unsigned int cfg_size;
	int ret;
	int irq;

	if (spi->controller->flags & SPI_CONTROLLER_HALF_DUPLEX) {
		dev_err(&spi->dev, "full duplex SPI required\n");
		return -EIO;
	}

	ts = devm_kzalloc(&spi->dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->spi = spi;
	ts->msg_buf = devm_kmalloc(&spi->dev, TCM_BUF_SIZE, GFP_KERNEL);
	ts->cmd_buf = devm_kmalloc(&spi->dev, TCM_BUF_SIZE, GFP_KERNEL);
	ts->ff_buf = devm_kmalloc(&spi->dev, TCM_BUF_SIZE, GFP_KERNEL);
	if (!ts->msg_buf || !ts->cmd_buf || !ts->ff_buf)
		return -ENOMEM;
	memset(ts->ff_buf, 0xff, TCM_BUF_SIZE);

	mutex_init(&ts->io_lock);
	mutex_init(&ts->cmd_lock);
	init_completion(&ts->cmd_done);

	spi_set_drvdata(spi, ts);

	ts->vdd = devm_regulator_get_optional(&spi->dev, "vdd");
	if (IS_ERR(ts->vdd)) {
		ret = PTR_ERR(ts->vdd);
		if (ret != -ENODEV)
			return dev_err_probe(&spi->dev, ret,
					     "failed to get vdd supply\n");
		ts->vdd = NULL;
	}

	if (ts->vdd) {
		ret = regulator_enable(ts->vdd);
		if (ret)
			return dev_err_probe(&spi->dev, ret,
					     "failed to enable vdd\n");
	}

	/* Reset is asserted (active) then released; polarity from DT */
	ts->reset = devm_gpiod_get_optional(&spi->dev, "reset",
					    GPIOD_OUT_HIGH);
	if (IS_ERR(ts->reset)) {
		ret = PTR_ERR(ts->reset);
		goto err_regulator;
	}
	if (ts->reset) {
		msleep(TCM_RESET_ASSERT_MS);
		gpiod_set_value(ts->reset, 0);
		msleep(TCM_STARTUP_SETTLE_MS);
	}

	irq = spi->irq;
	if (irq <= 0) {
		dev_err(&spi->dev, "no IRQ configured\n");
		ret = -ENODEV;
		goto err_regulator;
	}
	ts->irq = irq;

	ret = devm_request_threaded_irq(&spi->dev, irq, NULL, tcm_irq_thread,
					IRQF_ONESHOT, "omnivision_tcm_spi", ts);
	if (ret)
		goto err_regulator;

	ret = tcm_identify(ts);
	if (ret) {
		dev_err(&spi->dev, "controller not in application mode\n");
		goto err_regulator;
	}

	ret = tcm_get_app_info(ts);
	if (ret < 0) {
		dev_err(&spi->dev, "failed to get application info\n");
		goto err_regulator;
	}
	cfg_size = ret;

	dev_info(&spi->dev, "touchscreen %ux%u, %u objects\n",
		 ts->max_x, ts->max_y, ts->max_objects);

	ret = tcm_set_report_config(ts, cfg_size);
	if (ret) {
		dev_err(&spi->dev, "failed to set touch report config\n");
		goto err_regulator;
	}

	ret = tcm_setup_input(ts);
	if (ret)
		goto err_regulator;

	ret = tcm_send_cmd(ts, TCM_CMD_ENABLE_REPORT, NULL, 0, TCM_STATUS_OK);
	if (ret) {
		dev_err(&spi->dev, "failed to enable touch reports\n");
		goto err_input;
	}

	return 0;

err_input:
	input_unregister_device(ts->input);
err_regulator:
	if (ts->vdd)
		regulator_disable(ts->vdd);
	return ret;
}

static void omnivision_tcm_spi_remove(struct spi_device *spi)
{
	struct tcm_data *ts = spi_get_drvdata(spi);

	input_unregister_device(ts->input);
	if (ts->vdd)
		regulator_disable(ts->vdd);
}

static int omnivision_tcm_spi_suspend(struct device *dev)
{
	struct tcm_data *ts = dev_get_drvdata(dev);
	int ret;

	ret = tcm_send_cmd(ts, TCM_CMD_DISABLE_REPORT, NULL, 0,
			   TCM_STATUS_OK);
	if (ret)
		return ret;

	disable_irq(ts->irq);
	tcm_release_all(ts);
	return 0;
}

static int omnivision_tcm_spi_resume(struct device *dev)
{
	struct tcm_data *ts = dev_get_drvdata(dev);
	int ret;

	enable_irq(ts->irq);

	ret = tcm_send_cmd(ts, TCM_CMD_ENABLE_REPORT, NULL, 0, TCM_STATUS_OK);
	if (ret) {
		disable_irq(ts->irq);
		return ret;
	}

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(omnivision_tcm_spi_pm,
				omnivision_tcm_spi_suspend, omnivision_tcm_spi_resume);

static const struct of_device_id omnivision_tcm_spi_of_match[] = {
	{ .compatible = "omnivision_tcm-spi" },
	{ }
};
MODULE_DEVICE_TABLE(of, omnivision_tcm_spi_of_match);

static const struct spi_device_id omnivision_tcm_spi_id_table[] = {
	{ "omnivision_tcm_spi", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, omnivision_tcm_spi_id_table);

static struct spi_driver omnivision_tcm_spi_driver = {
	.driver = {
		.name = "omnivision_tcm_spi",
		.of_match_table = omnivision_tcm_spi_of_match,
		.pm = pm_sleep_ptr(&omnivision_tcm_spi_pm),
	},
	.probe = omnivision_tcm_spi_probe,
	.remove = omnivision_tcm_spi_remove,
	.id_table = omnivision_tcm_spi_id_table,
};
module_spi_driver(omnivision_tcm_spi_driver);

MODULE_DESCRIPTION("Omnivision TouchComm touchscreen driver (SPI)");
MODULE_LICENSE("GPL");
