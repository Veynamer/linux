// SPDX-License-Identifier: GPL-2.0
/*
 * Unisoc sensor hub IIO driver
 *
 * Copyright (C) 2025 Otto Pflüger
 */

#include <linux/iio/buffer.h>
#include <linux/iio/common/sprd_shub.h>
#include <linux/platform_device.h>
#include <linux/rpmsg.h>
#include <linux/unaligned.h>

#define SHUB_MSG_MAGIC_CHAR		'~'
#define SHUB_MSG_MAGIC_COUNT		4

enum {
	SHUB_ACCELEROMETER_SENSOR = 1,
	SHUB_GEOMAGNETIC_SENSOR = 2,
	SHUB_ORIENTATION_SENSOR = 3,
	SHUB_GYROSCOPE_SENSOR = 4,
	SHUB_LIGHT_SENSOR = 5,
	SHUB_PROXIMITY_SENSOR = 8,
};

struct shub_msg_hdr {
	u8 magic[SHUB_MSG_MAGIC_COUNT];
	u8 sensor;
	u8 type;
	__be16 length;
	__be16 checksum;
} __packed;

struct shub_batch_cmd {
	__le32 sample_interval_us;
	__le32 batch_interval_us;
	__le32 reserved;
} __packed;

struct shub_data_evt {
	u8 handle;
	u8 reserved;
	u8 data[24];
	__le64 timestamp;
} __packed;

struct shub_sensor_info {
	char model[20];
	char manufacturer[20];
	__le32 version;
	__le32 handle;
	__le32 maxrange;
	__le32 resolution;
	__le32 power;
	__le32 mindelay_us;
	__le32 fifo_reserved_event_count;
	__le32 fifo_max_event_count;
	__le32 maxdelay_us;
} __packed;

struct shub_timesync_cmd {
	__le64 boottime;
	__le64 systimer_cnt;
	__le64 sysfrt_cnt;
	__le32 systimer_mult;
	__le32 systimer_shift;
	__le32 sysfrt_mult;
	__le32 sysfrt_shift;
} __packed;

enum {
	SHUB_CMD_ENABLE = 0x00,
	SHUB_CMD_DISABLE = 0x01,
	SHUB_CMD_SET_BATCH = 0x02,
	SHUB_EVT_DATA = 0x04,
	SHUB_CMD_GET_SENSOR_INFO = 0x09,
	SHUB_CMD_SET_CALIB_DATA = 0xc2,
	SHUB_CMD_START_CALIBRATION = 0xc3,
	SHUB_CMD_SET_TIMESYNC = 0xc4,
	SHUB_CMD_GET_CALIB_DATA = 0xc6,
	SHUB_CMD_GET_FWVERSION = 0xcd,
};

enum shub_msg_rx_state {
	SHUB_MSG_RX_SYNC,
	SHUB_MSG_RX_HEADER,
	SHUB_MSG_RX_DATA,
};

struct sprd_shub {
	struct device *dev;
	struct rpmsg_endpoint *ept;

	enum shub_msg_rx_state rx_state;
	size_t rx_pos;
	struct shub_msg_hdr rx_hdr;
	u8 rx_data[256];

	struct delayed_work init_work;
	spinlock_t lock;
	struct idr sensors;
};

struct sprd_shub_sensor {
	struct sprd_shub *shub;
	struct platform_device *pdev;
	struct work_struct reg_work;
	u8 id;
};

static u16 shub_msg_checksum(const void *data, size_t len)
{
	const __be16 *buf = data;
	u32 sum = 0;

	while (len >= 2) {
		sum += be16_to_cpu(*buf++);
		len -= 2;
	}

	if (len == 1)
		sum += be16_to_cpu(*buf) & 0xff00;

	sum = (sum & 0xffff) + (sum >> 16);
	sum = (sum & 0xffff) + (sum >> 16);

	return ~sum;
}

static const char *shub_sensor_type_names[] = {
	[SHUB_ACCELEROMETER_SENSOR] = "sprd-shub-accel",
	[SHUB_GEOMAGNETIC_SENSOR] = "sprd-shub-magn",
	[SHUB_ORIENTATION_SENSOR] = "sprd-shub-compass",
	[SHUB_GYROSCOPE_SENSOR] = "sprd-shub-gyro",
	[SHUB_LIGHT_SENSOR] = "sprd-shub-light",
	[SHUB_PROXIMITY_SENSOR] = "sprd-shub-proximity",
};

static void shub_sensor_register(struct work_struct *work)
{
	struct sprd_shub_sensor *sensor =
		container_of(work, struct sprd_shub_sensor, reg_work);
	struct sprd_shub *shub = sensor->shub;
	struct platform_device *pdev;
	const char *name = NULL;

	if (sensor->id < ARRAY_SIZE(shub_sensor_type_names))
		name = shub_sensor_type_names[sensor->id];

	if (!name) {
		dev_warn(shub->dev, "unknown sensor ID %d\n", sensor->id);
		return;
	}

	pdev = platform_device_register_data(shub->dev, name, sensor->id,
					     &sensor, sizeof(sensor));
	if (IS_ERR(pdev))
		dev_err(shub->dev, "failed to register %s: %pe\n", name, pdev);

	spin_lock_irq(&shub->lock);
	sensor->pdev = pdev;
	spin_unlock_irq(&shub->lock);
}

static void shub_add_sensor(struct sprd_shub *shub, u8 sensor_id)
{
	struct sprd_shub_sensor *sensor;
	unsigned long flags;
	int ret;

	sensor = kzalloc(sizeof(*sensor), GFP_ATOMIC);
	if (!sensor)
		return;

	sensor->shub = shub;
	sensor->id = sensor_id;
	INIT_WORK(&sensor->reg_work, shub_sensor_register);

	spin_lock_irqsave(&shub->lock, flags);
	ret = idr_alloc(&shub->sensors, sensor, sensor_id, sensor_id + 1, GFP_ATOMIC);
	if (ret == sensor_id) {
		schedule_work(&sensor->reg_work);
	} else {
		dev_err(shub->dev, "sensor %d already allocated\n", sensor_id);
		kfree(sensor);
	}
	spin_unlock_irqrestore(&shub->lock, flags);
}

static void shub_handle_sensor_info(struct sprd_shub *shub)
{
	struct shub_sensor_info *info = (void *)shub->rx_data;
	u8 sensor_id = le32_to_cpu(info->handle);

	shub_add_sensor(shub, sensor_id);
}

static void shub_handle_data(struct sprd_shub *shub)
{
	struct shub_data_evt *evt = (void *)shub->rx_data;
	struct sprd_shub_sensor_priv *priv;
	struct sprd_shub_sensor *sensor;
	struct iio_dev *iio_dev;
	unsigned long flags;

	spin_lock_irqsave(&shub->lock, flags);

	sensor = idr_find(&shub->sensors, evt->handle - 100);
	if (!sensor || !sensor->pdev)
		goto out;

	iio_dev = platform_get_drvdata(sensor->pdev);
	if (!iio_dev)
		goto out;

	priv = iio_priv(iio_dev);
	priv->process(iio_dev, evt->data, le64_to_cpu(evt->timestamp));

out:
	spin_unlock_irqrestore(&shub->lock, flags);
}

static void shub_handle_msg(struct sprd_shub *shub)
{
	dev_dbg(shub->dev, "event: 0x%02x 0x%02x, data size 0x%04x\n",
		shub->rx_hdr.sensor, shub->rx_hdr.type,
		be16_to_cpu(shub->rx_hdr.length));

	switch (shub->rx_hdr.type) {
	case SHUB_EVT_DATA:
		shub_handle_data(shub);
		break;

	case SHUB_CMD_GET_SENSOR_INFO:
		shub_handle_sensor_info(shub);
		break;

	case SHUB_CMD_GET_FWVERSION:
		/* Give the hub some time to initialize */
		schedule_delayed_work(&shub->init_work, 2 * HZ);
		break;
	}
}

static size_t shub_rx_buf(u8 **buf, int *cnt, void *to, size_t *pos,
			  size_t expected, size_t avail)
{
	size_t len = min_t(size_t, *cnt, expected - *pos);

	if (*pos + len <= avail)
		memcpy(to + *pos, *buf, len);
	else if (*pos < avail)
		memcpy(to + *pos, *buf, avail - *pos);

	*pos += len;
	*buf += len;
	*cnt -= len;

	return expected - *pos;
}

static int shub_rpmsg_callback(struct rpmsg_device *rpdev, void *data,
			       int cnt, void *userdata, u32 addr)
{
	struct sprd_shub *shub = userdata;
	u8 *buf = data;

	while (cnt > 0) {
		switch (shub->rx_state) {
		case SHUB_MSG_RX_SYNC:
			while (cnt && shub->rx_pos < SHUB_MSG_MAGIC_COUNT) {
				if (*buf == SHUB_MSG_MAGIC_CHAR)
					shub->rx_pos++;
				else
					shub->rx_pos = 0;
				buf++;
				cnt--;
			}

			if (cnt == 0)
				break;

			shub->rx_state = SHUB_MSG_RX_HEADER;
			fallthrough;

		case SHUB_MSG_RX_HEADER:
			if (shub_rx_buf(&buf, &cnt, &shub->rx_hdr, &shub->rx_pos,
					sizeof(struct shub_msg_hdr),
					sizeof(struct shub_msg_hdr)))
				return 0;

			shub->rx_pos = 0;
			shub->rx_state = SHUB_MSG_RX_DATA;
			break;

		case SHUB_MSG_RX_DATA:
			if (shub_rx_buf(&buf, &cnt, shub->rx_data, &shub->rx_pos,
					be16_to_cpu(shub->rx_hdr.length),
					sizeof(shub->rx_data)))
				return 0;

			shub_handle_msg(shub);

			shub->rx_pos = 0;
			shub->rx_state = SHUB_MSG_RX_SYNC;
			break;
		}
	}

	return 0;
}

static int shub_send_msg(struct sprd_shub *shub, u8 sensor, u8 type,
			 void *data, size_t data_size)
{
	static u8 dummy_data = { 0xff };
	struct shub_msg_hdr hdr;
	__be16 checksum;
	int ret;

	if (data_size > U16_MAX)
		return -EINVAL;

	if (data_size == 0) {
		data = &dummy_data;
		data_size = 1;
	}

	dev_dbg(shub->dev, "send command: 0x%02x 0x%02x, data size 0x%04zx\n",
		sensor, type, data_size);

	memset(hdr.magic, SHUB_MSG_MAGIC_CHAR, SHUB_MSG_MAGIC_COUNT);
	hdr.sensor = sensor;
	hdr.type = type;
	hdr.length = cpu_to_be16(data_size);
	hdr.checksum = cpu_to_be16(shub_msg_checksum(&hdr, sizeof(hdr) - 2));

	ret = rpmsg_send(shub->ept, &hdr, sizeof(hdr));
	if (ret) {
		dev_err(shub->dev, "failed to send command header: %d\n", ret);
		return ret;
	}

	ret = rpmsg_send(shub->ept, data, data_size);
	if (ret) {
		dev_err(shub->dev, "failed to send data: %d\n", ret);
		return ret;
	}

	checksum = cpu_to_be16(shub_msg_checksum(data, data_size));

	ret = rpmsg_send(shub->ept, &checksum, sizeof(checksum));
	if (ret) {
		dev_err(shub->dev, "failed to send data: %d\n", ret);
		return ret;
	}

	return 0;
}

static const int sprd_shub_frequencies[][2] = {
	{   3, 125000 },
	{   6, 250000 },
	{  12, 500000 },
	{  25, 0 },
	{  50, 0 },
	{ 100, 0 },
	{ 200, 0 },
	{ 500, 0 },
};

static const int sprd_shub_intervals[] = {
	320000,
	160000,
	80000,
	40000,
	20000,
	10000,
	5000,
	2000,
};

static int shub_sensor_postenable(struct iio_dev *iio_dev)
{
	struct sprd_shub_sensor_priv *priv = iio_priv(iio_dev);
	struct sprd_shub_sensor *sensor = priv->sensor;
	struct sprd_shub *shub = sensor->shub;
	struct shub_batch_cmd batch_cmd = {
		.sample_interval_us = sprd_shub_intervals[priv->freq_idx],
		.batch_interval_us = 0,
	};
	int ret;

	spin_lock_irq(&shub->lock);
	priv->scan_mask = *iio_dev->active_scan_mask;
	spin_unlock_irq(&shub->lock);

	ret = shub_send_msg(shub, 100 + sensor->id, SHUB_CMD_SET_BATCH,
			    &batch_cmd, sizeof(batch_cmd));
	if (ret)
		return ret;

	return shub_send_msg(shub, 100 + sensor->id, SHUB_CMD_ENABLE, NULL, 0);
}

static int shub_sensor_postdisable(struct iio_dev *iio_dev)
{
	struct sprd_shub_sensor_priv *priv = iio_priv(iio_dev);
	struct sprd_shub_sensor *sensor = priv->sensor;
	struct sprd_shub *shub = sensor->shub;

	return shub_send_msg(shub, 100 + sensor->id, SHUB_CMD_DISABLE, NULL, 0);
}

const struct iio_buffer_setup_ops sprd_shub_buffer_ops = {
	.postenable = &shub_sensor_postenable,
	.postdisable = &shub_sensor_postdisable
};
EXPORT_SYMBOL_GPL(sprd_shub_buffer_ops);

static const struct iio_mount_matrix shub_default_mount_matrix = {
	.rotation = {
		"-1", "0", "0",
		"0", "-1", "0",
		"0", "0", "1",
	},
};

const struct iio_mount_matrix *
sprd_shub_get_mount_matrix(const struct iio_dev *iio_dev,
			   const struct iio_chan_spec *chan)
{
	return &shub_default_mount_matrix;
}
EXPORT_SYMBOL_GPL(sprd_shub_get_mount_matrix);

int sprd_shub_calib_read(struct iio_dev *iio_dev, int idx, int *val)
{
	struct sprd_shub_sensor_priv *priv = iio_priv(iio_dev);

	if (idx >= ARRAY_SIZE(priv->calib_data))
		return -EINVAL;

	*val = priv->calib_data[idx];

	return 0;
}
EXPORT_SYMBOL_GPL(sprd_shub_calib_read);

int sprd_shub_calib_write(struct iio_dev *iio_dev, int idx, int val)
{
	struct sprd_shub_sensor_priv *priv = iio_priv(iio_dev);
	struct sprd_shub_sensor *sensor = priv->sensor;
	struct sprd_shub *shub = sensor->shub;

	if (idx >= ARRAY_SIZE(priv->calib_data))
		return -EINVAL;

	priv->calib_data[idx] = val;

	return shub_send_msg(shub, 100 + sensor->id, SHUB_CMD_SET_CALIB_DATA,
			     priv->calib_data, sizeof(priv->calib_data));
}
EXPORT_SYMBOL_GPL(sprd_shub_calib_write);

int sprd_shub_read_freq(struct iio_dev *iio_dev, int *val, int *val2)
{
	struct sprd_shub_sensor_priv *priv = iio_priv(iio_dev);

	*val = sprd_shub_frequencies[priv->freq_idx][0];
	*val2 = sprd_shub_frequencies[priv->freq_idx][1];

	return IIO_VAL_INT_PLUS_MICRO;
}
EXPORT_SYMBOL_GPL(sprd_shub_read_freq);

int sprd_shub_write_freq(struct iio_dev *iio_dev, int val, int val2)
{
	struct sprd_shub_sensor_priv *priv = iio_priv(iio_dev);
	struct sprd_shub_sensor *sensor = priv->sensor;
	struct sprd_shub *shub = sensor->shub;
	struct shub_batch_cmd batch_cmd = {
		.batch_interval_us = 0,
	};
	int i;

	for (i = 0; i < ARRAY_SIZE(sprd_shub_frequencies); i++) {
		if (sprd_shub_frequencies[i][0] == val &&
		    sprd_shub_frequencies[i][1] == val2) {
			priv->freq_idx = i;
			batch_cmd.sample_interval_us = sprd_shub_intervals[i];
			break;
		}
	}

	if (i == ARRAY_SIZE(sprd_shub_frequencies))
		return -EINVAL;

	return shub_send_msg(shub, 100 + sensor->id, SHUB_CMD_SET_BATCH,
			     &batch_cmd, sizeof(batch_cmd));
}
EXPORT_SYMBOL_GPL(sprd_shub_write_freq);

int sprd_shub_read_avail(struct iio_dev *iio_dev,
			 struct iio_chan_spec const *chan,
			 const int **vals, int *type,
			 int *length, long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*length = ARRAY_SIZE(sprd_shub_frequencies) * 2;
		*vals = &sprd_shub_frequencies[0][0];
		*type = IIO_VAL_INT_PLUS_MICRO;
		return IIO_AVAIL_LIST;
	}

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(sprd_shub_read_avail);

static s32 shub_float_to_int(u32 float32)
{
	int fraction, shift,
	    scale = SPRD_SHUB_FIXED_SCALE,
	    mantissa = float32 & GENMASK(22, 0),
	    sign = (float32 & BIT(31)) ? -1 : 1,
	    exp = (float32 & ~BIT(31)) >> 23;

	/* special case 0 */
	if (!exp && !mantissa)
		return 0;

	exp -= 127;
	if (exp < 0) {
		exp = -exp;
		/* return values ranging from 1 to scale-1 */
		return sign * ((((BIT(23) + mantissa) * scale) >> 23) >> exp);
	}

	/* return values starting at 1*scale */
	shift = 23 - exp;
	float32 = BIT(exp) + (mantissa >> shift);
	fraction = mantissa & GENMASK(shift - 1, 0);

	return sign * (float32 * scale + ((fraction * scale) >> shift));
}

void sprd_shub_process_xyz(struct iio_dev *iio_dev, void *data, u64 timestamp)
{
	struct sprd_shub_sensor_priv *priv = iio_priv(iio_dev);
	s32 buf[6] __aligned(8);
	s32 *p = buf;
	__le32 *flt = data;
	unsigned int i;

	memset(buf, 0, sizeof(buf));

	for_each_set_bit(i, &priv->scan_mask, iio_get_masklength(iio_dev)) {
		if (i < 3)
			*p++ = shub_float_to_int(get_unaligned_le32(&flt[i]));
	}

	iio_push_to_buffers_with_timestamp(iio_dev, buf, timestamp);
}
EXPORT_SYMBOL_GPL(sprd_shub_process_xyz);

void sprd_shub_process_single(struct iio_dev *iio_dev, void *data, u64 timestamp)
{
	struct sprd_shub_sensor_priv *priv = iio_priv(iio_dev);
	s32 buf[4] __aligned(8);

	memset(buf, 0, sizeof(buf));

	if (test_bit(0, &priv->scan_mask))
		buf[0] = shub_float_to_int(get_unaligned_le32(data));

	iio_push_to_buffers_with_timestamp(iio_dev, buf, timestamp);
}
EXPORT_SYMBOL_GPL(sprd_shub_process_single);

static void shub_initialize(struct work_struct *work)
{
	struct sprd_shub *shub = container_of(to_delayed_work(work),
					      struct sprd_shub, init_work);

	/*
	 * This command makes the sensor hub send an info event for
	 * each sensor, which we handle by registering the sensor.
	 */
	shub_send_msg(shub, 0xff, SHUB_CMD_GET_SENSOR_INFO, NULL, 0);
}

static int shub_rpmsg_probe(struct rpmsg_device *rpdev)
{
	struct rpmsg_channel_info chinfo = {
		.src = RPMSG_ADDR_ANY,
		.dst = RPMSG_ADDR_ANY,
	};
	struct shub_timesync_cmd timesync = {
		.systimer_mult = cpu_to_le32(4096000000),
		.systimer_shift = cpu_to_le32(12),
		.sysfrt_mult = cpu_to_le32(4000000000),
		.sysfrt_shift = cpu_to_le32(17),
	};
	struct sprd_shub *shub;
	int ret;

	shub = devm_kzalloc(&rpdev->dev, sizeof(*shub), GFP_KERNEL);
	if (!shub)
		return -ENOMEM;

	shub->dev = &rpdev->dev;
	dev_set_drvdata(&rpdev->dev, shub);

	spin_lock_init(&shub->lock);
	idr_init(&shub->sensors);
	INIT_DELAYED_WORK(&shub->init_work, shub_initialize);

	strscpy(chinfo.name, "shub_main", sizeof(chinfo.name));
	shub->ept = rpmsg_create_ept(rpdev, shub_rpmsg_callback, shub, chinfo);
	if (!shub->ept)
		return -ENOMEM;

	ret = shub_send_msg(shub, 0xff, SHUB_CMD_SET_TIMESYNC,
			    &timesync, sizeof(timesync));
	if (ret)
		return ret;

	ret = shub_send_msg(shub, 0xff, SHUB_CMD_GET_FWVERSION, NULL, 0);
	if (ret)
		return ret;

	/*
	 * Register virtual sensors.
	 * FIXME: These sensors may depend on multiple physical sensors.
	 * If these are not present on some devices, the corresponding
	 * virtual sensors should not be registered.
	 */
	shub_add_sensor(shub, SHUB_ORIENTATION_SENSOR);

	return 0;
}

static void shub_rpmsg_remove(struct rpmsg_device *rpdev)
{
	struct sprd_shub *shub = dev_get_drvdata(&rpdev->dev);
	struct sprd_shub_sensor *sensor;
	int sensor_id;

	rpmsg_destroy_ept(shub->ept);

	idr_for_each_entry(&shub->sensors, sensor, sensor_id) {
		cancel_work_sync(&sensor->reg_work);
		platform_device_unregister(sensor->pdev);
		kfree(sensor);
	}
	idr_destroy(&shub->sensors);
}

static const struct of_device_id shub_of_match[] = {
	{ .compatible = "sprd,shub" },
	{ }
};
MODULE_DEVICE_TABLE(of, shub_of_match);

static struct rpmsg_driver shub_rpmsg_driver = {
	.probe = shub_rpmsg_probe,
	.remove = shub_rpmsg_remove,
	.drv = {
		.name = "sprd-shub",
		.of_match_table = shub_of_match,
	},
};

module_rpmsg_driver(shub_rpmsg_driver);

MODULE_DESCRIPTION("Unisoc sensor hub IIO driver");
MODULE_LICENSE("GPL");
