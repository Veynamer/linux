// SPDX-License-Identifier: GPL-2.0-only
/*
 * Unisoc sensor hub IIO ambient light sensor driver
 *
 * Copyright (C) 2025 Otto Pflüger
 */

#include <linux/mod_devicetable.h>
#include <linux/iio/common/sprd_shub.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/platform_device.h>

static const struct iio_chan_spec shub_light_iio_channels[] = {
	{
		.type = IIO_LIGHT,
		.scan_index = 0,
		.scan_type = {
			.sign = 's',
			.realbits = 32,
			.storagebits = 32,
			.endianness = IIO_CPU,
		},
		.info_mask_separate = BIT(IIO_CHAN_INFO_SCALE) |
				      BIT(IIO_CHAN_INFO_CALIBSCALE) |
				      BIT(IIO_CHAN_INFO_SAMP_FREQ),
	},
	{
		.type = IIO_TIMESTAMP,
		.channel = -1,
		.scan_index = 1,
		.scan_type = {
			.sign = 'u',
			.realbits = 64,
			.storagebits = 64,
			.endianness = IIO_CPU,
		},
	},
};

static int shub_light_read_raw(struct iio_dev *iio_dev,
			       const struct iio_chan_spec *chan, int *val,
			       int *val2, long mask)
{
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_CALIBSCALE:
		ret = sprd_shub_calib_read(iio_dev, 0, val);
		if (ret)
			return ret;

		*val2 = 10000;
		return IIO_VAL_FRACTIONAL;

	case IIO_CHAN_INFO_SCALE:
		*val = 1;
		*val2 = SPRD_SHUB_FIXED_SCALE;
		return IIO_VAL_FRACTIONAL;

	case IIO_CHAN_INFO_SAMP_FREQ:
		return sprd_shub_read_freq(iio_dev, val, val2);
	}

	return -EINVAL;
}

static int shub_light_write_raw(struct iio_dev *iio_dev,
				const struct iio_chan_spec *chan, int val,
				int val2, long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_CALIBSCALE:
		return sprd_shub_calib_write(iio_dev, 0, val);

	case IIO_CHAN_INFO_SAMP_FREQ:
		return sprd_shub_write_freq(iio_dev, val, val2);
	}

	return -EINVAL;
}

static const struct iio_info shub_light_iio_info = {
	.read_raw = shub_light_read_raw,
	.write_raw = shub_light_write_raw,
	.read_avail = sprd_shub_read_avail,
};

static int sprd_shub_light_probe(struct platform_device *pdev)
{
	struct iio_dev *iio_dev;
	struct sprd_shub_sensor_priv *priv;
	int ret;

	iio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*priv));
	if (!iio_dev)
		return -ENOMEM;

	priv = iio_priv(iio_dev);
	priv->sensor = *(struct sprd_shub_sensor **)pdev->dev.platform_data;
	priv->process = sprd_shub_process_single;

	iio_dev->name = "sprd-shub-light";
	iio_dev->info = &shub_light_iio_info;
	iio_dev->channels = shub_light_iio_channels;
	iio_dev->num_channels = ARRAY_SIZE(shub_light_iio_channels);

	ret = devm_iio_kfifo_buffer_setup(&pdev->dev, iio_dev, &sprd_shub_buffer_ops);
	if (ret)
		return ret;

	ret = devm_iio_device_register(&pdev->dev, iio_dev);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, iio_dev);

	return 0;
}

static void sprd_shub_light_remove(struct platform_device *pdev)
{
	platform_set_drvdata(pdev, NULL);
}

static const struct platform_device_id sprd_shub_light_ids[] = {
	{ .name = "sprd-shub-light" },
	{ }
};
MODULE_DEVICE_TABLE(platform, sprd_shub_light_ids);

static struct platform_driver sprd_shub_light_driver = {
	.probe = sprd_shub_light_probe,
	.remove = sprd_shub_light_remove,
	.driver	= {
		.name = "sprd_shub_light",
	},
	.id_table = sprd_shub_light_ids,
};
module_platform_driver(sprd_shub_light_driver);

MODULE_DESCRIPTION("Unisoc sensor hub ambient light sensor driver");
MODULE_LICENSE("GPL");
