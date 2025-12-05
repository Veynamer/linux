/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __SPRD_SHUB_H__
#define __SPRD_SHUB_H__

#include <linux/iio/iio.h>

struct sprd_shub_sensor_priv {
	struct sprd_shub_sensor *sensor;
	void (*process)(struct iio_dev *iio_dev, void *data, u64 timestamp);

	unsigned long scan_mask;
	int freq_idx;
	s32 calib_data[32];
};

extern const struct iio_buffer_setup_ops sprd_shub_buffer_ops;

const struct iio_mount_matrix *
sprd_shub_get_mount_matrix(const struct iio_dev *iio_dev,
			   const struct iio_chan_spec *chan);

int sprd_shub_calib_read(struct iio_dev *iio_dev, int idx, int *val);
int sprd_shub_calib_write(struct iio_dev *iio_dev, int idx, int val);

int sprd_shub_read_freq(struct iio_dev *iio_dev, int *val, int *val2);
int sprd_shub_write_freq(struct iio_dev *iio_dev, int val, int val2);
int sprd_shub_read_avail(struct iio_dev *iio_dev,
			 struct iio_chan_spec const *chan,
			 const int **vals, int *type,
			 int *length, long mask);

#define SPRD_SHUB_FIXED_SCALE	256

void sprd_shub_process_xyz(struct iio_dev *iio_dev, void *data, u64 timestamp);
void sprd_shub_process_single(struct iio_dev *iio_dev, void *data, u64 timestamp);

#endif /* __SPRD_SHUB_H__ */
