// SPDX-License-Identifier: GPL-2.0
/*
 * Unisoc camera frontend driver - configuration node
 *
 * Copyright (C) 2025 Otto Pflüger
 */

#include <media/v4l2-ioctl.h>
#include <media/v4l2-isp.h>
#include <media/v4l2-mc.h>
#include <media/videobuf2-vmalloc.h>

#include "camsys.h"

#define to_sprd_config_vdev(q) \
	container_of(q, struct sprd_config_vdev, queue)

#define to_sprd_config_buf(b) \
	container_of(b, struct sprd_config_buffer, vbuf)

static const struct v4l2_isp_params_block_type_info
sprd_dcam_config_block_types_info[] = {
	[SPRD_DCAM_BLOCK_AE] = {
		.size = sizeof(struct sprd_camsys_ae_config),
	},
	[SPRD_DCAM_BLOCK_AF] = {
		.size = sizeof(struct sprd_camsys_af_config),
	},
	[SPRD_DCAM_BLOCK_AWB] = {
		.size = sizeof(struct sprd_camsys_awb_config),
	},
	[SPRD_DCAM_BLOCK_BLC] = {
		.size = sizeof(struct sprd_camsys_blc_config),
	},
	[SPRD_DCAM_BLOCK_LSC] = {
		.size = sizeof(struct sprd_camsys_lsc_config),
	},
};

static int sprd_config_querycap(struct file *file, void *fh,
				struct v4l2_capability *cap)
{
	strscpy(cap->driver, "sprd-camsys", sizeof(cap->driver));
	strscpy(cap->card, "Unisoc Camera Subsystem", sizeof(cap->card));

	return 0;
}

static int sprd_config_enum_fmt(struct file *file, void *fh,
				struct v4l2_fmtdesc *f)
{
	if (f->index > 0)
		return -EINVAL;

	f->pixelformat = V4L2_META_FMT_SPRD_DCAM_CFG;

	return 0;
}

static int sprd_config_g_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	static const struct v4l2_meta_format mfmt = {
		.dataformat = V4L2_META_FMT_SPRD_DCAM_CFG,
		.buffersize = v4l2_isp_params_buffer_size(SPRD_DCAM_MAX_CONFIG_SIZE),
	};

	f->fmt.meta = mfmt;

	return 0;
}

static int sprd_config_queue_setup(struct vb2_queue *q,
				   unsigned int *num_buffers,
				   unsigned int *num_planes,
				   unsigned int sizes[],
				   struct device *alloc_devs[])
{
	if (*num_planes && *num_planes > 1)
		return -EINVAL;

	if (sizes[0] && sizes[0] < v4l2_isp_params_buffer_size(SPRD_DCAM_MAX_CONFIG_SIZE))
		return -EINVAL;

	*num_planes = 1;

	if (!sizes[0])
		sizes[0] = v4l2_isp_params_buffer_size(SPRD_DCAM_MAX_CONFIG_SIZE);

	return 0;
}

static int sprd_config_buf_init(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct sprd_config_buffer *buf = to_sprd_config_buf(vbuf);

	buf->config = kvmalloc(v4l2_isp_params_buffer_size(SPRD_DCAM_MAX_CONFIG_SIZE),
			       GFP_KERNEL);
	if (!buf->config)
		return -ENOMEM;

	return 0;
}

static void sprd_config_buf_cleanup(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct sprd_config_buffer *buf = to_sprd_config_buf(vbuf);

	kvfree(buf->config);
	buf->config = NULL;
}

static int sprd_config_buf_prepare(struct vb2_buffer *vb)
{
	struct sprd_config_vdev *cfg = to_sprd_config_vdev(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct sprd_config_buffer *buf = to_sprd_config_buf(vbuf);
	struct v4l2_isp_params_buffer *config = vb2_plane_vaddr(vb, 0);
	size_t buffer_size = v4l2_isp_params_buffer_size(SPRD_DCAM_MAX_CONFIG_SIZE);
	int ret;

	ret = v4l2_isp_params_validate_buffer_size(&cfg->camsys->pdev->dev, vb, buffer_size);
	if (ret)
		return ret;

	memcpy(buf->config, config, buffer_size);

	return v4l2_isp_params_validate_buffer(&cfg->camsys->pdev->dev, vb, buf->config,
					       sprd_dcam_config_block_types_info,
					       ARRAY_SIZE(sprd_dcam_config_block_types_info));
}

static void sprd_config_buf_queue(struct vb2_buffer *vb)
{
	struct sprd_config_vdev *cfg = to_sprd_config_vdev(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct sprd_config_buffer *buf = to_sprd_config_buf(vbuf);
	unsigned long flags;

	spin_lock_irqsave(&cfg->buf_lock, flags);
	list_add_tail(&buf->link, &cfg->buf_queue);
	spin_unlock_irqrestore(&cfg->buf_lock, flags);
}

static void sprd_config_stop_streaming(struct vb2_queue *q)
{
	struct sprd_config_vdev *cfg = to_sprd_config_vdev(q);
	struct sprd_config_buffer *buf;
	unsigned long flags;

	spin_lock_irqsave(&cfg->buf_lock, flags);

	while (!list_empty(&cfg->buf_queue)) {
		buf = list_first_entry(&cfg->buf_queue, struct sprd_config_buffer, link);
		list_del(&buf->link);
		vb2_buffer_done(&buf->vbuf.vb2_buf, VB2_BUF_STATE_ERROR);
	}

	spin_unlock_irqrestore(&cfg->buf_lock, flags);
}

static const struct v4l2_ioctl_ops sprd_config_ioctl_ops = {
	.vidioc_querycap		= sprd_config_querycap,
	.vidioc_enum_fmt_meta_out	= sprd_config_enum_fmt,
	.vidioc_g_fmt_meta_out		= sprd_config_g_fmt,
	.vidioc_s_fmt_meta_out		= sprd_config_g_fmt,
	.vidioc_try_fmt_meta_out	= sprd_config_g_fmt,
	.vidioc_reqbufs			= vb2_ioctl_reqbufs,
	.vidioc_querybuf		= vb2_ioctl_querybuf,
	.vidioc_qbuf			= vb2_ioctl_qbuf,
	.vidioc_expbuf			= vb2_ioctl_expbuf,
	.vidioc_dqbuf			= vb2_ioctl_dqbuf,
	.vidioc_create_bufs		= vb2_ioctl_create_bufs,
	.vidioc_prepare_buf		= vb2_ioctl_prepare_buf,
	.vidioc_streamon		= vb2_ioctl_streamon,
	.vidioc_streamoff		= vb2_ioctl_streamoff,
};

static const struct v4l2_file_operations sprd_config_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= video_ioctl2,
	.open		= v4l2_fh_open,
	.release	= vb2_fop_release,
	.poll		= vb2_fop_poll,
	.mmap		= vb2_fop_mmap,
};

static const struct vb2_ops sprd_config_vb2_ops = {
	.queue_setup	= sprd_config_queue_setup,
	.buf_init	= sprd_config_buf_init,
	.buf_cleanup	= sprd_config_buf_cleanup,
	.buf_prepare	= sprd_config_buf_prepare,
	.buf_queue	= sprd_config_buf_queue,
	.stop_streaming	= sprd_config_stop_streaming,
};

int sprd_config_vdev_register(struct sprd_config_vdev *cfg)
{
	struct sprd_camsys *cs = cfg->camsys;
	struct video_device *vdev = &cfg->vdev;
	struct vb2_queue *q = &cfg->queue;
	int ret;

	mutex_init(&cfg->lock);
	spin_lock_init(&cfg->buf_lock);
	INIT_LIST_HEAD(&cfg->buf_queue);

	/* vdev->name is set by the caller */
	vdev->device_caps = V4L2_CAP_META_OUTPUT | V4L2_CAP_STREAMING;
	vdev->v4l2_dev = &cs->v4l2_dev;
	vdev->fops = &sprd_config_fops;
	vdev->release = video_device_release_empty;
	vdev->ioctl_ops = &sprd_config_ioctl_ops;
	vdev->lock = &cfg->lock;
	vdev->vfl_dir = VFL_DIR_TX;

	video_set_drvdata(vdev, cfg);

	q->type = V4L2_BUF_TYPE_META_OUTPUT;
	q->io_modes = VB2_DMABUF | VB2_MMAP;
	q->ops = &sprd_config_vb2_ops;
	q->mem_ops = &vb2_vmalloc_memops;
	q->buf_struct_size = sizeof(struct sprd_config_buffer);
	q->min_queued_buffers = 1;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->lock = &cfg->lock;
	q->dev = &cs->pdev->dev;
	ret = vb2_queue_init(q);
	if (ret) {
		dev_err(&cs->pdev->dev, "failed to init vb2 queue: %d\n", ret);
		goto err_destroy_mutex;
	}

	vdev->queue = q;

	cfg->pads[SPRD_CONFIG_PAD_SRC].flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&vdev->entity, SPRD_CONFIG_PAD_NUM, cfg->pads);
	if (ret)
		goto err_release_queue;

	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		dev_err(&cs->pdev->dev, "failed to register %s: %d\n",
			vdev->name, ret);
		goto err_cleanup_entity;
	}

	return 0;

err_cleanup_entity:
	media_entity_cleanup(&vdev->entity);
err_release_queue:
	vb2_queue_release(q);
err_destroy_mutex:
	mutex_destroy(&cfg->lock);
	return ret;
}

void sprd_config_vdev_unregister(struct sprd_config_vdev *cfg)
{
	video_unregister_device(&cfg->vdev);
	media_entity_cleanup(&cfg->vdev.entity);
	vb2_queue_release(&cfg->queue);
	mutex_destroy(&cfg->lock);
}

void sprd_config_next_frame(struct sprd_dcam *dcam)
{
	struct sprd_config_vdev *cfg = &dcam->config;
	struct v4l2_isp_params_buffer *config;
	struct sprd_config_buffer *buf;
	size_t block_offset = 0;
	size_t max_offset;

	spin_lock(&cfg->buf_lock);
	buf = list_first_entry_or_null(&cfg->buf_queue, struct sprd_config_buffer, link);
	if (buf)
		list_del(&buf->link);
	spin_unlock(&cfg->buf_lock);

	if (!buf)
		return;

	config = buf->config;

	max_offset = config->data_size;

	/*
	 * Walk the list of parameter blocks and process them. No validation is
	 * done here, as the contents of the config buffer are already checked
	 * when the buffer is queued.
	 */
	while (max_offset && block_offset < max_offset) {
		union sprd_camsys_config_block block;

		block.data = &config->data[block_offset];

		if (dcam->hw->config_block[block.header->type]) {
			dev_dbg(dcam->dev, "configuring block %d\n", block.header->type);
			dcam->hw->config_block[block.header->type](dcam, block);
		} else {
			dev_dbg(dcam->dev, "block %d not supported by HW\n", block.header->type);
		}

		block_offset += block.header->size;
	}

	buf->vbuf.vb2_buf.timestamp = ktime_get_ns();
	buf->vbuf.sequence = dcam->sequence;
	vb2_buffer_done(&buf->vbuf.vb2_buf, VB2_BUF_STATE_DONE);
}
