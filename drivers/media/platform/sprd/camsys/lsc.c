// SPDX-License-Identifier: GPL-2.0
/*
 * Unisoc camera frontend driver - lens shading table node
 *
 * Copyright (C) 2025 Otto Pflüger
 */

#include <media/v4l2-ioctl.h>
#include <media/v4l2-mc.h>
#include <media/videobuf2-dma-contig.h>

#include "camsys.h"

#define SPRD_CAMSYS_LSC_BUF_SIZE	0x3000

#define to_sprd_lsc_vdev(q) \
	container_of(q, struct sprd_lsc_vdev, queue)

#define to_sprd_lsc_buf(b) \
	container_of(b, struct sprd_lsc_buffer, vbuf)

static int sprd_lsc_querycap(struct file *file, void *fh,
			     struct v4l2_capability *cap)
{
	strscpy(cap->driver, "sprd-camsys", sizeof(cap->driver));
	strscpy(cap->card, "Unisoc Camera Subsystem", sizeof(cap->card));

	return 0;
}

static int sprd_lsc_enum_fmt(struct file *file, void *fh,
			     struct v4l2_fmtdesc *f)
{
	if (f->index > 0)
		return -EINVAL;

	f->pixelformat = V4L2_META_FMT_SPRD_DCAM_LSC;

	return 0;
}

static int sprd_lsc_g_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	static const struct v4l2_meta_format mfmt = {
		.dataformat = V4L2_META_FMT_SPRD_DCAM_LSC,
		.buffersize = SPRD_CAMSYS_LSC_BUF_SIZE,
	};

	f->fmt.meta = mfmt;

	return 0;
}

static int sprd_lsc_queue_setup(struct vb2_queue *q,
				unsigned int *num_buffers,
				unsigned int *num_planes,
				unsigned int sizes[],
				struct device *alloc_devs[])
{
	if (*num_planes && *num_planes > 1)
		return -EINVAL;

	if (sizes[0] && sizes[0] < SPRD_CAMSYS_LSC_BUF_SIZE)
		return -EINVAL;

	*num_planes = 1;

	if (!sizes[0])
		sizes[0] = SPRD_CAMSYS_LSC_BUF_SIZE;

	return 0;
}

static int sprd_lsc_buf_init(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct sprd_lsc_buffer *buf = to_sprd_lsc_buf(vbuf);

	buf->addr = vb2_dma_contig_plane_dma_addr(vb, 0);

	return 0;
}

static void sprd_lsc_buf_queue(struct vb2_buffer *vb)
{
	struct sprd_lsc_vdev *lsc =
		container_of(vb->vb2_queue, struct sprd_lsc_vdev, queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct sprd_lsc_buffer *buf = to_sprd_lsc_buf(vbuf);
	unsigned long flags;

	spin_lock_irqsave(&lsc->buf_lock, flags);
	list_add_tail(&buf->link, &lsc->buf_queue);
	spin_unlock_irqrestore(&lsc->buf_lock, flags);
}

static void sprd_lsc_stop_streaming(struct vb2_queue *q)
{
	struct sprd_lsc_vdev *lsc =
		container_of(q, struct sprd_lsc_vdev, queue);
	struct sprd_lsc_buffer *buf;
	unsigned long flags;

	spin_lock_irqsave(&lsc->buf_lock, flags);

	while (!list_empty(&lsc->buf_queue)) {
		buf = list_first_entry(&lsc->buf_queue, struct sprd_lsc_buffer, link);
		list_del(&buf->link);
		vb2_buffer_done(&buf->vbuf.vb2_buf, VB2_BUF_STATE_ERROR);
	}

	if (lsc->current_buf) {
		vb2_buffer_done(&lsc->current_buf->vbuf.vb2_buf, VB2_BUF_STATE_ERROR);
		lsc->current_buf = NULL;
	}

	spin_unlock_irqrestore(&lsc->buf_lock, flags);
}

static const struct v4l2_ioctl_ops sprd_lsc_ioctl_ops = {
	.vidioc_querycap		= sprd_lsc_querycap,
	.vidioc_enum_fmt_meta_out	= sprd_lsc_enum_fmt,
	.vidioc_g_fmt_meta_out		= sprd_lsc_g_fmt,
	.vidioc_s_fmt_meta_out		= sprd_lsc_g_fmt,
	.vidioc_try_fmt_meta_out	= sprd_lsc_g_fmt,
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

static const struct v4l2_file_operations sprd_lsc_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= video_ioctl2,
	.open		= v4l2_fh_open,
	.release	= vb2_fop_release,
	.poll		= vb2_fop_poll,
	.mmap		= vb2_fop_mmap,
};

static const struct vb2_ops sprd_lsc_vb2_ops = {
	.queue_setup	= sprd_lsc_queue_setup,
	.buf_init	= sprd_lsc_buf_init,
	.buf_queue	= sprd_lsc_buf_queue,
	.stop_streaming	= sprd_lsc_stop_streaming,
};

int sprd_lsc_vdev_register(struct sprd_lsc_vdev *lsc)
{
	struct sprd_camsys *cs = lsc->camsys;
	struct video_device *vdev = &lsc->vdev;
	struct vb2_queue *q = &lsc->queue;
	int ret;

	mutex_init(&lsc->lock);
	spin_lock_init(&lsc->buf_lock);
	INIT_LIST_HEAD(&lsc->buf_queue);

	/* vdev->name is set by the caller */
	vdev->device_caps = V4L2_CAP_META_OUTPUT | V4L2_CAP_STREAMING;
	vdev->v4l2_dev = &cs->v4l2_dev;
	vdev->fops = &sprd_lsc_fops;
	vdev->release = video_device_release_empty;
	vdev->ioctl_ops = &sprd_lsc_ioctl_ops;
	vdev->lock = &lsc->lock;
	vdev->vfl_dir = VFL_DIR_TX;

	video_set_drvdata(vdev, lsc);

	q->type = V4L2_BUF_TYPE_META_OUTPUT;
	q->io_modes = VB2_DMABUF | VB2_MMAP;
	q->ops = &sprd_lsc_vb2_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->buf_struct_size = sizeof(struct sprd_lsc_buffer);
	q->min_queued_buffers = 1;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->lock = &lsc->lock;
	q->dev = &cs->pdev->dev;
	ret = vb2_queue_init(q);
	if (ret) {
		dev_err(&cs->pdev->dev, "failed to init vb2 queue: %d\n", ret);
		goto err_destroy_mutex;
	}

	vdev->queue = q;

	lsc->pads[SPRD_LSC_PAD_SRC].flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&vdev->entity, SPRD_LSC_PAD_NUM,
				     lsc->pads);
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
	mutex_destroy(&lsc->lock);
	return ret;
}

void sprd_lsc_vdev_unregister(struct sprd_lsc_vdev *lsc)
{
	video_unregister_device(&lsc->vdev);
	media_entity_cleanup(&lsc->vdev.entity);
	vb2_queue_release(&lsc->queue);
	mutex_destroy(&lsc->lock);
}

struct sprd_lsc_buffer *
sprd_lsc_next_frame(struct sprd_lsc_vdev *lsc)
{
	struct sprd_lsc_buffer *buf;
	unsigned long flags;

	spin_lock_irqsave(&lsc->buf_lock, flags);

	if (lsc->current_buf) {
		dev_warn_ratelimited(&lsc->camsys->pdev->dev,
			"lsc buffer was not done before next frame!\n");
		buf = NULL;
		goto out;
	}

	/* Stage a new buffer. */
	buf = list_first_entry_or_null(&lsc->buf_queue, struct sprd_lsc_buffer, link);
	if (buf)
		list_del(&buf->link);

	lsc->current_buf = buf;

out:
	spin_unlock_irqrestore(&lsc->buf_lock, flags);

	return buf;
}

void sprd_lsc_done(struct sprd_lsc_vdev *lsc, unsigned int sequence)
{
	struct sprd_lsc_buffer *buf;

	spin_lock(&lsc->buf_lock);
	buf = lsc->current_buf;
	if (buf)
		lsc->current_buf = NULL;
	spin_unlock(&lsc->buf_lock);

	if (buf) {
		buf->vbuf.vb2_buf.timestamp = ktime_get_ns();
		buf->vbuf.sequence = sequence;
		vb2_buffer_done(&buf->vbuf.vb2_buf, VB2_BUF_STATE_DONE);
	}
}
