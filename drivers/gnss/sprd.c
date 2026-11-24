// SPDX-License-Identifier: GPL-2.0
/*
 * Unisoc GNSS driver
 *
 * Copyright (C) 2025 Otto Pflüger
 */

#include <linux/gnss.h>
#include <linux/mutex.h>
#include <linux/pm.h>
#include <linux/remoteproc.h>
#include <linux/rpmsg.h>
#include <linux/spinlock.h>
#include <linux/unaligned.h>

#define SPRD_MSG_MAGIC_CHAR		'~'
#define SPRD_MSG_MAGIC_COUNT		4

#define SPRD_GNSS_CMD_WAKEUP_SLEEP	0x0401
#define SPRD_GNSS_RSP_WAKEUP_SLEEP	0x0401

#define SPRD_GNSS_CMD_FW_IDLE_ON	0x0703
#define SPRD_GNSS_RSP_FW_IDLE_ON	0x0803

#define SPRD_GNSS_CMD_FW_IDLE_OFF	0x0704
#define SPRD_GNSS_RSP_FW_IDLE_OFF	0x0604

#define SPRD_GNSS_EVT_DIAG		0x0503

#define SPRD_GNSS_EVT_NMEA		0x0c01

struct sprd_msg_hdr {
	u8 magic[SPRD_MSG_MAGIC_COUNT];
	__be16 type;
	__be16 length;
	__be16 checksum;
} __packed;

enum sprd_msg_rx_state {
	SPRD_MSG_RX_SYNC,
	SPRD_MSG_RX_HEADER,
	SPRD_MSG_RX_DATA,
	SPRD_MSG_RX_CHECKSUM,
};

struct sprd_gnss_priv {
	struct device *dev;
	struct rpmsg_endpoint *ept;
	struct rproc *rproc;
	struct gnss_device *gdev;
	struct mutex cmd_lock; /* serializes commands and their responses */
	struct mutex power_lock; /* serializes open/close and system PM */
	spinlock_t rx_lock; /* parser state and gdev lifetime during callbacks */
	bool ready;
	bool opened;

	u16 expected_rsp;
	struct wait_queue_head rsp_wq;

	enum sprd_msg_rx_state rx_state;
	unsigned int rx_pos;
	struct sprd_msg_hdr rx_hdr;

	bool diag_skip;
};

static u16 sprd_msg_checksum(const void *data, size_t len)
{
	const u8 *buf = data;
	u32 sum = 0;

	while (len >= 2) {
		sum += get_unaligned_be16(buf);
		buf += 2;
		len -= 2;
	}

	if (len == 1)
		sum += (u16)*buf << 8;

	sum = (sum & 0xffff) + (sum >> 16);
	sum = (sum & 0xffff) + (sum >> 16);

	return ~sum;
}

static void sprd_gnss_handle_msg(struct sprd_gnss_priv *priv)
{
	u16 expected_rsp = READ_ONCE(priv->expected_rsp);
	u16 msg_type = be16_to_cpu(priv->rx_hdr.type);

	if (msg_type != SPRD_GNSS_EVT_DIAG)
		dev_dbg(priv->dev, "event: 0x%04x, data size 0x%04x\n",
			msg_type, be16_to_cpu(priv->rx_hdr.length));

	if (expected_rsp != 0 && msg_type == expected_rsp) {
		WRITE_ONCE(priv->expected_rsp, 0);
		wake_up_all(&priv->rsp_wq);
		return;
	}
}

static void sprd_gnss_resync(struct sprd_gnss_priv *priv)
{
	u8 *hdr = (u8 *)&priv->rx_hdr;
	unsigned int pos;

	/* Preserve an overlapping magic sequence after a corrupt header. */
	for (pos = 1; pos <= sizeof(priv->rx_hdr) - SPRD_MSG_MAGIC_COUNT; pos++) {
		if (memcmp(hdr + pos, priv->rx_hdr.magic, SPRD_MSG_MAGIC_COUNT))
			continue;
		priv->rx_pos = sizeof(priv->rx_hdr) - pos;
		memmove(hdr, hdr + pos, priv->rx_pos);
		priv->rx_state = SPRD_MSG_RX_HEADER;
		return;
	}
	priv->rx_pos = 0;
	for (pos = sizeof(priv->rx_hdr); pos && hdr[pos - 1] == SPRD_MSG_MAGIC_CHAR; pos--)
		priv->rx_pos++;
	priv->rx_state = SPRD_MSG_RX_SYNC;
}

static int sprd_gnss_receive(struct sprd_gnss_priv *priv, void *data, int len)
{
	struct gnss_device *gdev = priv->gdev;
	unsigned int xlen;
	u8 *buf = data;

	while (len > 0) {
		switch (priv->rx_state) {
		case SPRD_MSG_RX_SYNC:
			while (len && priv->rx_pos < SPRD_MSG_MAGIC_COUNT) {
				if (*buf == SPRD_MSG_MAGIC_CHAR)
					priv->rx_pos++;
				else
					priv->rx_pos = 0;
				buf++;
				len--;
			}

			if (len == 0)
				break;

			priv->rx_state = SPRD_MSG_RX_HEADER;
			memset(priv->rx_hdr.magic, SPRD_MSG_MAGIC_CHAR, SPRD_MSG_MAGIC_COUNT);
			fallthrough;

		case SPRD_MSG_RX_HEADER:
			xlen = sizeof(struct sprd_msg_hdr) - priv->rx_pos;
			memcpy((u8 *)&priv->rx_hdr + priv->rx_pos,
			       buf, min_t(unsigned int, xlen, len));

			if (xlen > len) {
				priv->rx_pos += len;
				return 0;
			}

			buf += xlen;
			len -= xlen;
			priv->rx_pos = 0;
			if (sprd_msg_checksum(&priv->rx_hdr, sizeof(priv->rx_hdr))) {
				sprd_gnss_resync(priv);
				break;
			}

			sprd_gnss_handle_msg(priv);

			priv->rx_state = SPRD_MSG_RX_DATA;
			break;

		case SPRD_MSG_RX_DATA:
			xlen = be16_to_cpu(priv->rx_hdr.length) - priv->rx_pos;

			if (priv->opened && be16_to_cpu(priv->rx_hdr.type) == SPRD_GNSS_EVT_NMEA)
				gnss_insert_raw(gdev, buf, min_t(unsigned int, xlen, len));

			if (xlen > len) {
				priv->rx_pos += len;
				return 0;
			}

			buf += xlen;
			len -= xlen;
			priv->rx_pos = 0;

			priv->rx_state = be16_to_cpu(priv->rx_hdr.length) ?
				SPRD_MSG_RX_CHECKSUM : SPRD_MSG_RX_SYNC;
			break;

		case SPRD_MSG_RX_CHECKSUM:
			/* Payload checksum is separate from rx_hdr.length. */
			xlen = min_t(unsigned int, 2 - priv->rx_pos, len);
			buf += xlen;
			len -= xlen;
			priv->rx_pos += xlen;
			if (priv->rx_pos == 2) {
				priv->rx_pos = 0;
				priv->rx_state = SPRD_MSG_RX_SYNC;
			}
			break;
		}
	}

	return 0;
}

static int sprd_gnss_rpmsg_callback(struct rpmsg_device *rpdev, void *data,
				    int len, void *userdata, u32 addr)
{
	struct sprd_gnss_priv *priv = dev_get_drvdata(&rpdev->dev);
	unsigned long flags;

	if (!priv)
		return 0;
	spin_lock_irqsave(&priv->rx_lock, flags);
	if (priv->ready)
		sprd_gnss_receive(priv, data, len);
	spin_unlock_irqrestore(&priv->rx_lock, flags);
	return 0;
}

static int sprd_gnss_cmd(struct sprd_gnss_priv *priv, u16 cmd,
			 void *data, size_t data_size, u16 expected_rsp)
{
	struct sprd_msg_hdr hdr;
	int ret;

	if (data_size > U16_MAX)
		return -EINVAL;
	mutex_lock(&priv->cmd_lock);

	dev_dbg(priv->dev, "send command: 0x%04x, data size 0x%04zx\n",
		cmd, data_size);

	memset(hdr.magic, SPRD_MSG_MAGIC_CHAR, SPRD_MSG_MAGIC_COUNT);
	hdr.type = cpu_to_be16(cmd);
	hdr.length = cpu_to_be16(data_size);
	hdr.checksum = cpu_to_be16(sprd_msg_checksum(&hdr, sizeof(hdr) - 2));

	WRITE_ONCE(priv->expected_rsp, expected_rsp);

	ret = rpmsg_send(priv->ept, &hdr, sizeof(hdr));
	if (ret) {
		dev_err(priv->dev, "failed to send command header: %d\n", ret);
		goto out;
	}

	if (data_size > 0) {
		__be16 checksum;

		ret = rpmsg_send(priv->ept, data, data_size);
		if (ret) {
			dev_err(priv->dev, "failed to send data: %d\n", ret);
			goto out;
		}
		
		checksum = cpu_to_be16(sprd_msg_checksum(data, data_size));

		ret = rpmsg_send(priv->ept, &checksum, sizeof(checksum));
		if (ret) {
			dev_err(priv->dev, "failed to send data: %d\n", ret);
			goto out;
		}
	}

	if (!wait_event_timeout(priv->rsp_wq,
				READ_ONCE(priv->expected_rsp) == 0,
				msecs_to_jiffies(10000)))
		ret = -ETIMEDOUT;
out:
	WRITE_ONCE(priv->expected_rsp, 0);
	mutex_unlock(&priv->cmd_lock);
	return ret;
}

static void sprd_gnss_close_locked(struct gnss_device *gdev)
{
	struct sprd_gnss_priv *priv = gnss_get_drvdata(gdev);
	unsigned long flags;
	u8 sleep_data[] = { 2, 0, 0 };

	spin_lock_irqsave(&priv->rx_lock, flags);
	priv->opened = false;
	spin_unlock_irqrestore(&priv->rx_lock, flags);

	sprd_gnss_cmd(priv, SPRD_GNSS_CMD_FW_IDLE_ON, NULL, 0,
		      SPRD_GNSS_RSP_FW_IDLE_ON);

	sprd_gnss_cmd(priv, SPRD_GNSS_CMD_WAKEUP_SLEEP,
		      &sleep_data, sizeof(sleep_data),
		      SPRD_GNSS_RSP_WAKEUP_SLEEP);

	rproc_shutdown(priv->rproc);
}

static void sprd_gnss_close(struct gnss_device *gdev)
{
	struct sprd_gnss_priv *priv = gnss_get_drvdata(gdev);

	mutex_lock(&priv->power_lock);
	sprd_gnss_close_locked(gdev);
	mutex_unlock(&priv->power_lock);
}

static int sprd_gnss_open(struct gnss_device *gdev)
{
	struct sprd_gnss_priv *priv = gnss_get_drvdata(gdev);
	unsigned long flags;
	u8 wakeup_data[] = { 1, 0, 0 };
	int ret;

	mutex_lock(&priv->power_lock);
	spin_lock_irqsave(&priv->rx_lock, flags);
	priv->rx_state = SPRD_MSG_RX_SYNC;
	priv->rx_pos = 0;
	spin_unlock_irqrestore(&priv->rx_lock, flags);
	ret = rproc_boot(priv->rproc);
	if (ret)
		goto out;

	/* This command may fail if the firmware is not sleeping */
	sprd_gnss_cmd(priv, SPRD_GNSS_CMD_WAKEUP_SLEEP,
		      &wakeup_data, sizeof(wakeup_data),
		      SPRD_GNSS_RSP_WAKEUP_SLEEP);

	ret = sprd_gnss_cmd(priv, SPRD_GNSS_CMD_FW_IDLE_OFF, NULL, 0,
			    SPRD_GNSS_RSP_FW_IDLE_OFF);
	if (ret)
		goto err;

	spin_lock_irqsave(&priv->rx_lock, flags);
	priv->opened = true;
	spin_unlock_irqrestore(&priv->rx_lock, flags);
	goto out;

err:
	sprd_gnss_close_locked(gdev);
out:
	mutex_unlock(&priv->power_lock);
	return ret;
}

static const struct gnss_operations sprd_gnss_ops = {
	.open		= sprd_gnss_open,
	.close		= sprd_gnss_close,
};

static int sprd_gnss_rpmsg_probe(struct rpmsg_device *rpdev)
{
	struct sprd_gnss_priv *priv;
	struct gnss_device *gdev;
	unsigned long flags;
	phandle rproc_handle;
	int ret;

	priv = devm_kzalloc(&rpdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = &rpdev->dev;
	priv->ept = rpdev->ept;
	init_waitqueue_head(&priv->rsp_wq);
	mutex_init(&priv->cmd_lock);
	mutex_init(&priv->power_lock);
	spin_lock_init(&priv->rx_lock);

	ret = of_property_read_u32(rpdev->dev.of_node, "sprd,remoteproc",
				   &rproc_handle);
	if (ret) {
		dev_err(&rpdev->dev, "cannot read remoteproc handle\n");
		return ret;
	}

	gdev = gnss_allocate_device(&rpdev->dev);
	if (!gdev)
		return -ENOMEM;

	gdev->ops = &sprd_gnss_ops;
	gdev->type = GNSS_TYPE_NMEA;
	gnss_set_drvdata(gdev, priv);
	priv->gdev = gdev;
	dev_set_drvdata(&rpdev->dev, priv);

	priv->rproc = rproc_get_by_phandle(rproc_handle);
	if (!priv->rproc) {
		ret = dev_err_probe(&rpdev->dev, -EPROBE_DEFER,
				    "cannot find remoteproc\n");
		goto put_gdev;
	}
	/* In particular, remove this endpoint before an external WCN disappears. */
	if (!device_link_add(&rpdev->dev, priv->rproc->dev.parent,
			     DL_FLAG_AUTOREMOVE_CONSUMER)) {
		ret = -ENOMEM;
		goto put_rproc;
	}

	spin_lock_irqsave(&priv->rx_lock, flags);
	priv->ready = true;
	spin_unlock_irqrestore(&priv->rx_lock, flags);
	ret = gnss_register_device(gdev);
	if (ret)
		goto put_rproc;

	return 0;
put_rproc:
	rproc_put(priv->rproc);
put_gdev:
	spin_lock_irqsave(&priv->rx_lock, flags);
	priv->ready = false;
	spin_unlock_irqrestore(&priv->rx_lock, flags);
	dev_set_drvdata(&rpdev->dev, NULL);
	gnss_put_device(gdev);
	return ret;
}

static void sprd_gnss_rpmsg_remove(struct rpmsg_device *rpdev)
{
	struct sprd_gnss_priv *priv = dev_get_drvdata(&rpdev->dev);
	struct gnss_device *gdev = priv->gdev;
	unsigned long flags;

	gnss_deregister_device(gdev);
	spin_lock_irqsave(&priv->rx_lock, flags);
	priv->ready = false;
	priv->gdev = NULL;
	spin_unlock_irqrestore(&priv->rx_lock, flags);
	gnss_put_device(gdev);

	rproc_put(priv->rproc);
}

static int sprd_gnss_wake(struct sprd_gnss_priv *priv)
{
	u8 data[] = { 1, 0, 0 };

	/* The wake command can fail when the receiver is already awake. */
	sprd_gnss_cmd(priv, SPRD_GNSS_CMD_WAKEUP_SLEEP, data, sizeof(data),
		      SPRD_GNSS_RSP_WAKEUP_SLEEP);
	return sprd_gnss_cmd(priv, SPRD_GNSS_CMD_FW_IDLE_OFF, NULL, 0,
			     SPRD_GNSS_RSP_FW_IDLE_OFF);
}

static int sprd_gnss_suspend(struct device *dev)
{
	struct sprd_gnss_priv *priv = dev_get_drvdata(dev);
	u8 data[] = { 2, 0, 0 };
	int ret = 0;

	mutex_lock(&priv->power_lock);
	if (priv->opened) {
		ret = sprd_gnss_cmd(priv, SPRD_GNSS_CMD_FW_IDLE_ON, NULL, 0,
				    SPRD_GNSS_RSP_FW_IDLE_ON);
		if (!ret)
			ret = sprd_gnss_cmd(priv, SPRD_GNSS_CMD_WAKEUP_SLEEP,
					    data, sizeof(data), SPRD_GNSS_RSP_WAKEUP_SLEEP);
		if (ret)
			sprd_gnss_wake(priv);
	}
	mutex_unlock(&priv->power_lock);
	return ret;
}

static int sprd_gnss_resume(struct device *dev)
{
	struct sprd_gnss_priv *priv = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&priv->power_lock);
	if (priv->opened)
		ret = sprd_gnss_wake(priv);
	mutex_unlock(&priv->power_lock);
	return ret;
}

static DEFINE_SIMPLE_DEV_PM_OPS(sprd_gnss_pm_ops, sprd_gnss_suspend, sprd_gnss_resume);

static const struct of_device_id sprd_gnss_of_match[] = {
	{ .compatible = "sprd,gnss-sipc" },
	{ }
};
MODULE_DEVICE_TABLE(of, sprd_gnss_of_match);

static struct rpmsg_driver sprd_gnss_rpmsg_driver = {
	.probe = sprd_gnss_rpmsg_probe,
	.remove = sprd_gnss_rpmsg_remove,
	.callback = sprd_gnss_rpmsg_callback,
	.drv = {
		.name = "sprd-gnss",
		.of_match_table = sprd_gnss_of_match,
		.pm = pm_sleep_ptr(&sprd_gnss_pm_ops),
	},
};

module_rpmsg_driver(sprd_gnss_rpmsg_driver);

MODULE_DESCRIPTION("Unisoc GNSS receiver driver");
MODULE_LICENSE("GPL");
