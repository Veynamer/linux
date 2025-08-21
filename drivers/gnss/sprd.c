// SPDX-License-Identifier: GPL-2.0
/*
 * Unisoc GNSS driver
 *
 * Copyright (C) 2025 Otto Pflüger
 */

#include <linux/gnss.h>
#include <linux/remoteproc.h>
#include <linux/rpmsg.h>

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
};

struct sprd_gnss_priv {
	struct device *dev;
	struct rpmsg_endpoint *ept;
	struct rproc *rproc;

	u16 expected_rsp;
	struct wait_queue_head rsp_wq;

	enum sprd_msg_rx_state rx_state;
	unsigned int rx_pos;
	struct sprd_msg_hdr rx_hdr;

	bool diag_skip;
};

static u16 sprd_msg_checksum(const void *data, size_t len)
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

static int sprd_gnss_rpmsg_callback(struct rpmsg_device *rpdev, void *data,
				    int len, void *userdata, u32 addr)
{
	struct gnss_device *gdev = dev_get_drvdata(&rpdev->dev);
	struct sprd_gnss_priv *priv = gnss_get_drvdata(gdev);
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
			fallthrough;

		case SPRD_MSG_RX_HEADER:
			xlen = sizeof(struct sprd_msg_hdr) - priv->rx_pos;
			memcpy((u8 *)&priv->rx_hdr + priv->rx_pos,
			       buf, min(xlen, len));

			if (xlen > len) {
				priv->rx_pos += len;
				return 0;
			}

			buf += xlen;
			len -= xlen;
			priv->rx_pos = 0;

			sprd_gnss_handle_msg(priv);

			priv->rx_state = SPRD_MSG_RX_DATA;
			break;

		case SPRD_MSG_RX_DATA:
			xlen = be16_to_cpu(priv->rx_hdr.length) - priv->rx_pos;

			if (be16_to_cpu(priv->rx_hdr.type) == SPRD_GNSS_EVT_NMEA)
				gnss_insert_raw(gdev, buf, min(xlen, len));

			if (xlen > len) {
				priv->rx_pos += len;
				return 0;
			}

			buf += xlen;
			len -= xlen;
			priv->rx_pos = 0;

			priv->rx_state = SPRD_MSG_RX_SYNC;
			break;
		}
	}

	return 0;
}

static int sprd_gnss_cmd(struct sprd_gnss_priv *priv, u16 cmd,
			 void *data, size_t data_size, u16 expected_rsp)
{
	struct sprd_msg_hdr hdr;
	int ret;

	if (data_size > U16_MAX)
		return -EINVAL;

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
		return ret;
	}

	if (data_size > 0) {
		__be16 checksum;

		ret = rpmsg_send(priv->ept, data, data_size);
		if (ret) {
			dev_err(priv->dev, "failed to send data: %d\n", ret);
			return ret;
		}
		
		checksum = cpu_to_be16(sprd_msg_checksum(data, data_size));

		ret = rpmsg_send(priv->ept, &checksum, sizeof(checksum));
		if (ret) {
			dev_err(priv->dev, "failed to send data: %d\n", ret);
			return ret;
		}
	}

	if (!wait_event_timeout(priv->rsp_wq,
				READ_ONCE(priv->expected_rsp) == 0,
				msecs_to_jiffies(10000)))
		return -ETIMEDOUT;

	return 0;
}

static void sprd_gnss_close(struct gnss_device *gdev)
{
	struct sprd_gnss_priv *priv = gnss_get_drvdata(gdev);
	u8 sleep_data[] = { 2, 0, 0 };

	sprd_gnss_cmd(priv, SPRD_GNSS_CMD_FW_IDLE_ON, NULL, 0,
		      SPRD_GNSS_RSP_FW_IDLE_ON);

	sprd_gnss_cmd(priv, SPRD_GNSS_CMD_WAKEUP_SLEEP,
		      &sleep_data, sizeof(sleep_data),
		      SPRD_GNSS_RSP_WAKEUP_SLEEP);

	rproc_shutdown(priv->rproc);
}

static int sprd_gnss_open(struct gnss_device *gdev)
{
	struct sprd_gnss_priv *priv = gnss_get_drvdata(gdev);
	u8 wakeup_data[] = { 1, 0, 0 };
	int ret;

	ret = rproc_boot(priv->rproc);
	if (ret)
		return ret;

	/* This command may fail if the firmware is not sleeping */
	sprd_gnss_cmd(priv, SPRD_GNSS_CMD_WAKEUP_SLEEP,
		      &wakeup_data, sizeof(wakeup_data),
		      SPRD_GNSS_RSP_WAKEUP_SLEEP);

	ret = sprd_gnss_cmd(priv, SPRD_GNSS_CMD_FW_IDLE_OFF, NULL, 0,
			    SPRD_GNSS_RSP_FW_IDLE_OFF);
	if (ret)
		goto err;

	return 0;

err:
	sprd_gnss_close(gdev);

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
	phandle rproc_handle;
	int ret;

	priv = devm_kzalloc(&rpdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = &rpdev->dev;
	priv->ept = rpdev->ept;
	init_waitqueue_head(&priv->rsp_wq);

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
	dev_set_drvdata(&rpdev->dev, gdev);

	priv->rproc = rproc_get_by_phandle(rproc_handle);
	if (!priv->rproc) {
		return dev_err_probe(&rpdev->dev, -EPROBE_DEFER,
				     "cannot find remoteproc\n");
	}

	ret = gnss_register_device(gdev);
	if (ret) {
		rproc_put(priv->rproc);
		return ret;
	}

	return 0;
}

static void sprd_gnss_rpmsg_remove(struct rpmsg_device *rpdev)
{
	struct gnss_device *gdev = dev_get_drvdata(&rpdev->dev);
	struct sprd_gnss_priv *priv = gnss_get_drvdata(gdev);

	gnss_deregister_device(gdev);
	gnss_put_device(gdev);

	rproc_put(priv->rproc);
}

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
	},
};

module_rpmsg_driver(sprd_gnss_rpmsg_driver);

MODULE_DESCRIPTION("Unisoc GNSS receiver driver");
MODULE_LICENSE("GPL");
