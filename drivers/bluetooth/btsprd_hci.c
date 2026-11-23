// SPDX-License-Identifier: GPL-2.0-only
/*
 * Unisoc Marlin3-Lite (SC2355) Bluetooth HCI driver on the SDIO WCN core.
 *
 * Registers a native hci0 over wcn_chan and performs the Unisoc
 * pskey/RF/enable vendor-command init sequence in-kernel.
 */

#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/firmware.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/nvmem-consumer.h>
#include <linux/property.h>
#include <linux/random.h>
#include <linux/slab.h>

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

#include "hci_uart.h"
#include "btsprd_ini.h"
#include <linux/misc/unisoc-wcn.h>

#define BT_PSKEY_MAX		160
#define BT_RF_MAX		252

struct btsprdsdio {
	struct hci_dev *hdev;
	struct hci_uart hu;
	struct sk_buff *rx_skb;
	struct mutex rx_lock;
	bool opened;
	struct wcn_core *core;
	struct wcn_chan *tx_chan;
	struct wcn_chan *rx_chan;
	size_t rf_cfg_len;
	u8 rf_cfg[BT_RF_MAX];
	size_t pskey_cfg_len;
	u8 pskey_cfg[BT_PSKEY_MAX];
};

static const struct h4_recv_pkt sprd_recv_pkts[] = {
	{ H4_RECV_ACL,      .recv = hci_recv_frame },
	{ H4_RECV_SCO,      .recv = hci_recv_frame },
	{ H4_RECV_EVENT,    .recv = hci_recv_frame },
};

static void btsprdsdio_rx(struct wcn_chan *chan, struct sk_buff *skb)
{
	struct btsprdsdio *bts = wcn_chan_get_drvdata(chan);

	mutex_lock(&bts->rx_lock);
	if (!bts->opened)
		goto out;

	bts->rx_skb = h4_recv_buf(&bts->hu, bts->rx_skb, skb->data, skb->len,
				  sprd_recv_pkts,
				  ARRAY_SIZE(sprd_recv_pkts));
	if (IS_ERR(bts->rx_skb)) {
		bt_dev_err(bts->hdev, "frame reassembly failed (%ld)",
			   PTR_ERR(bts->rx_skb));
		bts->rx_skb = NULL;
	} else {
		bts->hdev->stat.byte_rx += skb->len;
	}

out:
	mutex_unlock(&bts->rx_lock);
	kfree_skb(skb);
}

static const struct wcn_chan_ops btsprdsdio_rx_ops = {
	.rx = btsprdsdio_rx,
};

static int btsprdsdio_send(struct hci_dev *hdev, struct sk_buff *skb)
{
	struct btsprdsdio *bts = hci_get_drvdata(hdev);
	int ret;

	memcpy(skb_push(skb, 1), &hci_skb_pkt_type(skb), 1);

	ret = wcn_chan_tx(bts->tx_chan, skb->data, skb->len);
	if (ret) {
		hdev->stat.err_tx++;
		/* hci_send_frame() frees the skb when send returns an error. */
		return ret;
	}

	hdev->stat.byte_tx += skb->len;
	switch (hci_skb_pkt_type(skb)) {
	case HCI_COMMAND_PKT:
		hdev->stat.cmd_tx++;
		break;
	case HCI_ACLDATA_PKT:
		hdev->stat.acl_tx++;
		break;
	case HCI_SCODATA_PKT:
		hdev->stat.sco_tx++;
		break;
	}

	kfree_skb(skb);
	return 0;
}

static int btsprdsdio_open(struct hci_dev *hdev)
{
	struct btsprdsdio *bts = hci_get_drvdata(hdev);
	int ret;

	ret = wcn_power_get(bts->core, WCN_CHILD_BT);
	if (ret)
		bt_dev_err(hdev, "WCN power on failed (%d)", ret);
	else {
		mutex_lock(&bts->rx_lock);
		bts->opened = true;
		mutex_unlock(&bts->rx_lock);
	}

	return ret;
}

static int btsprdsdio_close(struct hci_dev *hdev)
{
	struct btsprdsdio *bts = hci_get_drvdata(hdev);

	mutex_lock(&bts->rx_lock);
	bts->opened = false;
	kfree_skb(bts->rx_skb);
	bts->rx_skb = NULL;
	mutex_unlock(&bts->rx_lock);

	wcn_power_put(bts->core, WCN_CHILD_BT);

	return 0;
}

static int btsprdsdio_set_bdaddr(struct hci_dev *hdev, const bdaddr_t *bdaddr)
{
	const u8 enable_cmd[] = { 0, 9, 1 };
	struct btsprdsdio *bts = hci_get_drvdata(hdev);
	struct sk_buff *skb;

	memcpy(bts->pskey_cfg + 20, bdaddr, 6);
	skb = __hci_cmd_sync(hdev, 0xfca0, bts->pskey_cfg_len, bts->pskey_cfg,
			     HCI_INIT_TIMEOUT);
	if (IS_ERR(skb))
		goto err;
	kfree_skb(skb);

	skb = __hci_cmd_sync(hdev, 0xfca2, bts->rf_cfg_len, bts->rf_cfg,
			     HCI_INIT_TIMEOUT);
	if (IS_ERR(skb))
		goto err;
	kfree_skb(skb);

	skb = __hci_cmd_sync(hdev, 0xfca1, sizeof(enable_cmd), enable_cmd,
			     HCI_INIT_TIMEOUT);
	if (IS_ERR(skb))
		goto err;
	kfree_skb(skb);

	skb = __hci_cmd_sync(hdev, HCI_OP_RESET, 0, NULL, HCI_INIT_TIMEOUT);
	if (IS_ERR(skb))
		goto err;
	kfree_skb(skb);

	return 0;

err:
	bt_dev_err(hdev, "initialization sequence failed (%ld)", PTR_ERR(skb));
	return PTR_ERR(skb);
}

static int btsprdsdio_load_cfg(struct device *dev, const char *name,
			       u8 *out, size_t out_max, size_t *out_len)
{
	const struct firmware *fw;
	int ret;

	ret = request_firmware(&fw, name, dev);
	if (ret < 0) {
		dev_err(dev, "firmware request for %s failed (%d)\n", name, ret);
		return ret;
	}

	ret = btsprd_ini_pack((const char *)fw->data, fw->size, out, out_max);
	release_firmware(fw);
	if (ret < 0) {
		dev_err(dev, "failed to parse %s (%d)\n", name, ret);
		return ret;
	}

	*out_len = ret;
	return 0;
}

static int btsprdsdio_read_uid(struct device *dev, u8 uid[8])
{
	static const char * const names[2] = { "uid-end", "uid-start" };
	struct nvmem_cell *cell;
	size_t len;
	void *buf;
	int i;

	for (i = 0; i < 2; i++) {
		cell = nvmem_cell_get(dev, names[i]);
		if (IS_ERR(cell))
			return PTR_ERR(cell);

		buf = nvmem_cell_read(cell, &len);
		nvmem_cell_put(cell);
		if (IS_ERR(buf))
			return PTR_ERR(buf);
		if (len < 4) {
			kfree(buf);
			return -EINVAL;
		}

		memcpy(uid + i * 4, buf, 4);
		kfree(buf);
	}

	return 0;
}

static int btsprdsdio_setup_bdaddr(struct hci_dev *hdev, struct device *dev)
{
	bdaddr_t ba;
	u8 uid[8];
	int ret;

	if (!device_property_read_u8_array(dev, "local-bd-address",
					   (u8 *)&ba, sizeof(ba)) &&
	    bacmp(&ba, BDADDR_ANY)) {
		bacpy(&hdev->public_addr, &ba);
		return 0;
	}

	ret = btsprdsdio_read_uid(dev, uid);
	if (ret == -EPROBE_DEFER)
		return ret;
	if (ret) {
		bt_dev_warn(hdev, "chip UID unavailable (%d), using random BD address",
			    ret);
		get_random_bytes(&ba, sizeof(ba));
	} else {
		ba.b[0] = uid[0] ^ uid[6];
		ba.b[1] = uid[1] ^ uid[7];
		ba.b[2] = uid[2];
		ba.b[3] = uid[3];
		ba.b[4] = uid[4];
		ba.b[5] = uid[5];
	}

	ba.b[5] = (ba.b[5] & 0xfe) | 0x02;
	bacpy(&hdev->public_addr, &ba);

	return 0;
}

static int btsprdsdio_probe(struct platform_device *pdev)
{
	struct wcn_child_pdata *pdata = dev_get_platdata(&pdev->dev);
	struct btsprdsdio *bts;
	struct hci_dev *hdev;
	struct wcn_core *core;
	int ret;

	if (!pdata || !pdata->core || !pdata->data ||
	    !pdata->data->pskey_name || !pdata->data->rf_name)
		return -ENODEV;

	core = pdata->core;

	bts = devm_kzalloc(&pdev->dev, sizeof(*bts), GFP_KERNEL);
	if (!bts)
		return -ENOMEM;

	bts->core = core;
	mutex_init(&bts->rx_lock);

	ret = btsprdsdio_load_cfg(&pdev->dev, pdata->data->pskey_name,
				  bts->pskey_cfg, BT_PSKEY_MAX,
				  &bts->pskey_cfg_len);
	if (ret)
		return ret;

	ret = btsprdsdio_load_cfg(&pdev->dev, pdata->data->rf_name, bts->rf_cfg,
				  BT_RF_MAX, &bts->rf_cfg_len);
	if (ret)
		return ret;

	if (bts->pskey_cfg_len > 5)
		bts->pskey_cfg[5] &= ~0x01;

	hdev = hci_alloc_dev();
	if (!hdev)
		return -ENOMEM;

	hci_set_drvdata(hdev, bts);
	bts->hdev = hdev;
	bts->hu.hdev = hdev;
	SET_HCIDEV_DEV(hdev, &pdev->dev);

	hdev->manufacturer = 1855;
	hdev->bus = HCI_SDIO;
	hdev->open = btsprdsdio_open;
	hdev->close = btsprdsdio_close;
	hdev->send = btsprdsdio_send;
	hdev->set_bdaddr = btsprdsdio_set_bdaddr;

	hci_set_quirk(hdev, HCI_QUIRK_INVALID_BDADDR);

	ret = btsprdsdio_setup_bdaddr(hdev, &pdev->dev);
	if (ret)
		goto free_dev;

	bts->rx_chan = wcn_chan_open(core, pdata->data->channels.rx_data,
				     &btsprdsdio_rx_ops, bts);
	if (IS_ERR(bts->rx_chan)) {
		ret = PTR_ERR(bts->rx_chan);
		goto free_dev;
	}
	bts->tx_chan = wcn_chan_open(core, pdata->data->channels.tx_data, NULL,
				     bts);
	if (IS_ERR(bts->tx_chan)) {
		ret = PTR_ERR(bts->tx_chan);
		goto close_rx;
	}

	platform_set_drvdata(pdev, bts);

	ret = hci_register_dev(hdev);
	if (ret < 0)
		goto close_tx;

	dev_info(&pdev->dev,
		 "registered %s (pskey %zu B, rf %zu B, tx chn %d, rx chn %d)\n",
		 hdev->name, bts->pskey_cfg_len, bts->rf_cfg_len,
		 pdata->data->channels.tx_data, pdata->data->channels.rx_data);
	return 0;

close_tx:
	wcn_chan_close(bts->tx_chan);
close_rx:
	wcn_chan_close(bts->rx_chan);
free_dev:
	hci_free_dev(hdev);
	return ret;
}

static void btsprdsdio_remove(struct platform_device *pdev)
{
	struct btsprdsdio *bts = platform_get_drvdata(pdev);

	hci_unregister_dev(bts->hdev);
	wcn_chan_close(bts->tx_chan);
	wcn_chan_close(bts->rx_chan);
	hci_free_dev(bts->hdev);
}

static const struct of_device_id btsprdsdio_of_match[] = {
	{ .compatible = "sprd,bluetooth-hci" },
	{ }
};
MODULE_DEVICE_TABLE(of, btsprdsdio_of_match);

static struct platform_driver btsprdsdio_driver = {
	.probe = btsprdsdio_probe,
	.remove = btsprdsdio_remove,
	.driver = {
		.name = "btsprdsdio",
		.of_match_table = btsprdsdio_of_match,
	},
};
module_platform_driver(btsprdsdio_driver);

MODULE_DESCRIPTION("Unisoc Marlin3-Lite SDIO HCI driver");
MODULE_LICENSE("GPL v2");
