// SPDX-License-Identifier: GPL-2.0-only
#include <linux/of.h>
#include <linux/freezer.h>
#include <linux/platform_device.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/unaligned.h>
#include <linux/wait.h>

#include "sc23xx.h"
#include <linux/misc/unisoc-wcn.h>

#define SC23XX_SDIO_DATA_TXQ_HIGH	384
#define SC23XX_SDIO_DATA_TXQ_LOW		128
#define SC23XX_SDIO_DATA_TXQ_MAX		512

struct sc23xx_sdio {
	struct sc23xx_dev sdev;
	struct wcn_core *core;
	const struct wcn_child_data *data;
	struct wcn_chan *tx_cmd;
	struct wcn_chan *tx_data;
	struct wcn_chan *rx_cmd;
	struct wcn_chan *rx_log;
	struct wcn_chan *rx_data;
	struct sk_buff_head data_txq;
	wait_queue_head_t data_tx_wait;
	struct task_struct *data_tx_thread;
};

static struct wcn_chan *sc23xx_sdio_open_chan(struct sc23xx_sdio *priv,
					      unsigned int id,
					     const struct wcn_chan_ops *ops)
{
	return wcn_chan_open(priv->core, id, ops, priv);
}

static void sc23xx_sdio_rx_cmd(struct wcn_chan *chan, struct sk_buff *skb)
{
	struct sc23xx_sdio *priv = wcn_chan_get_drvdata(chan);

	sc23xx_rx_msg(&priv->sdev, SC23XX_MSG_TYPE_CMD, skb);
}

static void sc23xx_sdio_rx_data(struct wcn_chan *chan, struct sk_buff *skb)
{
	struct sc23xx_sdio *priv = wcn_chan_get_drvdata(chan);

	sc23xx_rx_msg(&priv->sdev, SC23XX_MSG_TYPE_DATA, skb);
}

static void sc23xx_sdio_rx_log(struct wcn_chan *chan, struct sk_buff *skb)
{
	kfree_skb(skb);
}

static const struct wcn_chan_ops sc23xx_sdio_rx_cmd_ops = {
	.rx = sc23xx_sdio_rx_cmd,
};

static const struct wcn_chan_ops sc23xx_sdio_rx_data_ops = {
	.rx = sc23xx_sdio_rx_data,
};

static const struct wcn_chan_ops sc23xx_sdio_rx_log_ops = {
	.rx = sc23xx_sdio_rx_log,
};

static int sc23xx_sdio_tx_cmd(struct sc23xx_dev *sdev, struct sk_buff *skb)
{
	struct sc23xx_sdio *priv = container_of(sdev, struct sc23xx_sdio, sdev);

	return wcn_chan_tx(priv->tx_cmd, skb->data, skb->len);
}

static bool sc23xx_sdio_prepare_data_frame(struct sc23xx_sdio *priv,
					   struct sk_buff *skb)
{
	struct sc23xx_dev *sdev = &priv->sdev;
	struct sc23xx_tx_data_hdr *hdr = (void *)skb->data;

	if (sdev->credit_capa == SC23XX_TX_WITH_CREDIT) {
		int color = sc23xx_tx_reserve_credit(sdev);

		if (color < 0)
			return false;

		put_unaligned_le16(sc23xx_tx_seq_info(sdev, color),
				   &hdr->seq_info);
	}

	wiphy_dbg(sdev->wiphy,
		  "tx data: type=%u lut=%u seq_info=0x%04x len=%u cred[%d %d %d %d]\n",
		  hdr->common.type, hdr->sta_lut_index,
		  get_unaligned_le16(&hdr->seq_info), skb->len,
		  atomic_read(&sdev->tx_credit[0]),
		  atomic_read(&sdev->tx_credit[1]),
		  atomic_read(&sdev->tx_credit[2]),
		  atomic_read(&sdev->tx_credit[3]));

	return true;
}

static bool sc23xx_sdio_send_data_batch(struct sc23xx_sdio *priv)
{
	struct wcn_tx_packet packets[WCN_TX_BATCH_MAX];
	struct sk_buff *skbs[WCN_TX_BATCH_MAX];
	unsigned int num = 0, i;
	struct sk_buff *skb;
	int ret;

	/* Drain only queued frames; do not delay a lone frame to fill a batch. */
	while (num < ARRAY_SIZE(packets) && (skb = skb_dequeue(&priv->data_txq))) {
		if (!sc23xx_sdio_prepare_data_frame(priv, skb)) {
			skb_queue_head(&priv->data_txq, skb);
			break;
		}
		packets[num].data = skb->data;
		packets[num].len = skb->len;
		skbs[num++] = skb;
	}
	if (!num)
		return false;

	ret = wcn_chan_tx_batch(priv->tx_data, packets, num);
	if (ret)
		wiphy_warn_ratelimited(priv->sdev.wiphy,
				       "data TX batch (%u frames) failed: %d\n", num, ret);

	for (i = 0; i < num; i++)
		dev_kfree_skb_any(skbs[i]);
	return true;
}

static int sc23xx_sdio_data_tx_thread(void *data)
{
	struct sc23xx_sdio *priv = data;
	struct sc23xx_dev *sdev = &priv->sdev;
	struct sk_buff *skb;

	set_freezable();
	while (!kthread_freezable_should_stop(NULL)) {
		if (skb_queue_empty(&priv->data_txq)) {
			mutex_lock(&sdev->pwr_state_lock);
			sdev->pwr_state = SC23XX_PWR_HOST_IDLE;
			mutex_unlock(&sdev->pwr_state_lock);

			wait_event_freezable(priv->data_tx_wait,
					     !skb_queue_empty(&priv->data_txq) ||
					     kthread_should_stop());
			if (kthread_should_stop())
				break;
		}

		mutex_lock(&sdev->pwr_state_lock);
		if (sdev->pwr_state == SC23XX_PWR_SUSPENDED)
			sc23xx_wakeup_fw(sdev);
		sdev->pwr_state = SC23XX_PWR_ACTIVE;
		mutex_unlock(&sdev->pwr_state_lock);

		wait_event_freezable(priv->data_tx_wait,
				     sc23xx_tx_credit_ready(sdev) ||
				     kthread_should_stop());
		if (kthread_should_stop())
			break;

		while (!skb_queue_empty(&priv->data_txq)) {
			if (kthread_freezable_should_stop(NULL))
				break;
			if (!sc23xx_sdio_send_data_batch(priv))
				break;

			if (skb_queue_len(&priv->data_txq) <=
			    SC23XX_SDIO_DATA_TXQ_LOW)
				sc23xx_netif_tx(sdev, true);
		}

		if (skb_queue_empty(&priv->data_txq))
			sc23xx_netif_tx(sdev, true);
	}

	while ((skb = skb_dequeue(&priv->data_txq)))
		dev_kfree_skb_any(skb);

	return 0;
}

static void sc23xx_sdio_tx_data(struct sc23xx_dev *sdev, struct sk_buff *skb)
{
	struct sc23xx_sdio *priv = container_of(sdev, struct sc23xx_sdio, sdev);
	unsigned int qlen;

	if (skb_queue_len(&priv->data_txq) >= SC23XX_SDIO_DATA_TXQ_MAX) {
		wiphy_warn_ratelimited(sdev->wiphy,
				       "data TX backlog full, dropping frame\n");
		dev_kfree_skb_any(skb);
		return;
	}

	skb_queue_tail(&priv->data_txq, skb);
	qlen = skb_queue_len(&priv->data_txq);
	wake_up(&priv->data_tx_wait);

	if (qlen >= SC23XX_SDIO_DATA_TXQ_HIGH)
		sc23xx_netif_tx(sdev, false);
}

static void sc23xx_sdio_tx_kick(struct sc23xx_dev *sdev)
{
	struct sc23xx_sdio *priv = container_of(sdev, struct sc23xx_sdio, sdev);

	wake_up(&priv->data_tx_wait);
}

static const struct sc23xx_bus_ops sc23xx_sdio_bus_ops = {
	.tx_cmd = sc23xx_sdio_tx_cmd,
	.tx_data = sc23xx_sdio_tx_data,
	.tx_kick = sc23xx_sdio_tx_kick,
};

static int sc23xx_sdio_open_channels(struct sc23xx_sdio *priv)
{
	priv->rx_cmd = sc23xx_sdio_open_chan(priv, priv->data->channels.rx_cmd,
					     &sc23xx_sdio_rx_cmd_ops);
	if (IS_ERR(priv->rx_cmd))
		return PTR_ERR(priv->rx_cmd);

	priv->rx_log = sc23xx_sdio_open_chan(priv, priv->data->channels.rx_log,
					     &sc23xx_sdio_rx_log_ops);
	if (IS_ERR(priv->rx_log))
		return PTR_ERR(priv->rx_log);

	priv->rx_data = sc23xx_sdio_open_chan(priv, priv->data->channels.rx_data,
					      &sc23xx_sdio_rx_data_ops);
	if (IS_ERR(priv->rx_data))
		return PTR_ERR(priv->rx_data);

	priv->tx_cmd = sc23xx_sdio_open_chan(priv, priv->data->channels.tx_cmd,
					     NULL);
	if (IS_ERR(priv->tx_cmd))
		return PTR_ERR(priv->tx_cmd);

	priv->tx_data = sc23xx_sdio_open_chan(priv, priv->data->channels.tx_data,
					      NULL);
	if (IS_ERR(priv->tx_data))
		return PTR_ERR(priv->tx_data);

	return 0;
}

static void sc23xx_sdio_close_chan(struct wcn_chan **chan)
{
	if (!IS_ERR_OR_NULL(*chan))
		wcn_chan_close(*chan);
	*chan = NULL;
}

static void sc23xx_sdio_close_channels(struct sc23xx_sdio *priv)
{
	sc23xx_sdio_close_chan(&priv->tx_data);
	sc23xx_sdio_close_chan(&priv->tx_cmd);
	sc23xx_sdio_close_chan(&priv->rx_data);
	sc23xx_sdio_close_chan(&priv->rx_log);
	sc23xx_sdio_close_chan(&priv->rx_cmd);
}

static int sc23xx_sdio_probe(struct platform_device *pdev)
{
	struct wcn_child_pdata *pdata = dev_get_platdata(&pdev->dev);
	struct sc23xx_sdio *priv;
	struct wcn_core *core;
	int ret;

	if (!pdata || !pdata->core || !pdata->data)
		return -ENODEV;

	core = pdata->core;

	priv = sc23xx_alloc_device(&pdev->dev, sizeof(*priv),
				   &sc23xx_sdio_bus_ops);
	if (IS_ERR(priv))
		return PTR_ERR(priv);

	priv->core = core;
	priv->data = pdata->data;
	priv->sdev.use_dma = false;
	priv->sdev.mc_filter_no_empty = priv->data->mc_filter_no_empty;
	platform_set_drvdata(pdev, priv);

	skb_queue_head_init(&priv->data_txq);
	init_waitqueue_head(&priv->data_tx_wait);
	priv->data_tx_thread = kthread_run(sc23xx_sdio_data_tx_thread, priv,
					   "sc23xx-tx");
	if (IS_ERR(priv->data_tx_thread)) {
		ret = PTR_ERR(priv->data_tx_thread);
		priv->data_tx_thread = NULL;
		dev_err(&pdev->dev, "failed to start TX thread: %d\n", ret);
		goto err_free;
	}

	ret = sc23xx_sdio_open_channels(priv);
	if (ret) {
		dev_err(&pdev->dev, "failed to open channels: %d\n", ret);
		goto err_channels;
	}

	ret = wcn_power_get(core, WCN_CHILD_WLAN);
	if (ret) {
		dev_err(&pdev->dev, "failed to power on WCN: %d\n", ret);
		goto err_channels;
	}

	ret = sc23xx_sync_version(&priv->sdev);
	if (ret) {
		dev_err(&pdev->dev, "version handshake failed: %d\n", ret);
		goto err_power;
	}

	ret = sc23xx_load_hw_param(&priv->sdev, "sprd/wifi_board_config.ini");
	if (ret) {
		dev_err(&pdev->dev, "failed to download config: %d\n", ret);
		goto err_power;
	}

	ret = sc23xx_register_device(&priv->sdev);
	if (ret)
		goto err_power;

	return 0;

err_power:
	wcn_power_put(core, WCN_CHILD_WLAN);
err_channels:
	kthread_stop(priv->data_tx_thread);
	sc23xx_sdio_close_channels(priv);
err_free:
	sc23xx_free_device(&priv->sdev);
	return ret;
}

static void sc23xx_sdio_remove(struct platform_device *pdev)
{
	struct sc23xx_sdio *priv = platform_get_drvdata(pdev);

	sc23xx_unregister_device(&priv->sdev);
	kthread_stop(priv->data_tx_thread);
	sc23xx_sdio_close_channels(priv);
	wcn_power_put(priv->core, WCN_CHILD_WLAN);
	sc23xx_free_device(&priv->sdev);
}

static const struct of_device_id sc23xx_sdio_of_match[] = {
	{ .compatible = "sprd,umw2652-wlan" },
	{ }
};
MODULE_DEVICE_TABLE(of, sc23xx_sdio_of_match);

static struct platform_driver sc23xx_sdio_driver = {
	.probe = sc23xx_sdio_probe,
	.remove = sc23xx_sdio_remove,
	.driver = {
		.name = "sc23xx-wlan-sdio",
		.of_match_table = sc23xx_sdio_of_match,
	},
};
module_platform_driver(sc23xx_sdio_driver);

MODULE_DESCRIPTION("Unisoc SC2355 SDIO wireless transport");
MODULE_LICENSE("GPL");
