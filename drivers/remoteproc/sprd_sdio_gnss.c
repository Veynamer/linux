// SPDX-License-Identifier: GPL-2.0-only
/* GNSS processor in a Marlin3 combo chip, controlled through WCN SDIO. */

#include <linux/firmware.h>
#include <linux/misc/unisoc-wcn.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/remoteproc.h>

struct sprd_sdio_gnss {
	struct wcn_core *core;
};

static int sprd_sdio_gnss_prepare(struct rproc *rproc)
{
	struct sprd_sdio_gnss *priv = rproc->priv;

	return wcn_gnss_prepare(priv->core);
}

static int sprd_sdio_gnss_unprepare(struct rproc *rproc)
{
	struct sprd_sdio_gnss *priv = rproc->priv;

	return wcn_gnss_unprepare(priv->core);
}

static int sprd_sdio_gnss_sanity(struct rproc *rproc, const struct firmware *fw)
{
	struct sprd_sdio_gnss *priv = rproc->priv;

	return wcn_gnss_validate_fw(priv->core, fw->data, fw->size);
}

static int sprd_sdio_gnss_load(struct rproc *rproc, const struct firmware *fw)
{
	struct sprd_sdio_gnss *priv = rproc->priv;

	return wcn_gnss_load(priv->core, fw->data, fw->size);
}

static int sprd_sdio_gnss_start(struct rproc *rproc)
{
	struct sprd_sdio_gnss *priv = rproc->priv;

	return wcn_gnss_start(priv->core);
}

static int sprd_sdio_gnss_stop(struct rproc *rproc)
{
	struct sprd_sdio_gnss *priv = rproc->priv;
	int ret;

	ret = wcn_gnss_stop(priv->core);
	if (ret)
		dev_warn(&rproc->dev, "GNSS stop failed: %d\n", ret);
	/* Unprepare retries shutdown and releases the vote even after an I/O error. */
	return 0;
}

static const struct rproc_ops sprd_sdio_gnss_ops = {
	.prepare = sprd_sdio_gnss_prepare,
	.unprepare = sprd_sdio_gnss_unprepare,
	.sanity_check = sprd_sdio_gnss_sanity,
	.load = sprd_sdio_gnss_load,
	.start = sprd_sdio_gnss_start,
	.stop = sprd_sdio_gnss_stop,
};

static int sprd_sdio_gnss_probe(struct platform_device *pdev)
{
	const struct wcn_child_pdata *pdata = dev_get_platdata(&pdev->dev);
	struct sprd_sdio_gnss *priv;
	const char *firmware;
	struct rproc *rproc;
	int ret;

	if (!pdata || !pdata->core || !pdata->data || !pdata->data->firmware_name)
		return -ENODEV;
	firmware = pdata->data->firmware_name;
	device_property_read_string(&pdev->dev, "firmware-name", &firmware);
	rproc = rproc_alloc(&pdev->dev, dev_name(&pdev->dev), &sprd_sdio_gnss_ops,
			    firmware, sizeof(*priv));
	if (!rproc)
		return -ENOMEM;
	rproc->auto_boot = false;
	priv = rproc->priv;
	priv->core = pdata->core;
	platform_set_drvdata(pdev, rproc);
	ret = rproc_add(rproc);
	if (ret)
		rproc_free(rproc);
	return ret;
}

static void sprd_sdio_gnss_remove(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);
	struct sprd_sdio_gnss *priv = rproc->priv;

	/* Managed consumers are unbound before this supplier is removed. */
	rproc_del(rproc);
	wcn_gnss_unprepare(priv->core);
	rproc_free(rproc);
}

static const struct of_device_id sprd_sdio_gnss_of_match[] = {
	{ .compatible = "unisoc,marlin3-gnss" },
	{ }
};
MODULE_DEVICE_TABLE(of, sprd_sdio_gnss_of_match);

static struct platform_driver sprd_sdio_gnss_driver = {
	.probe = sprd_sdio_gnss_probe,
	.remove = sprd_sdio_gnss_remove,
	.driver = {
		.name = "sprd-sdio-gnss",
		.of_match_table = sprd_sdio_gnss_of_match,
	},
};
module_platform_driver(sprd_sdio_gnss_driver);

MODULE_DESCRIPTION("Unisoc Marlin3 SDIO GNSS remote processor");
MODULE_LICENSE("GPL");
