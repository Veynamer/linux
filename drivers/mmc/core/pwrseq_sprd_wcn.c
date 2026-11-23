// SPDX-License-Identifier: GPL-2.0-only
/*
 * pwrseq_sprd_wcn.c - power sequence for Unisoc Marlin3lite WCN
 *
 * Copyright (C) 2026 Veynamer <veyr1thalone@gmail.com>
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/mfd/syscon.h>
#include <linux/mmc/host.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#include "pwrseq.h"

struct mmc_pwrseq_sprd_wcn {
	struct mmc_pwrseq pwrseq;
	struct gpio_desc *enable_gpio;
	struct gpio_desc *reset_gpio;
	struct regulator *dvdd12;
	struct regulator *avdd12;
	struct regulator *avdd33;
	struct regulator *dcxo18;
	struct clk *clk_32k;
	struct clk *clk_source;
	struct clk *clk_enable;
	struct regmap *pmic;
	u32 avdd12_parent_bound_chip[4];
	u32 avdd12_bound_wbreq[4];
	u32 avdd33_bound_wbreq[4];
	bool has_avdd12_parent_bound_chip;
	bool has_avdd12_bound_wbreq;
	bool has_avdd33_bound_wbreq;
	bool dcxo18_enabled;
	bool dvdd12_enabled;
	bool avdd12_enabled;
	bool avdd33_enabled;
	bool clk_32k_enabled;
	bool clk_enable_enabled;
	unsigned int post_power_on_delay_ms;
	unsigned int power_off_delay_us;
};

#define to_pwrseq_sprd_wcn(p) container_of(p, struct mmc_pwrseq_sprd_wcn, pwrseq)

static int mmc_pwrseq_sprd_wcn_pmic_write(struct mmc_pwrseq_sprd_wcn *pwrseq,
					  const u32 *cfg, bool bound)
{
	if (!pwrseq->pmic || !cfg[0])
		return 0;

	return regmap_update_bits(pwrseq->pmic, cfg[0], cfg[1],
				  bound ? cfg[3] : cfg[2]);
}

static int mmc_pwrseq_sprd_wcn_regulator_enable(struct device *dev,
						struct regulator *regulator,
						int min_uV, int max_uV)
{
	int ret;

	if (!regulator)
		return 0;

	ret = regulator_set_voltage(regulator, min_uV, max_uV);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set regulator voltage\n");

	ret = regulator_enable(regulator);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable regulator\n");

	return 0;
}

static void mmc_pwrseq_sprd_wcn_regulator_disable(struct regulator *regulator,
						  bool *enabled)
{
	if (!regulator || !*enabled)
		return;

	regulator_disable(regulator);
	*enabled = false;
}

static struct regmap *mmc_pwrseq_sprd_wcn_find_regmap(struct device_node *np)
{
	struct platform_device *pdev;
	struct device *spidev;
	struct regmap *regmap;

	pdev = of_find_device_by_node(np);
	if (pdev) {
		regmap = dev_get_regmap(&pdev->dev, NULL);
		if (!regmap)
			regmap = dev_get_regmap(pdev->dev.parent, NULL);
		put_device(&pdev->dev);
		if (regmap)
			return regmap;
	}

	spidev = bus_find_device_by_of_node(&spi_bus_type, np);
	if (!spidev)
		return NULL;

	regmap = dev_get_regmap(spidev, NULL);
	put_device(spidev);

	return regmap;
}

static int mmc_pwrseq_sprd_wcn_get_pmic(struct device *dev,
					struct mmc_pwrseq_sprd_wcn *pwrseq)
{
	struct device_node *np;
	struct regmap *regmap;

	np = of_parse_phandle(dev->of_node, "sprd,pmic", 0);
	if (!np)
		np = of_find_compatible_node(NULL, NULL, "sprd,sc2730");
	if (!np)
		np = of_find_compatible_node(NULL, NULL, "sprd,sc27xx-syscon");
	if (!np)
		np = of_find_compatible_node(NULL, NULL, "sprd,ump962x-syscon");
	if (!np)
		return -ENODEV;

	regmap = mmc_pwrseq_sprd_wcn_find_regmap(np);
	of_node_put(np);
	if (!regmap)
		return -EPROBE_DEFER;

	pwrseq->pmic = regmap;

	return 0;
}

static int mmc_pwrseq_sprd_wcn_parse_pmic(struct device *dev,
					  struct mmc_pwrseq_sprd_wcn *pwrseq)
{
	struct device_node *np = dev->of_node;
	int ret;

	pwrseq->has_avdd12_parent_bound_chip =
		!of_property_read_u32_array(np, "sprd,avdd12-parent-bound-chip",
					    pwrseq->avdd12_parent_bound_chip,
					    ARRAY_SIZE(pwrseq->avdd12_parent_bound_chip));
	pwrseq->has_avdd12_bound_wbreq =
		!of_property_read_u32_array(np, "sprd,avdd12-bound-wbreq",
					    pwrseq->avdd12_bound_wbreq,
					    ARRAY_SIZE(pwrseq->avdd12_bound_wbreq));
	pwrseq->has_avdd33_bound_wbreq =
		!of_property_read_u32_array(np, "sprd,avdd33-bound-wbreq",
					    pwrseq->avdd33_bound_wbreq,
					    ARRAY_SIZE(pwrseq->avdd33_bound_wbreq));

	if (!pwrseq->has_avdd12_parent_bound_chip &&
	    !pwrseq->has_avdd12_bound_wbreq &&
	    !pwrseq->has_avdd33_bound_wbreq)
		return 0;

	ret = mmc_pwrseq_sprd_wcn_get_pmic(dev, pwrseq);
	if (ret == -EPROBE_DEFER)
		return ret;

	if (ret)
		dev_warn(dev, "PMIC regmap not available: %d\n", ret);

	return 0;
}

static void mmc_pwrseq_sprd_wcn_rollback(struct mmc_pwrseq_sprd_wcn *pwrseq)
{
	if (pwrseq->has_avdd33_bound_wbreq)
		mmc_pwrseq_sprd_wcn_pmic_write(pwrseq,
					       pwrseq->avdd33_bound_wbreq,
					       false);

	if (pwrseq->has_avdd12_bound_wbreq)
		mmc_pwrseq_sprd_wcn_pmic_write(pwrseq,
					       pwrseq->avdd12_bound_wbreq,
					       false);

	if (pwrseq->has_avdd12_parent_bound_chip)
		mmc_pwrseq_sprd_wcn_pmic_write(pwrseq,
					       pwrseq->avdd12_parent_bound_chip,
					       true);

	mmc_pwrseq_sprd_wcn_regulator_disable(pwrseq->avdd33,
					      &pwrseq->avdd33_enabled);
	mmc_pwrseq_sprd_wcn_regulator_disable(pwrseq->dcxo18,
					      &pwrseq->dcxo18_enabled);

	if (pwrseq->clk_enable_enabled) {
		clk_disable_unprepare(pwrseq->clk_enable);
		pwrseq->clk_enable_enabled = false;
	}

	if (pwrseq->clk_32k_enabled) {
		clk_disable_unprepare(pwrseq->clk_32k);
		pwrseq->clk_32k_enabled = false;
	}

	gpiod_set_value_cansleep(pwrseq->enable_gpio, 0);
	mmc_pwrseq_sprd_wcn_regulator_disable(pwrseq->dvdd12,
					      &pwrseq->dvdd12_enabled);
	mmc_pwrseq_sprd_wcn_regulator_disable(pwrseq->avdd12,
					      &pwrseq->avdd12_enabled);
	gpiod_set_value_cansleep(pwrseq->reset_gpio, 1);
}

static void mmc_pwrseq_sprd_wcn_pre_power_on(struct mmc_host *host)
{
	struct mmc_pwrseq_sprd_wcn *pwrseq = to_pwrseq_sprd_wcn(host->pwrseq);
	struct device *dev = pwrseq->pwrseq.dev;
	int ret;

	if (pwrseq->has_avdd12_parent_bound_chip) {
		ret = mmc_pwrseq_sprd_wcn_pmic_write(pwrseq,
						     pwrseq->avdd12_parent_bound_chip,
						     false);
		if (ret) {
			dev_err(dev, "failed to update AVDD12 parent PMIC bits: %d\n",
				ret);
			goto err;
		}
	}

	ret = mmc_pwrseq_sprd_wcn_regulator_enable(dev, pwrseq->dcxo18,
						   1800000, 1800000);
	if (ret) {
		dev_err(dev, "failed to enable DCXO supply: %d\n", ret);
		goto err;
	}
	pwrseq->dcxo18_enabled = true;

	if (pwrseq->clk_32k) {
		ret = clk_prepare_enable(pwrseq->clk_32k);
		if (ret) {
			dev_err(dev, "failed to enable 32 kHz clock: %d\n", ret);
			goto err;
		}
		pwrseq->clk_32k_enabled = true;
	}

	if (pwrseq->clk_enable) {
		ret = clk_prepare_enable(pwrseq->clk_enable);
		if (ret) {
			dev_err(dev, "failed to enable clock gate: %d\n", ret);
			goto err;
		}
		pwrseq->clk_enable_enabled = true;
	}

	ret = mmc_pwrseq_sprd_wcn_regulator_enable(dev, pwrseq->dvdd12,
						   200000, 1200000);
	if (ret) {
		dev_err(dev, "failed to enable DVDD12 supply: %d\n", ret);
		goto err;
	}
	if (pwrseq->dvdd12)
		pwrseq->dvdd12_enabled = true;

	gpiod_set_value_cansleep(pwrseq->reset_gpio, 1);
	gpiod_set_value_cansleep(pwrseq->enable_gpio, 0);
	mdelay(1);
	gpiod_set_value_cansleep(pwrseq->enable_gpio, 1);
	mdelay(1);

	usleep_range(4000, 5000);
	gpiod_set_value_cansleep(pwrseq->reset_gpio, 0);

	usleep_range(4000, 5000);
	ret = mmc_pwrseq_sprd_wcn_regulator_enable(dev, pwrseq->avdd12,
						   1200000, 1200000);
	if (ret) {
		dev_err(dev, "failed to enable AVDD12 supply: %d\n", ret);
		goto err;
	}
	pwrseq->avdd12_enabled = true;

	if (pwrseq->has_avdd12_bound_wbreq) {
		ret = mmc_pwrseq_sprd_wcn_pmic_write(pwrseq,
						     pwrseq->avdd12_bound_wbreq,
						     true);
		if (ret) {
			dev_err(dev, "failed to update AVDD12 PMIC bits: %d\n",
				ret);
			goto err;
		}
	}

	usleep_range(50, 60);
	ret = mmc_pwrseq_sprd_wcn_regulator_enable(dev, pwrseq->avdd33,
						   3300000, 3300000);
	if (ret) {
		dev_err(dev, "failed to enable AVDD33 supply: %d\n", ret);
		goto err;
	}
	pwrseq->avdd33_enabled = true;

	if (pwrseq->has_avdd33_bound_wbreq) {
		ret = mmc_pwrseq_sprd_wcn_pmic_write(pwrseq,
						     pwrseq->avdd33_bound_wbreq,
						     true);
		if (ret) {
			dev_err(dev, "failed to update AVDD33 PMIC bits: %d\n",
				ret);
			goto err;
		}
	}

	return;

err:
	mmc_pwrseq_sprd_wcn_rollback(pwrseq);
}

static void mmc_pwrseq_sprd_wcn_post_power_on(struct mmc_host *host)
{
	struct mmc_pwrseq_sprd_wcn *pwrseq = to_pwrseq_sprd_wcn(host->pwrseq);

	if (pwrseq->post_power_on_delay_ms)
		msleep(pwrseq->post_power_on_delay_ms);
}

static void mmc_pwrseq_sprd_wcn_power_off(struct mmc_host *host)
{
	struct mmc_pwrseq_sprd_wcn *pwrseq = to_pwrseq_sprd_wcn(host->pwrseq);

	if (pwrseq->has_avdd12_bound_wbreq)
		mmc_pwrseq_sprd_wcn_pmic_write(pwrseq,
					       pwrseq->avdd12_bound_wbreq,
					       false);

	if (pwrseq->has_avdd33_bound_wbreq)
		mmc_pwrseq_sprd_wcn_pmic_write(pwrseq,
					       pwrseq->avdd33_bound_wbreq,
					       false);

	if (pwrseq->has_avdd12_parent_bound_chip)
		mmc_pwrseq_sprd_wcn_pmic_write(pwrseq,
					       pwrseq->avdd12_parent_bound_chip,
					       true);

	mmc_pwrseq_sprd_wcn_regulator_disable(pwrseq->avdd33,
					      &pwrseq->avdd33_enabled);
	mmc_pwrseq_sprd_wcn_regulator_disable(pwrseq->dcxo18,
					      &pwrseq->dcxo18_enabled);

	if (pwrseq->clk_enable_enabled) {
		clk_disable_unprepare(pwrseq->clk_enable);
		pwrseq->clk_enable_enabled = false;
	}

	if (pwrseq->clk_32k_enabled) {
		clk_disable_unprepare(pwrseq->clk_32k);
		pwrseq->clk_32k_enabled = false;
	}

	gpiod_set_value_cansleep(pwrseq->enable_gpio, 0);
	mmc_pwrseq_sprd_wcn_regulator_disable(pwrseq->dvdd12,
					      &pwrseq->dvdd12_enabled);
	mmc_pwrseq_sprd_wcn_regulator_disable(pwrseq->avdd12,
					      &pwrseq->avdd12_enabled);
	gpiod_set_value_cansleep(pwrseq->reset_gpio, 1);

	if (pwrseq->power_off_delay_us)
		udelay(pwrseq->power_off_delay_us);
}

static const struct mmc_pwrseq_ops mmc_pwrseq_sprd_wcn_ops = {
	.pre_power_on = mmc_pwrseq_sprd_wcn_pre_power_on,
	.post_power_on = mmc_pwrseq_sprd_wcn_post_power_on,
	.power_off = mmc_pwrseq_sprd_wcn_power_off,
};

static int mmc_pwrseq_sprd_wcn_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mmc_pwrseq_sprd_wcn *pwrseq;
	u32 delay;
	int ret;

	pwrseq = devm_kzalloc(dev, sizeof(*pwrseq), GFP_KERNEL);
	if (!pwrseq)
		return -ENOMEM;

	pwrseq->enable_gpio = devm_gpiod_get(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(pwrseq->enable_gpio))
		return dev_err_probe(dev, PTR_ERR(pwrseq->enable_gpio),
				     "failed to get enable GPIO\n");

	pwrseq->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(pwrseq->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(pwrseq->reset_gpio),
				     "failed to get reset GPIO\n");

	pwrseq->dvdd12 = devm_regulator_get_optional(dev, "dvdd12");
	if (IS_ERR(pwrseq->dvdd12)) {
		if (PTR_ERR(pwrseq->dvdd12) != -ENODEV)
			return dev_err_probe(dev, PTR_ERR(pwrseq->dvdd12),
					     "failed to get DVDD12 supply\n");
		pwrseq->dvdd12 = NULL;
	}

	pwrseq->avdd12 = devm_regulator_get(dev, "avdd12");
	if (IS_ERR(pwrseq->avdd12))
		return dev_err_probe(dev, PTR_ERR(pwrseq->avdd12),
				     "failed to get AVDD12 supply\n");

	pwrseq->avdd33 = devm_regulator_get(dev, "avdd33");
	if (IS_ERR(pwrseq->avdd33))
		return dev_err_probe(dev, PTR_ERR(pwrseq->avdd33),
				     "failed to get AVDD33 supply\n");

	pwrseq->dcxo18 = devm_regulator_get(dev, "dcxo18");
	if (IS_ERR(pwrseq->dcxo18))
		return dev_err_probe(dev, PTR_ERR(pwrseq->dcxo18),
				     "failed to get DCXO18 supply\n");

	pwrseq->clk_32k = devm_clk_get(dev, "clk_32k");
	if (IS_ERR(pwrseq->clk_32k))
		return dev_err_probe(dev, PTR_ERR(pwrseq->clk_32k),
				     "failed to get 32 kHz clock\n");

	pwrseq->clk_source = devm_clk_get(dev, "source");
	if (IS_ERR(pwrseq->clk_source))
		return dev_err_probe(dev, PTR_ERR(pwrseq->clk_source),
				     "failed to get clock source\n");

	pwrseq->clk_enable = devm_clk_get(dev, "enable");
	if (IS_ERR(pwrseq->clk_enable))
		return dev_err_probe(dev, PTR_ERR(pwrseq->clk_enable),
				     "failed to get clock gate\n");

	ret = clk_set_parent(pwrseq->clk_32k, pwrseq->clk_source);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set clock parent\n");

	ret = clk_set_rate(pwrseq->clk_32k, 32768);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set clock rate\n");

	ret = mmc_pwrseq_sprd_wcn_parse_pmic(dev, pwrseq);
	if (ret)
		return ret;

	if (!device_property_read_u32(dev, "post-power-on-delay-ms", &delay))
		pwrseq->post_power_on_delay_ms = delay;

	if (!device_property_read_u32(dev, "power-off-delay-us", &delay))
		pwrseq->power_off_delay_us = delay;

	pwrseq->pwrseq.dev = dev;
	pwrseq->pwrseq.ops = &mmc_pwrseq_sprd_wcn_ops;
	pwrseq->pwrseq.owner = THIS_MODULE;
	platform_set_drvdata(pdev, pwrseq);

	return mmc_pwrseq_register(&pwrseq->pwrseq);
}

static void mmc_pwrseq_sprd_wcn_remove(struct platform_device *pdev)
{
	struct mmc_pwrseq_sprd_wcn *pwrseq = platform_get_drvdata(pdev);

	mmc_pwrseq_unregister(&pwrseq->pwrseq);
}

static const struct of_device_id mmc_pwrseq_sprd_wcn_of_match[] = {
	{ .compatible = "unisoc,marlin3lite-pwrseq" },
	{ }
};
MODULE_DEVICE_TABLE(of, mmc_pwrseq_sprd_wcn_of_match);

static struct platform_driver mmc_pwrseq_sprd_wcn_driver = {
	.probe = mmc_pwrseq_sprd_wcn_probe,
	.remove = mmc_pwrseq_sprd_wcn_remove,
	.driver = {
		.name = "mmc-pwrseq-sprd-wcn",
		.of_match_table = mmc_pwrseq_sprd_wcn_of_match,
	},
};
module_platform_driver(mmc_pwrseq_sprd_wcn_driver);

MODULE_DESCRIPTION("Unisoc Marlin3lite WCN power sequence");
MODULE_AUTHOR("Veynamer <veyr1thalone@gmail.com>");
MODULE_LICENSE("GPL");
