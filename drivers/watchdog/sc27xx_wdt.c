// SPDX-License-Identifier: GPL-2.0-only
/*
 * Spreadtrum SC27xx PMIC watchdog driver
 * Copyright (C) 2026
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/watchdog.h>

#define SC27XX_WDT_LOAD_LOW		0x0
#define SC27XX_WDT_LOAD_HIGH		0x4
#define SC27XX_WDT_CTRL			0x8
#define SC27XX_WDT_INT_CLR		0xc
#define SC27XX_WDT_INT_RAW		0x10
#define SC27XX_WDT_CNT_LOW		0x18
#define SC27XX_WDT_CNT_HIGH		0x1c
#define SC27XX_WDT_LOCK			0x20
#define SC27XX_WDT_IRQ_LOAD_LOW		0x2c
#define SC27XX_WDT_IRQ_LOAD_HIGH	0x30

#define SC27XX_WDT_INT_EN_BIT		BIT(0)
#define SC27XX_WDT_CNT_EN_BIT		BIT(1)
#define SC27XX_WDT_NEW_VER_EN		BIT(2)
#define SC27XX_WDT_RST_EN_BIT		BIT(3)

#define SC27XX_WDT_INT_CLEAR_BIT	BIT(0)

#define SC27XX_WDT_INT_RAW_BIT		BIT(0)
#define SC27XX_WDT_LD_BUSY_BIT		BIT(4)

#define SC27XX_WDT_CNT_STEP		32768

#define SC27XX_WDT_UNLOCK_KEY		0xe551
#define SC27XX_WDT_MIN_TIMEOUT		3
#define SC27XX_WDT_MAX_TIMEOUT		60

#define SC27XX_WDT_CNT_HIGH_SHIFT	16
#define SC27XX_WDT_LOW_VALUE_MASK	GENMASK(15, 0)
#define SC27XX_WDT_LOAD_TIMEOUT		1000

struct sc27xx_wdt {
	struct regmap *regmap;
	u32 base;
	struct watchdog_device wdd;
};

static inline struct sc27xx_wdt *to_sc27xx_wdt(struct watchdog_device *wdd)
{
	return container_of(wdd, struct sc27xx_wdt, wdd);
}

static inline int sc27xx_wdt_unlock(struct sc27xx_wdt *wdt)
{
	return regmap_write(wdt->regmap, wdt->base + SC27XX_WDT_LOCK,
			    SC27XX_WDT_UNLOCK_KEY);
}

static inline int sc27xx_wdt_lock(struct sc27xx_wdt *wdt)
{
	return regmap_write(wdt->regmap, wdt->base + SC27XX_WDT_LOCK, 0x0);
}

static int sc27xx_wdt_load_value(struct sc27xx_wdt *wdt, u32 timeout,
				 u32 pretimeout)
{
	u32 val, delay_cnt = 0;
	u32 tmr_step = timeout * SC27XX_WDT_CNT_STEP;
	u32 prtmr_step = pretimeout * SC27XX_WDT_CNT_STEP;
	int ret;

	do {
		ret = regmap_read(wdt->regmap, wdt->base + SC27XX_WDT_INT_RAW,
				  &val);
		if (ret)
			return ret;
		if (!(val & SC27XX_WDT_LD_BUSY_BIT))
			break;

		usleep_range(10, 100);
	} while (delay_cnt++ < SC27XX_WDT_LOAD_TIMEOUT);

	if (delay_cnt >= SC27XX_WDT_LOAD_TIMEOUT)
		return -EBUSY;

	ret = sc27xx_wdt_unlock(wdt);
	if (ret)
		return ret;

	ret = regmap_write(wdt->regmap, wdt->base + SC27XX_WDT_LOAD_HIGH,
			   (tmr_step >> SC27XX_WDT_CNT_HIGH_SHIFT) &
			   SC27XX_WDT_LOW_VALUE_MASK);
	if (ret)
		goto lock;
	ret = regmap_write(wdt->regmap, wdt->base + SC27XX_WDT_LOAD_LOW,
			   tmr_step & SC27XX_WDT_LOW_VALUE_MASK);
	if (ret)
		goto lock;
	ret = regmap_write(wdt->regmap, wdt->base + SC27XX_WDT_IRQ_LOAD_HIGH,
			   (prtmr_step >> SC27XX_WDT_CNT_HIGH_SHIFT) &
			   SC27XX_WDT_LOW_VALUE_MASK);
	if (ret)
		goto lock;
	ret = regmap_write(wdt->regmap, wdt->base + SC27XX_WDT_IRQ_LOAD_LOW,
			   prtmr_step & SC27XX_WDT_LOW_VALUE_MASK);
	if (ret)
		goto lock;

	ret = sc27xx_wdt_lock(wdt);

	return ret;
lock:
	sc27xx_wdt_lock(wdt);

	return ret;
}

static int sc27xx_wdt_start(struct watchdog_device *wdd)
{
	struct sc27xx_wdt *wdt = to_sc27xx_wdt(wdd);
	u32 val;
	int ret;

	ret = sc27xx_wdt_load_value(wdt, wdd->timeout, wdd->pretimeout);
	if (ret)
		return ret;

	ret = sc27xx_wdt_unlock(wdt);
	if (ret)
		return ret;

	val = SC27XX_WDT_CNT_EN_BIT | SC27XX_WDT_INT_EN_BIT |
	SC27XX_WDT_RST_EN_BIT;
	ret = regmap_update_bits(wdt->regmap, wdt->base + SC27XX_WDT_CTRL,
				 val, val);
	if (ret)
		goto lock;

	ret = sc27xx_wdt_lock(wdt);
	if (ret)
		return ret;

	set_bit(WDOG_HW_RUNNING, &wdd->status);

	return 0;
lock:
	sc27xx_wdt_lock(wdt);

	return ret;
}

static int sc27xx_wdt_stop(struct watchdog_device *wdd)
{
	struct sc27xx_wdt *wdt = to_sc27xx_wdt(wdd);
	u32 val = SC27XX_WDT_CNT_EN_BIT | SC27XX_WDT_RST_EN_BIT |
	SC27XX_WDT_INT_EN_BIT;
	int ret;

	ret = sc27xx_wdt_unlock(wdt);
	if (ret)
		return ret;

	ret = regmap_update_bits(wdt->regmap, wdt->base + SC27XX_WDT_CTRL,
				 val, 0);
	if (ret)
		goto lock;

	ret = sc27xx_wdt_lock(wdt);

	return ret;
lock:
	sc27xx_wdt_lock(wdt);

	return ret;
}

static int sc27xx_wdt_set_timeout(struct watchdog_device *wdd, u32 timeout)
{
	struct sc27xx_wdt *wdt = to_sc27xx_wdt(wdd);

	if (timeout == wdd->timeout)
		return 0;

	wdd->timeout = timeout;

	return sc27xx_wdt_load_value(wdt, timeout, wdd->pretimeout);
}

static int sc27xx_wdt_ping(struct watchdog_device *wdd)
{
	struct sc27xx_wdt *wdt = to_sc27xx_wdt(wdd);

	return sc27xx_wdt_load_value(wdt, wdd->timeout, wdd->pretimeout);
}

static int sc27xx_wdt_set_pretimeout(struct watchdog_device *wdd,
				     u32 new_pretimeout)
{
	struct sc27xx_wdt *wdt = to_sc27xx_wdt(wdd);

	if (new_pretimeout < wdd->min_timeout)
		return -EINVAL;

	wdd->pretimeout = new_pretimeout;

	return sc27xx_wdt_load_value(wdt, wdd->timeout, new_pretimeout);
}

static const struct watchdog_info sc27xx_wdt_info = {
	.options = WDIOF_SETTIMEOUT | WDIOF_PRETIMEOUT | WDIOF_KEEPALIVEPING |
		   WDIOF_MAGICCLOSE,
	.identity = "Spreadtrum SC27xx PMIC Watchdog",
};

static const struct watchdog_ops sc27xx_wdt_ops = {
	.owner = THIS_MODULE,
	.start = sc27xx_wdt_start,
	.stop = sc27xx_wdt_stop,
	.set_timeout = sc27xx_wdt_set_timeout,
	.set_pretimeout = sc27xx_wdt_set_pretimeout,
	.ping = sc27xx_wdt_ping,
};

static int sc27xx_wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sc27xx_wdt *wdt;
	int ret;

	wdt = devm_kzalloc(dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;

	wdt->regmap = dev_get_regmap(dev->parent, NULL);
	if (!wdt->regmap)
		return -ENODEV;

	ret = device_property_read_u32(dev, "reg", &wdt->base);
	if (ret)
		return ret;

	wdt->wdd.info = &sc27xx_wdt_info;
	wdt->wdd.ops = &sc27xx_wdt_ops;
	wdt->wdd.parent = dev;
	wdt->wdd.min_timeout = SC27XX_WDT_MIN_TIMEOUT;
	wdt->wdd.max_timeout = SC27XX_WDT_MAX_TIMEOUT;
	wdt->wdd.timeout = SC27XX_WDT_MAX_TIMEOUT;

	ret = sc27xx_wdt_stop(&wdt->wdd);
	if (ret)
		return ret;

	watchdog_init_timeout(&wdt->wdd, 0, dev);
	watchdog_set_drvdata(&wdt->wdd, wdt);
	platform_set_drvdata(pdev, wdt);

	return devm_watchdog_register_device(dev, &wdt->wdd);
}

static const struct of_device_id sc27xx_wdt_of_match[] = {
	{ .compatible = "sprd,sc2730-wdt", },
	{ .compatible = "sprd,sc27xx-wdt", },
	{ }
};
MODULE_DEVICE_TABLE(of, sc27xx_wdt_of_match);

static struct platform_driver sc27xx_wdt_driver = {
	.probe = sc27xx_wdt_probe,
	.driver = {
		.name = "sc27xx-wdt",
		.of_match_table = sc27xx_wdt_of_match,
	},
};
module_platform_driver(sc27xx_wdt_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Spreadtrum SC27xx PMIC watchdog driver");
